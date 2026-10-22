// SPDX-License-Identifier: GPL-2.0+
/*
 * 8250_dma.c - DMA Engine API support for 8250.c
 *
 * Copyright (C) 2013 Intel Corporation
 */
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/dma-mapping.h>

#include "8250.h"

static void serial8250_tx_dma_complete(struct uart_8250_port *p)
{
	struct uart_8250_dma	*dma = p->dma;
	struct tty_port		*tport = &p->port.state->port;
	int		ret;

	dma_sync_single_for_cpu(dmaengine_get_dma_device(dma->txchan), dma->tx_addr,
				UART_XMIT_SIZE, DMA_TO_DEVICE);

	dma->tx_running = 0;

	if (dma->tx_flushed) {
		p->port.icount.tx += dma->tx_size;
		uart_xmit_unblock(&p->port);
	} else {
		uart_xmit_advance(&p->port, dma->tx_size);
	}

	if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
		uart_write_wakeup(&p->port);

	ret = serial8250_tx_dma(p);
	if (ret || !dma->tx_running)
		serial8250_set_THRI(p);
}

static void __dma_tx_complete(void *param)
{
	struct uart_8250_port *p = param;
	struct uart_8250_dma *dma = p->dma;

	guard(uart_port_lock_irqsave)(&p->port);

	if (serial8250_tx_dma_blocked(dma) || !dma->tx_running)
		return;
	serial8250_tx_dma_complete(p);
}

static void serial8250_tx_dma_flush_work(struct work_struct *work)
{
	struct uart_8250_dma *dma = container_of(work, struct uart_8250_dma, tx_flush_work);
	struct uart_8250_port *p = dma->port;

	/* Neither the channel nor its buffer may be reused before this returns. */
	dmaengine_synchronize(dma->txchan);

	guard(serial8250_rpm)(p);

	guard(uart_port_lock_irqsave)(&p->port);

	dma->tx_terminating = false;
	dma->tx_size = 0;
	if (dma->shutting_down)
		return;
	dma->tx_complete(p);
}

void serial8250_tx_dma_sync_buffer(struct uart_8250_port *p,
				   struct scatterlist *sgl, unsigned int nents)
{
	struct uart_8250_dma *dma = p->dma;
	struct device *dev = dmaengine_get_dma_device(dma->txchan);
	struct scatterlist *sg;
	unsigned int i;

	/* DMA reads the FIFO directly; its occupied ranges remain reserved. */
	for_each_sg(sgl, sg, nents, i) {
		size_t offset = sg_dma_address(sg) - dma->tx_addr;

		dma_sync_single_range_for_device(dev, dma->tx_addr, offset,
						 sg_dma_len(sg), DMA_TO_DEVICE);
	}
}
EXPORT_SYMBOL_GPL(serial8250_tx_dma_sync_buffer);

static void serial8250_rx_dma_copy(struct uart_8250_port *p,
				   const struct dma_tx_state *state)
{
	struct uart_8250_dma	*dma = p->dma;
	struct tty_port		*tty_port = &p->port.state->port;
	size_t			count = dma->rx_size - state->residue;

	if (tty_insert_flip_string(tty_port, dma->rx_buf, count) != count)
		p->port.icount.buf_overrun++;
	p->port.icount.rx += count;

	tty_flip_buffer_push(tty_port);
}

static void serial8250_rx_dma_recover(struct uart_8250_port *p)
{
	struct uart_8250_dma *dma = p->dma;

	/* Keep PIO and new descriptors out until the worker stops the channel. */
	dma->rx_err = true;
	dma->rx_terminating = true;
	serial8250_out_IER(p, p->ier);
	schedule_work(&dma->rx_flush_work);
}

static void dma_rx_complete(void *param)
{
	struct uart_8250_port *p = param;
	struct uart_8250_dma *dma = p->dma;
	struct dma_tx_state state;
	enum dma_status status;

	guard(uart_port_lock_irqsave)(&p->port);

	if (dma->shutting_down || dma->rx_terminating)
		return;
	if (dma->rx_running) {
		status = dmaengine_tx_status(dma->rxchan, dma->rx_cookie, &state);
		if (status == DMA_ERROR) {
			serial8250_rx_dma_recover(p);
			return;
		}
		if (status != DMA_COMPLETE)
			return;
		serial8250_rx_dma_copy(p, &state);
		dma->rx_running = 0;
	}

	/* Preserve LSR read side effects when stop_rx() has disabled resubmission. */
	if ((serial_lsr_in(p) & UART_LSR_DR) && (p->ier & UART_IER_RDI))
		dma->rx_dma(p);
}

static void serial8250_rx_dma_flush_work(struct work_struct *work)
{
	struct uart_8250_dma *dma = container_of(work, struct uart_8250_dma, rx_flush_work);
	struct uart_8250_port *p = dma->port;
	bool recover;
	int ret;

	/* All packed flags share the port lock, including this worker snapshot. */
	scoped_guard(uart_port_lock_irqsave, &p->port)
		recover = dma->rx_err;

	if (recover) {
		ret = dmaengine_terminate_sync(dma->rxchan);
		if (ret) {
			/* Retain ownership and keep RX masked if DMA cannot be stopped. */
			dev_err(p->port.dev, "failed to stop RX DMA: %d\n", ret);
			return;
		}
	} else {
		dmaengine_synchronize(dma->rxchan);
	}

	guard(serial8250_rpm)(p);

	scoped_guard(uart_port_lock_irqsave, &p->port) {
		dma->rx_terminating = false;
		dma->rx_running = 0;
		if (dma->shutting_down)
			return;

		if (!recover)
			return;

		/* Apply the current enables, including any intervening stop_rx(). */
		serial8250_out_IER(p, p->ier);
	}
	dev_warn(p->port.dev, "RX DMA failed, using PIO\n");
}

int serial8250_tx_dma(struct uart_8250_port *p)
{
	struct uart_8250_dma		*dma = p->dma;
	struct tty_port			*tport = &p->port.state->port;
	struct dma_async_tx_descriptor	*desc;
	struct uart_port		*up = &p->port;
	struct scatterlist		*sg;
	struct scatterlist		sgl[2];
	int i;
	int ret;

	if (serial8250_tx_dma_blocked(dma))
		return 0;

	if (dma->tx_running) {
		if (up->x_char) {
			dmaengine_pause(dma->txchan);
			uart_xchar_out(up, UART_TX);
			dmaengine_resume(dma->txchan);
		}
		return 0;
	} else if (up->x_char) {
		uart_xchar_out(up, UART_TX);
	}

	if (uart_tx_stopped(&p->port) || kfifo_is_empty(&tport->xmit_fifo)) {
		/* We have been called from __dma_tx_complete() */
		return 0;
	}

	sg_init_table(sgl, ARRAY_SIZE(sgl));

	ret = kfifo_dma_out_prepare_mapped(&tport->xmit_fifo, sgl, ARRAY_SIZE(sgl),
					   UART_XMIT_SIZE, dma->tx_addr);

	dma->tx_size = 0;

	for_each_sg(sgl, sg, ret, i)
		dma->tx_size += sg_dma_len(sg);

	serial8250_do_prepare_tx_dma(p);
	serial8250_tx_dma_sync_buffer(p, sgl, ret);

	desc = dmaengine_prep_slave_sg(dma->txchan, sgl, ret,
				       DMA_MEM_TO_DEV,
				       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dma_sync_single_for_cpu(dmaengine_get_dma_device(dma->txchan), dma->tx_addr,
					UART_XMIT_SIZE, DMA_TO_DEVICE);
		ret = -EBUSY;
		goto err;
	}

	dma->tx_running = 1;
	dma->tx_flushed = false;
	dma->tx_complete = serial8250_tx_dma_complete;
	desc->callback = __dma_tx_complete;
	desc->callback_param = p;

	dma->tx_cookie = dmaengine_submit(desc);
	dma_async_issue_pending(dma->txchan);
	serial8250_clear_THRI(p);
	dma->tx_err = 0;

	return 0;
err:
	dma->tx_err = 1;
	return ret;
}

void serial8250_tx_dma_flush(struct uart_8250_port *p)
{
	struct uart_8250_dma *dma = p->dma;
	int ret;

	if (!dma->tx_running || serial8250_tx_dma_blocked(dma))
		return;

	/*
	 * kfifo_reset() has been called by the serial core, avoid
	 * advancing the reset FIFO when the old transfer completes.
	 */
	dma->tx_flushed = true;
	/* The core reset the FIFO indices, but DMA may still read its storage. */
	uart_xmit_block(&p->port);

	/*
	 * We can't use `dmaengine_terminate_sync` because `uart_flush_buffer` is
	 * holding the uart port spinlock.
	 */
	dma->tx_terminating = true;
	ret = dmaengine_terminate_async(dma->txchan);
	if (ret) {
		/* Keep the buffer owned by the transfer until its callback runs. */
		dma->tx_terminating = false;
		return;
	}
	schedule_work(&dma->tx_flush_work);
}

int serial8250_rx_dma(struct uart_8250_port *p)
{
	struct uart_8250_dma		*dma = p->dma;
	struct dma_async_tx_descriptor	*desc;

	if (dma->shutting_down || dma->rx_running)
		return 0;
	/* A retiring transfer has released the FIFO, but not its buffer. */
	if (dma->rx_err || dma->rx_terminating)
		return -EIO;

	serial8250_do_prepare_rx_dma(p);

	desc = dmaengine_prep_slave_single(dma->rxchan, dma->rx_addr,
					   dma->rx_size, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EBUSY;

	dma->rx_running = 1;
	desc->callback = dma_rx_complete;
	desc->callback_param = p;

	dma->rx_cookie = dmaengine_submit(desc);

	dma_async_issue_pending(dma->rxchan);

	return 0;
}

int serial8250_rx_dma_flush(struct uart_8250_port *p)
{
	struct uart_8250_dma *dma = p->dma;
	struct dma_tx_state state;
	enum dma_status status;
	int ret;

	lockdep_assert_held_once(&p->port.lock);

	if (!dma || !dma->rx_running)
		return 0;
	/* A late interrupt must not hand a still-active receiver to PIO. */
	if (dma->shutting_down)
		return -ESHUTDOWN;
	if (dma->rx_terminating)
		return -EIO;

	ret = dmaengine_pause(dma->rxchan);
	status = dmaengine_tx_status(dma->rxchan, dma->rx_cookie, &state);
	/* A completed transfer may reject pause before its callback gets the lock. */
	if (ret && status != DMA_COMPLETE)
		goto recover;
	if (status == DMA_ERROR) {
		ret = -EIO;
		goto recover;
	}
	/* Some providers still report IN_PROGRESS after a successful pause. */
	serial8250_rx_dma_copy(p, &state);
	dma->rx_terminating = true;
	ret = dmaengine_terminate_async(dma->rxchan);
	if (ret)
		goto recover;

	/* DMA released the FIFO; retain the buffer until callbacks retire. */
	dma->rx_running = 0;
	schedule_work(&dma->rx_flush_work);
	return 0;

recover:
	serial8250_rx_dma_recover(p);
	return ret;
}
EXPORT_SYMBOL_GPL(serial8250_rx_dma_flush);

int serial8250_request_dma(struct uart_8250_port *p)
{
	struct uart_8250_dma	*dma = p->dma;
	struct device *rx_dev;
	struct device *tx_dev;
	phys_addr_t rx_dma_addr = dma->rx_dma_addr ?
				  dma->rx_dma_addr : p->port.mapbase;
	phys_addr_t tx_dma_addr = dma->tx_dma_addr ?
				  dma->tx_dma_addr : p->port.mapbase;
	dma_cap_mask_t		mask;
	struct dma_slave_caps	caps;
	int			ret;

	/* Default slave configuration parameters */
	dma->rxconf.direction		= DMA_DEV_TO_MEM;
	dma->rxconf.src_addr_width	= DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma->rxconf.src_addr		= rx_dma_addr + UART_RX;

	dma->txconf.direction		= DMA_MEM_TO_DEV;
	dma->txconf.dst_addr_width	= DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma->txconf.dst_addr		= tx_dma_addr + UART_TX;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* Get a channel for RX */
	dma->rxchan = dma_request_slave_channel_compat(mask,
						       dma->fn, dma->rx_param,
						       p->port.dev, "rx");
	if (!dma->rxchan)
		return -ENODEV;

	/* 8250 rx dma requires dmaengine driver to support pause/terminate */
	ret = dma_get_slave_caps(dma->rxchan, &caps);
	if (ret)
		goto release_rx;
	if (!caps.cmd_pause || !caps.cmd_terminate ||
	    caps.residue_granularity == DMA_RESIDUE_GRANULARITY_DESCRIPTOR) {
		ret = -EINVAL;
		goto release_rx;
	}

	ret = dmaengine_slave_config(dma->rxchan, &dma->rxconf);
	if (ret)
		goto release_rx;

	/* Get a channel for TX */
	dma->txchan = dma_request_slave_channel_compat(mask,
						       dma->fn, dma->tx_param,
						       p->port.dev, "tx");
	if (!dma->txchan) {
		ret = -ENODEV;
		goto release_rx;
	}

	/* 8250 tx dma requires dmaengine driver to support terminate */
	ret = dma_get_slave_caps(dma->txchan, &caps);
	if (ret)
		goto err;
	if (!caps.cmd_terminate) {
		ret = -EINVAL;
		goto err;
	}

	ret = dmaengine_slave_config(dma->txchan, &dma->txconf);
	if (ret)
		goto err;

	rx_dev = dmaengine_get_dma_device(dma->rxchan);
	tx_dev = dmaengine_get_dma_device(dma->txchan);

	/* RX buffer */
	if (!dma->rx_size)
		dma->rx_size = PAGE_SIZE;

	dma->rx_buf = dma_alloc_coherent(rx_dev, dma->rx_size, &dma->rx_addr,
					 GFP_KERNEL);
	if (!dma->rx_buf) {
		ret = -ENOMEM;
		goto err;
	}

	/* Normal TX uses serial core's FIFO; flush reserves it until retirement. */
	dma->tx_addr = dma_map_single(tx_dev, p->port.state->port.xmit_buf,
				      UART_XMIT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(tx_dev, dma->tx_addr)) {
		ret = -ENOMEM;
		goto free_rx;
	}
	dma_sync_single_for_cpu(tx_dev, dma->tx_addr, UART_XMIT_SIZE, DMA_TO_DEVICE);

	dma->port = p;
	INIT_WORK(&dma->tx_flush_work, serial8250_tx_dma_flush_work);
	INIT_WORK(&dma->rx_flush_work, serial8250_rx_dma_flush_work);

	scoped_guard(uart_port_lock_irqsave, &p->port) {
		dma->tx_terminating = false;
		dma->tx_flushed = false;
		dma->rx_terminating = false;
		dma->rx_err = false;
		dma->shutting_down = false;
	}
	dev_dbg_ratelimited(p->port.dev, "got both dma channels\n");

	return 0;
free_rx:
	dma_free_coherent(rx_dev, dma->rx_size, dma->rx_buf, dma->rx_addr);
err:
	dma_release_channel(dma->txchan);
release_rx:
	dma_release_channel(dma->rxchan);
	return ret;
}
EXPORT_SYMBOL_GPL(serial8250_request_dma);

void serial8250_release_dma(struct uart_8250_port *p)
{
	struct uart_8250_dma *dma = p->dma;

	if (!dma)
		return;

	cancel_work_sync(&dma->tx_flush_work);
	cancel_work_sync(&dma->rx_flush_work);

	/* Release RX resources */
	dmaengine_terminate_sync(dma->rxchan);
	scoped_guard(uart_port_lock_irqsave, &p->port) {
		dma->rx_running = 0;
		dma->rx_terminating = false;
	}
	dma_free_coherent(dmaengine_get_dma_device(dma->rxchan), dma->rx_size,
			  dma->rx_buf, dma->rx_addr);
	dma_release_channel(dma->rxchan);
	dma->rxchan = NULL;

	/* Release TX resources */
	dmaengine_terminate_sync(dma->txchan);
	dma_unmap_single(dmaengine_get_dma_device(dma->txchan), dma->tx_addr,
			 UART_XMIT_SIZE, DMA_TO_DEVICE);
	dma_release_channel(dma->txchan);
	dma->txchan = NULL;
	scoped_guard(uart_port_lock_irqsave, &p->port) {
		dma->tx_running = 0;
		dma->tx_terminating = false;
		uart_xmit_unblock(&p->port);
	}

	dev_dbg_ratelimited(p->port.dev, "dma channels released\n");
}
EXPORT_SYMBOL_GPL(serial8250_release_dma);
