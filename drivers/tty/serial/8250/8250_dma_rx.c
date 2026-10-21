// SPDX-License-Identifier: GPL-2.0+
/* Continuous receive DMA, with provider-owned one-shot notifications. */
#include <linux/dma-mapping.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>

#include "8250.h"
#include "8250_rx_poll.h"

/* Bound each softirq turn independently of the hardware period size. */
#define RX_RING_BUDGET	4096
/* Target at most one eighth of a CPU in each port's drain callback. */
#define RX_RING_SERVICE_RATIO	8

struct serial8250_rx_ring {
	struct serial8250_rx_poll poll;
	struct dma_async_tx_descriptor *desc;
	u8 *staging;
	u32 consumed;
	u64 sampled_at;
	unsigned int frame_time;
	unsigned int ignore_status_mask;
	u16 errors;
	bool enabled;
	bool stopped;
};

static void serial8250_rx_ring_mask(struct uart_8250_port *up)
{
	struct uart_8250_dma *dma = up->dma;

	/* A persistent line error must not bypass receive coalescing. */
	up->ier &= ~UART_IER_RLSI;
	serial_out(up, UART_IER, up->ier);
	dmaengine_cyclic_notify(dma->rxchan, dma->rx_cookie, false,
			       dma->rx_ring->consumed);
}

static void serial8250_rx_ring_abort(struct uart_8250_port *up, int error)
{
	/* Keep PIO out of the FIFO until termination and callback retirement. */
	up->dma->rx_ring->enabled = false;
	serial8250_rx_dma_flush(up);
	dev_warn_ratelimited(up->port.dev, "continuous RX stopped (%d), using PIO\n", error);
}

static int serial8250_rx_ring_arm(struct uart_8250_port *up)
{
	struct uart_8250_dma *dma = up->dma;
	int ret;

	/* Re-enable errors too, including errors arriving without a DMA byte. */
	up->ier |= UART_IER_RLSI;
	serial_out(up, UART_IER, up->ier);
	ret = dmaengine_cyclic_notify(dma->rxchan, dma->rx_cookie, true,
				     dma->rx_ring->consumed);
	if (ret > 0)
		serial8250_rx_ring_mask(up);
	else if (ret < 0)
		serial8250_rx_ring_abort(up, ret);
	return ret;
}

/* There is no per-byte status in the DMA buffer. Do not publish suspect data. */
static u16 serial8250_rx_ring_errors(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;
	u16 status = serial_lsr_in(up);

	up->lsr_saved_flags = 0;
	if (status & UART_LSR_BI)
		port->icount.brk++;
	else if (status & UART_LSR_PE)
		port->icount.parity++;
	else if (status & UART_LSR_FE)
		port->icount.frame++;
	if (status & UART_LSR_OE)
		port->icount.overrun++;
	up->dma->rx_ring->errors |= status & UART_LSR_BRK_ERROR_BITS;
	return up->dma->rx_ring->errors;
}

static int serial8250_rx_ring_drain(struct uart_8250_port *up, unsigned int budget,
					    unsigned int *pending, bool retired)
{
	struct uart_8250_dma *dma = up->dma;
	struct serial8250_rx_ring *ring = dma->rx_ring;
	struct tty_port *tport = &up->port.state->port;
	struct dma_cyclic_state before, after;
	u32 count, available, first;
	u64 now = ktime_get_ns();
	u16 status;
	bool bad;
	int ret;

	ret = dmaengine_cyclic_progress(dma->rxchan, dma->rx_cookie, &before);
	if (ret)
		return ret;
	available = before.completed - ring->consumed;
	status = serial8250_rx_ring_errors(up);
	/* Status may precede the affected DMA byte. Retire that transfer first. */
	if (status && !retired)
		return -EIO;
	bad = status;
	/* A counter wrap must not turn an unobserved overwrite into an empty ring. */
	if (now - ring->sampled_at >= (u64)U32_MAX * (ring->frame_time - 1))
		bad = true;
	ring->sampled_at = now;
	count = min(available, budget);
	if (bad || before.in_flight_bytes >= dma->rx_size ||
	    available >= dma->rx_size - before.in_flight_bytes)
		goto overrun;
	if (!count) {
		*pending = 0;
		return 0;
	}

	dma_rmb();
	first = min_t(u32, count, dma->rx_size - dma->rx_pos);
	memcpy(ring->staging, dma->rx_buf + dma->rx_pos, first);
	if (count > first)
		memcpy(ring->staging + first, dma->rx_buf, count - first);

	/* Copy into private storage before checking whether DMA lapped the head. */
	rmb();
	ret = dmaengine_cyclic_progress(dma->rxchan, dma->rx_cookie, &after);
	if (ret)
		return ret;
	available = after.completed - ring->consumed;
	status = serial8250_rx_ring_errors(up);
	if (status && !retired)
		return -EIO;
	if (ktime_get_ns() - now >= (u64)U32_MAX * (ring->frame_time - 1) ||
	    status || after.in_flight_bytes >= dma->rx_size ||
	    available >= dma->rx_size - after.in_flight_bytes)
		goto overrun;
	up->port.icount.rx += count;
	if (!(ring->ignore_status_mask & UART_LSR_DR) &&
	    tty_insert_flip_string(tport, ring->staging, count) != count)
		up->port.icount.buf_overrun++;
	goto advance;

overrun:
	count = available;
	up->port.icount.buf_overrun++;
	if (status & UART_LSR_BI) {
		/*
		 * Enabled policies deliver BREAK as NUL. TTY_OVERRUN alone loses
		 * that byte in processed N_TTY modes (for example, with IXON).
		 * Do not append an overrun marker: real_raw would copy it as a
		 * second NUL. The discarded range is counted in buf_overrun.
		 */
		up->port.icount.rx++;
		if (!uart_handle_break(&up->port) &&
		    !(ring->ignore_status_mask & UART_LSR_DR))
			tty_insert_flip_char(tport, 0, TTY_NORMAL);
	} else if (!(ring->ignore_status_mask & UART_LSR_OE)) {
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);
	}
advance:
	ring->consumed += count;
	dma->rx_pos = (dma->rx_pos + count) & (dma->rx_size - 1);
	tty_flip_buffer_push(tport);
	ret = dmaengine_cyclic_progress(dma->rxchan, dma->rx_cookie, &after);
	if (ret)
		return ret;
	*pending = after.completed - ring->consumed;
	return min(count, budget);
}

static int serial8250_rx_ring_poll(struct uart_8250_port *up, unsigned int budget,
				   unsigned int *pending)
{
	int ret = serial8250_rx_ring_drain(up, budget, pending, false);

	if (ret < 0)
		serial8250_rx_ring_abort(up, ret);
	return ret;
}

static const struct serial8250_rx_poll_ops serial8250_rx_ring_ops = {
	.mask = serial8250_rx_ring_mask,
	.poll = serial8250_rx_ring_poll,
	.arm = serial8250_rx_ring_arm,
};

static void serial8250_rx_ring_notify(void *param)
{
	struct uart_8250_port *up = param;

	guard(uart_port_lock_irqsave)(&up->port);
	serial8250_rx_poll_notify(up->dma->rx_poll);
}

int serial8250_rx_ring_request(struct uart_8250_port *up)
{
	struct uart_8250_dma *dma = up->dma;
	struct serial8250_rx_ring *ring;
	int ret;

	if (!dma->cyclic_rx)
		return 0;
	if (!is_power_of_2(dma->rx_size) || dma->rx_size <= up->port.fifosize ||
	    dma->rx_size > U32_MAX)
		return -EINVAL;
	if (!dma->rxchan->device->device_cyclic_progress ||
	    !dma->rxchan->device->device_cyclic_notify)
		return -EOPNOTSUPP;
	ring = kzalloc_obj(*ring);
	if (!ring)
		return -ENOMEM;
	ring->staging = kmalloc(dma->rx_size, GFP_KERNEL);
	if (!ring->staging) {
		kfree(ring);
		return -ENOMEM;
	}
	/* The front end supplies the latency ceiling, not a fixed polling delay. */
	ret = serial8250_rx_poll_init(&ring->poll, up, &serial8250_rx_ring_ops,
				      RX_RING_BUDGET, dma->rx_max_delay_ns,
				      up->port.fifosize, RX_RING_SERVICE_RATIO);
	if (ret) {
		kfree(ring->staging);
		kfree(ring);
		return ret;
	}
	/* Prepare once, outside the port lock, and reuse after synchronization. */
	ring->desc = dmaengine_prep_dma_cyclic(dma->rxchan, dma->rx_addr, dma->rx_size,
					     1, DMA_DEV_TO_MEM, DMA_CTRL_ACK |
					     DMA_PREP_INTERRUPT | DMA_PREP_INTERRUPT_ONCE);
	if (!ring->desc) {
		ret = -ENOMEM;
		goto free_ring;
	}
	ret = dmaengine_desc_set_reuse(ring->desc);
	if (ret)
		goto free_ring;
	ring->desc->callback = serial8250_rx_ring_notify;
	ring->desc->callback_param = up;
	dma->rx_ring = ring;
	dma->rx_poll = &ring->poll;
	ring->stopped = true;
	return 0;
free_ring:
	/* The caller releases the channel, including any unsubmitted descriptor. */
	kfree(ring->staging);
	kfree(ring);
	return ret;
}

/* Caller has stopped/synchronized both the poller and DMA callbacks. */
void serial8250_rx_ring_release(struct uart_8250_port *up)
{
	struct uart_8250_dma *dma = up->dma;
	struct serial8250_rx_ring *ring = dma->rx_ring;

	if (!ring)
		return;
	dma->rx_poll = NULL;
	dma->rx_ring = NULL;
	dmaengine_desc_free(ring->desc);
	kfree(ring->staging);
	kfree(ring);
}

int serial8250_rx_ring_start(struct uart_8250_port *up)
{
	struct uart_8250_dma *dma = up->dma;
	struct serial8250_rx_ring *ring = dma->rx_ring;
	struct dma_cyclic_state state;
	int ret;

	lockdep_assert_held_once(&up->port.lock);
	if (dma->shutting_down || dma->rx_running || dma->rx_terminating)
		return 0;
	if (ring->stopped)
		return ring->enabled ? 0 : -EOPNOTSUPP;
	if (!ring->enabled) {
		ret = -EOPNOTSUPP;
		goto pio;
	}
	dma->rx_cookie = dmaengine_submit(ring->desc);
	ret = dma_submit_error(dma->rx_cookie);
	if (ret)
		goto fallback;
	dma->rx_pos = 0;
	ring->consumed = 0;
	ring->sampled_at = ktime_get_ns();
	ring->frame_time = up->port.frame_time;
	ring->ignore_status_mask = up->port.ignore_status_mask;
	ring->errors = 0;
	dma->rx_running = true;
	ret = dmaengine_cyclic_progress(dma->rxchan, dma->rx_cookie, &state);
	if (ret)
		goto stop;
	if (state.in_flight_bytes >= dma->rx_size - up->port.fifosize) {
		ret = -EINVAL;
		goto stop;
	}
	ring->poll.reserve_bytes = up->port.fifosize + state.in_flight_bytes;
	/*
	 * An idle port can remain open longer than the counter's wrap time.
	 * Sample at half that time so the next real reply is not discarded as
	 * ambiguous. At 12 Mbaud this is about 30 minutes, not RX moderation.
	 */
	ring->poll.max_idle_ns = (u64)U32_MAX * (up->port.frame_time - 1) / 2;
	ret = serial8250_rx_poll_start(&ring->poll);
	if (ret)
		goto stop;
	/* DMA notifications replace RDI; keep line-status errors enabled. */
	up->ier &= ~UART_IER_RDI;
	up->ier |= UART_IER_RLSI;
	serial_out(up, UART_IER, up->ier);
	dma_async_issue_pending(dma->rxchan);
	return 0;
stop:
	ring->enabled = false;
	serial8250_rx_dma_flush(up);
	return ret;
fallback:
	/* Do not retry a failed submission from every receive interrupt. */
	ring->enabled = false;
	dev_warn_ratelimited(up->port.dev, "continuous RX unavailable (%d), using PIO\n", ret);
pio:
	up->ier |= UART_IER_RDI | UART_IER_RLSI;
	serial_out(up, UART_IER, up->ier);
	return ret;
}

/* Called with the port lock, before changing baud, format or error policy. */
void serial8250_rx_ring_termios(struct uart_8250_port *up, struct ktermios *termios,
				unsigned int rs485_flags)
{
	struct uart_8250_dma *dma = up->dma;
	struct serial8250_rx_ring *ring;

	if (!dma || !dma->rx_ring)
		return;
	ring = dma->rx_ring;
	serial8250_rx_dma_flush(up);
	ring->stopped = false;
	/* Use PIO when per-character error handling or auto-RTS is requested. */
	ring->enabled = termios && (termios->c_cflag & CREAD) &&
		!(termios->c_cflag & (PARENB | CRTSCTS)) &&
		!(termios->c_iflag & (INPCK | IGNPAR | IGNBRK | BRKINT | PARMRK)) &&
		!(up->port.flags & UPF_SAK) &&
		!(rs485_flags & SER_RS485_ENABLED);
	/* The retiring transfer still owns the FIFO until its callback is drained. */
	up->ier &= ~(UART_IER_RDI | UART_IER_RLSI);
	if (!dma->rx_terminating)
		up->ier |= UART_IER_RDI | UART_IER_RLSI;
}

void serial8250_rx_ring_stop(struct uart_8250_port *up)
{
	if (!up->dma || !up->dma->rx_ring)
		return;
	up->dma->rx_ring->enabled = false;
	up->dma->rx_ring->stopped = true;
	serial8250_rx_dma_flush(up);
}

/* Returns true while DMA, including a retiring transfer, owns the RX FIFO. */
bool serial8250_rx_ring_irq(struct uart_8250_port *up, u16 status)
{
	struct uart_8250_dma *dma = up->dma;

	if (!dma || !dma->rx_ring || !dma->rx_running)
		return false;
	if (status & (UART_LSR_DR | UART_LSR_BRK_ERROR_BITS))
		serial8250_rx_poll_notify(dma->rx_poll);
	return true;
}

/* Called before terminate_async(); never read a merely paused DMA buffer. */
void serial8250_rx_ring_flush(struct uart_8250_port *up)
{
	/* Do not leave a level interrupt asserted while neither DMA nor PIO drains. */
	up->ier &= ~(UART_IER_RDI | UART_IER_RLSI);
	serial_out(up, UART_IER, up->ier);
	serial8250_rx_poll_stop(up->dma->rx_poll);
}

/* DMA and poll callbacks have retired; the reusable cookie retains its tail. */
void serial8250_rx_ring_complete(struct uart_8250_port *up)
{
	struct uart_8250_dma *dma = up->dma;
	struct serial8250_rx_ring *ring = dma->rx_ring;
	unsigned int pending;
	int ret;

	ret = serial8250_rx_ring_drain(up, dma->rx_size, &pending, true);
	if (ret < 0) {
		up->port.icount.buf_overrun++;
		ring->enabled = false;
		dev_warn_ratelimited(up->port.dev, "cannot recover RX DMA tail (%d)\n", ret);
	}
	if (ring->errors) {
		/*
		 * The affected byte may still be in the FIFO instead of the ring.
		 * Clear the uncertain tail only after DMA has stopped, then use PIO
		 * so a saved error cannot turn into a later, apparently clean byte.
		 */
		serial_out(up, UART_FCR, up->fcr | UART_FCR_CLEAR_RCVR);
		ring->enabled = false;
		ring->errors = 0;
	}
}
