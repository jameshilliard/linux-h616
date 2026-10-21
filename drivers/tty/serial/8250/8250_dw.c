// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare 8250 driver.
 *
 * Copyright 2011 Picochip, Jamie Iles.
 * Copyright 2013 Intel Corporation
 *
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy.  If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/sizes.h>

#include <asm/byteorder.h>

#include <linux/serial_8250.h>
#include <linux/serial_reg.h>

#include "8250_dwlib.h"

#define OCTEON_UART_USR	0x27 /* UART Status Register */

#define RZN1_UART_TDMACR 0x10c /* DMA Control Register Transmit Mode */
#define RZN1_UART_RDMACR 0x110 /* DMA Control Register Receive Mode */

/* Renesas specific register fields */
#define RZN1_UART_xDMACR_DMA_EN		BIT(0)
#define RZN1_UART_xDMACR_1_WORD_BURST	(0 << 1)
#define RZN1_UART_xDMACR_4_WORD_BURST	(1 << 1)
#define RZN1_UART_xDMACR_8_WORD_BURST	(2 << 1)
#define RZN1_UART_xDMACR_BLK_SZ(x)	((x) << 3)

/* Quirks */
#define DW_UART_QUIRK_OCTEON		BIT(0)
#define DW_UART_QUIRK_ARMADA_38X	BIT(1)
#define DW_UART_QUIRK_SKIP_SET_RATE	BIT(2)
#define DW_UART_QUIRK_IS_DMA_FC		BIT(3)
#define DW_UART_QUIRK_APMC0D08		BIT(4)
#define DW_UART_QUIRK_CPR_VALUE		BIT(5)
#define DW_UART_QUIRK_IER_KICK		BIT(6)
#define DW_UART_QUIRK_SKIP_EMPTY_FIFO_READ	BIT(7)
#define DW_UART_QUIRK_NO_DLF		BIT(8)

/*
 * Opt-in coordination for integrations with a shared integer-divisor baudclk.
 * All callbacks run with the port lock held. quiesce() must prevent register
 * bank changes from racing CPU or DMA accesses until unquiesce().
 */
struct dw8250_clk_ops {
	int (*quiesce)(struct uart_port *p);
	unsigned int (*get_divisor)(struct uart_port *p);
	int (*set_divisor)(struct uart_port *p, unsigned int divisor);
	int (*unquiesce)(struct uart_port *p);
};

/*
 * Number of consecutive IIR_NO_INT interrupts required to trigger interrupt
 * storm prevention code.
 */
#define DW_UART_QUIRK_IER_KICK_THRES	4

struct dw8250_platform_data {
	u8 usr_reg;
	u32 cpr_value;
	unsigned int type;
	unsigned int rx_maxburst;
	unsigned int tx_maxburst;
	bool cyclic_rx;
	unsigned int quirks;
	const struct dw8250_clk_ops *clk_ops;
	unsigned long max_uartclk;
};

struct dw8250_data {
	struct dw8250_port_data	data;
	const struct dw8250_platform_data *pdata;

	u32			msr_mask_on;
	u32			msr_mask_off;
	struct clk		*clk;
	struct clk		*pclk;
	struct reset_control	*rst;

	unsigned int		skip_autocfg:1;
	unsigned int		uart_16550_compatible:1;
	unsigned int		in_idle:1;

	u8			no_int_count;

	struct notifier_block	clk_notifier;
	/* Shared-clock state, protected by the port lock and CCF rate exclusion. */
	unsigned int		baud;
	unsigned int		rate_request;
	unsigned int		old_divisor;
	bool			quiesced;
	bool			in_termios;
	bool			clk_prepared;
	bool			clk_pm_ref;
	bool			clk_failed;
	int			config_error;
};

static inline struct dw8250_data *to_dw8250_data(struct dw8250_port_data *data)
{
	return container_of(data, struct dw8250_data, data);
}

static unsigned int dw8250_clk_divisor(unsigned long rate, unsigned int baud)
{
	return clamp_t(unsigned long, DIV_ROUND_CLOSEST_ULL(rate, 16ULL * baud),
		       1, UART_DIV_MAX);
}

static u64 dw8250_clk_error(unsigned long rate, unsigned int baud,
			    unsigned int divisor)
{
	u64 target = 16ULL * baud * divisor;

	return rate > target ? rate - target : target - rate;
}

static bool dw8250_clk_baud_ok(unsigned long rate, unsigned int baud)
{
	unsigned int divisor = dw8250_clk_divisor(rate, baud);

	/* Do not silently accept a grossly rounded BOTHER request after a veto. */
	return dw8250_clk_error(rate, baud, divisor) * 100 <=
	       16ULL * baud * divisor;
}

/* The port lock excludes I/O until the asynchronous hangup shuts it down. */
static void dw8250_clk_fail(struct uart_port *p, int error)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);

	d->config_error = error;
	d->clk_failed = true;
	serial_port_out(p, UART_IER, 0);
	if (p->state)
		tty_port_tty_hangup(&p->state->port, false);
}


static int dw8250_clk_notify(struct notifier_block *nb, unsigned long event,
				  void *arg)
{
	struct dw8250_data *d = container_of(nb, struct dw8250_data, clk_notifier);
	struct clk_notifier_data *change = arg;
	const struct dw8250_clk_ops *ops = d->pdata->clk_ops;
	struct uart_port *p = &d->data.uport->port;
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int baud, divisor, old_divisor;
	unsigned long flags;
	int active, ret = 0;

	if (event == PRE_RATE_CHANGE) {
		if (!change->new_rate || change->new_rate > d->pdata->max_uartclk)
			return NOTIFY_BAD;
		active = pm_runtime_get_if_active(p->dev);
		uart_port_lock_irqsave(p, &flags);
		baud = d->rate_request ?: d->baud;
		if (!baud || (p->suspended && !d->in_termios) ||
		    (!active && IS_ENABLED(CONFIG_PM)))
			goto pre_done;
		if (d->clk_failed) {
			ret = -EIO;
			goto pre_done;
		}
		if (active < 0 && IS_ENABLED(CONFIG_PM)) {
			ret = active;
			goto pre_done;
		}
		divisor = dw8250_clk_divisor(change->new_rate, baud);
		old_divisor = dw8250_clk_divisor(change->old_rate, baud);
		if (!d->rate_request &&
		    dw8250_clk_error(change->new_rate, baud, divisor) * old_divisor >
		    dw8250_clk_error(change->old_rate, baud, old_divisor) * divisor) {
			ret = -ERANGE;
			goto pre_done;
		}
		ret = ops->quiesce(p);
		if (ret) {
			if (up->dma && up->dma->quiesced)
				dw8250_clk_fail(p, ret);
			goto pre_done;
		}
		d->old_divisor = ops->get_divisor(p);
		ret = ops->set_divisor(p, divisor);
		if (ret) {
			d->config_error = ops->set_divisor(p, d->old_divisor);
			if (!d->config_error)
				d->config_error = ops->unquiesce(p);
			if (d->config_error)
				dw8250_clk_fail(p, d->config_error);
			goto pre_done;
		}
		d->clk_prepared = true;
		d->clk_pm_ref = active > 0;
		active = 0;
pre_done:
		uart_port_unlock_irqrestore(p, flags);
		if (active > 0)
			pm_runtime_put(p->dev);
		return ret ? NOTIFY_BAD : NOTIFY_OK;
	}
	if (event != POST_RATE_CHANGE && event != ABORT_RATE_CHANGE)
		return NOTIFY_DONE;

	uart_port_lock_irqsave(p, &flags);
	if (event == POST_RATE_CHANGE)
		p->uartclk = change->new_rate;
	if (d->clk_prepared) {
		if (event == ABORT_RATE_CHANGE)
			ret = ops->set_divisor(p, d->old_divisor);
		else if (d->rate_request)
			d->baud = d->rate_request;
		if (!ret && (event == ABORT_RATE_CHANGE || !d->in_termios))
			ret = ops->unquiesce(p);
		if (ret)
			dw8250_clk_fail(p, ret);
		d->clk_prepared = false;
	}
	active = d->clk_pm_ref;
	d->clk_pm_ref = false;
	uart_port_unlock_irqrestore(p, flags);
	if (active)
		pm_runtime_put(p->dev);
	if (ret)
		dev_err_ratelimited(p->dev, "clock handoff failed: %d\n", ret);
	return NOTIFY_OK;
}

static void dw8250_shared_clk_set_termios(struct uart_port *p, struct ktermios *termios,
				  const struct ktermios *old)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	const struct dw8250_clk_ops *ops = d->pdata->clk_ops;
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int baud = tty_termios_baud_rate(termios);
	unsigned int divisor;
	unsigned long flags;
	u64 step = 16ULL * baud;
	u64 candidate;
	long rounded, previous = 0;
	int ret;

	/* Notifiers never take the TTY locks held by this caller. */
	ret = clk_rate_exclusive_get(d->clk);
	if (ret)
		goto reject;
	if (up->dma && up->dma->rxchan) {
		flush_work(&up->dma->rx_flush_work);
		flush_work(&up->dma->tx_flush_work);
	}
	uart_port_lock_irqsave(p, &flags);
	if (d->clk_failed) {
		ret = -EIO;
		uart_port_unlock_irqrestore(p, flags);
		goto put_rate;
	}
	d->config_error = 0;
	d->in_termios = true;
	d->rate_request = baud;
	uart_port_unlock_irqrestore(p, flags);

	/*
	 * Search while the port is running. PRE holds it only for each actual
	 * transition, and ABORT releases it before trying another candidate.
	 * Old single-clock DTs retain fixed-rate operation.
	 */
	if (baud && d->pclk && !dw8250_clk_baud_ok(p->uartclk, baud)) {
		for (divisor = 1; divisor <= UART_DIV_MAX; divisor++) {
			candidate = step * divisor;
			if (candidate > d->pdata->max_uartclk)
				break;
			rounded = clk_round_rate(d->clk, candidate);
			/* Several divisors can round to the same rejected clock. */
			if (rounded == previous)
				continue;
			previous = rounded;
			if (rounded <= 0 || rounded > d->pdata->max_uartclk ||
			    !dw8250_clk_baud_ok(rounded, baud))
				continue;
			if (!clk_set_rate(d->clk, rounded))
				break;
			if (d->config_error)
				break;
		}
	}
	uart_port_lock_irqsave(p, &flags);
	d->rate_request = 0;
	ret = d->config_error;
	if (!ret)
		ret = ops->quiesce(p);
	if (ret) {
		d->in_termios = false;
		if (up->dma && up->dma->quiesced)
			dw8250_clk_fail(p, ret);
	}
	uart_port_unlock_irqrestore(p, flags);
	if (ret)
		goto put_rate;
	if (baud && !dw8250_clk_baud_ok(p->uartclk, baud)) {
		baud = d->baud ?: 9600;
		tty_termios_encode_baud_rate(termios, baud, baud);
	}
	dw8250_do_set_termios(p, termios, old);
	uart_port_lock_irqsave(p, &flags);
	d->in_termios = false;
	ret = d->config_error;
	if (!ret)
		ret = ops->unquiesce(p);
	if (ret)
		dw8250_clk_fail(p, ret);
	uart_port_unlock_irqrestore(p, flags);
put_rate:
	clk_rate_exclusive_put(d->clk);
reject:
	if (ret) {
		/* A failed hardware update hangs up, rather than claiming rollback. */
		if (old && !d->clk_failed)
			*termios = *old;
		dev_warn_ratelimited(p->dev, "cannot change UART configuration: %d\n", ret);
	}
}

static int dw8250_shared_clk_startup(struct uart_port *p)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	unsigned long flags;
	int ret;

	ret = clk_rate_exclusive_get(d->clk);
	if (ret)
		return ret;
	if (d->clk_failed) {
		/* Shutdown retired the old DMA channels before this new open. */
		if (!d->rst) {
			ret = -EIO;
			goto put_rate;
		}
		ret = reset_control_assert(d->rst);
		if (ret)
			goto put_rate;
		ret = reset_control_deassert(d->rst);
		if (ret)
			goto put_rate;
		uart_port_lock_irqsave(p, &flags);
		ret = d->pdata->clk_ops->set_divisor(p,
				dw8250_clk_divisor(p->uartclk, d->baud ?: 9600));
		if (!ret) {
			d->quiesced = false;
			d->clk_failed = false;
			d->config_error = 0;
		}
		uart_port_unlock_irqrestore(p, flags);
		if (ret)
			goto put_rate;
	}
	ret = serial8250_do_startup(p);
	if (!ret) {
		uart_port_lock_irqsave(p, &flags);
		ret = d->config_error;
		if (ret)
			dw8250_clk_fail(p, ret);
		uart_port_unlock_irqrestore(p, flags);
		if (ret)
			serial8250_do_shutdown(p);
	}
put_rate:
	clk_rate_exclusive_put(d->clk);
	return ret;
}

static inline u32 dw8250_modify_msr(struct uart_port *p, unsigned int offset, u32 value)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);

	/* Override any modem control signals if needed */
	if (offset == UART_MSR) {
		value |= d->msr_mask_on;
		value &= ~d->msr_mask_off;
	}

	return value;
}

static void dw8250_idle_exit(struct uart_port *p)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	struct uart_8250_port *up = up_to_u8250p(p);

	if (d->uart_16550_compatible)
		return;

	if (up->capabilities & UART_CAP_FIFO)
		serial_port_out(p, UART_FCR, up->fcr);
	serial_port_out(p, UART_MCR, up->mcr);
	serial_port_out(p, UART_IER, up->ier);

	/* DMA Rx is restarted by IRQ handler as needed. */
	if (up->dma)
		serial8250_tx_dma_resume(up);

	d->in_idle = 0;
}

/*
 * Ensure BUSY is not asserted. If DW UART is configured with
 * !uart_16550_compatible, the writes to LCR, DLL, and DLH fail while
 * BUSY is asserted.
 *
 * Context: port's lock must be held
 */
static int dw8250_idle_enter(struct uart_port *p)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	unsigned int usr_reg = d->pdata ? d->pdata->usr_reg : DW_UART_USR;
	struct uart_8250_port *up = up_to_u8250p(p);
	int retries;
	int ret;
	u32 lsr;

	lockdep_assert_held_once(&p->lock);

	if (d->uart_16550_compatible)
		return 0;

	d->in_idle = 1;

	/* Prevent triggering interrupt from RBR filling */
	serial_port_out(p, UART_IER, 0);

	if (up->dma) {
		ret = serial8250_rx_dma_flush(up);
		if (ret) {
			serial_port_out(p, UART_IER, up->ier);
			d->in_idle = 0;
			return ret;
		}
		if (serial8250_tx_dma_running(up))
			serial8250_tx_dma_pause(up);
	}

	/*
	 * Wait until Tx becomes empty + one extra frame time to ensure all bits
	 * have been sent on the wire.
	 *
	 * FIXME: frame_time delay is too long with very low baudrates.
	 */
	serial8250_fifo_wait_for_lsr_thre(up, NULL, p->fifosize);
	ndelay(p->frame_time);

	serial_port_out(p, UART_MCR, up->mcr | UART_MCR_LOOP);

	retries = 4;	/* Arbitrary limit, 2 was always enough in tests */
	do {
		serial8250_clear_fifos(up);
		if (!(serial_port_in(p, usr_reg) & DW_UART_USR_BUSY))
			break;
		/* FIXME: frame_time delay is too long with very low baudrates. */
		ndelay(p->frame_time);
	} while (--retries);

	lsr = serial_lsr_in(up);
	if (lsr & UART_LSR_DR) {
		serial_port_in(p, UART_RX);
		up->lsr_saved_flags = 0;
	}

	/* Now guaranteed to have BUSY deasserted? Just sanity check */
	if (serial_port_in(p, usr_reg) & DW_UART_USR_BUSY) {
		dw8250_idle_exit(p);
		return -EBUSY;
	}

	return 0;
}

static void dw8250_set_divisor(struct uart_port *p, unsigned int baud,
			       unsigned int quot, unsigned int quot_frac)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	int ret;

	ret = dw8250_idle_enter(p);
	if (ret < 0)
		return;

	__dw8250_set_divisor(p, quot_frac);

	serial_port_out(p, UART_LCR, up->lcr | UART_LCR_DLAB);
	if (!(serial_port_in(p, UART_LCR) & UART_LCR_DLAB))
		goto idle_failed;

	serial_dl_write(up, quot);
	serial_port_out(p, UART_LCR, up->lcr);

idle_failed:
	dw8250_idle_exit(p);
}

/*
 * This function is being called as part of the uart_port::serial_out()
 * routine. Hence, special care must be taken when serial_port_out() or
 * serial_out() against the modified registers here, i.e. LCR (d->in_idle is
 * used to break recursion loop).
 */
static void dw8250_check_lcr(struct uart_port *p, unsigned int offset, u32 value)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	u32 lcr;
	int ret;

	if (offset != UART_LCR || d->uart_16550_compatible)
		return;

	lcr = serial_port_in(p, UART_LCR);

	/* Make sure LCR write wasn't ignored */
	if ((value & ~UART_LCR_SPAR) == (lcr & ~UART_LCR_SPAR))
		return;

	if (d->in_idle)
		goto write_err;

	ret = dw8250_idle_enter(p);
	if (ret < 0)
		goto write_err;

	serial_port_out(p, UART_LCR, value);
	dw8250_idle_exit(p);
	return;

write_err:
	/*
	 * FIXME: this deadlocks if port->lock is already held
	 * dev_err(p->dev, "Couldn't set LCR to %d\n", value);
	 */
	return;		/* Silences "label at the end of compound statement" */
}

/*
 * With BUSY, LCR writes can be very expensive (IRQ + complex retry logic).
 * If the write does not change the value of the LCR register, skip it entirely.
 */
static bool dw8250_can_skip_reg_write(struct uart_port *p, unsigned int offset, u32 value)
{
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	u32 lcr;

	if (offset != UART_LCR || d->uart_16550_compatible)
		return false;

	lcr = serial_port_in(p, offset);
	return lcr == value;
}

/* Returns once the transmitter is empty or we run out of retries */
static void dw8250_tx_wait_empty(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int tries = 20000;
	unsigned int delay_threshold = tries - 1000;
	unsigned int lsr;

	while (tries--) {
		lsr = readb (p->membase + (UART_LSR << p->regshift));
		up->lsr_saved_flags |= lsr & up->lsr_save_mask;

		if (lsr & UART_LSR_TEMT)
			break;

		/* The device is first given a chance to empty without delay,
		 * to avoid slowdowns at high bitrates. If after 1000 tries
		 * the buffer has still not emptied, allow more time for low-
		 * speed links. */
		if (tries < delay_threshold)
			udelay (1);
	}
}

static void dw8250_serial_out(struct uart_port *p, unsigned int offset, u32 value)
{
	if (dw8250_can_skip_reg_write(p, offset, value))
		return;

	writeb(value, p->membase + (offset << p->regshift));
	dw8250_check_lcr(p, offset, value);
}

static void dw8250_serial_out38x(struct uart_port *p, unsigned int offset, u32 value)
{
	if (dw8250_can_skip_reg_write(p, offset, value))
		return;

	/* Allow the TX to drain before we reconfigure */
	if (offset == UART_LCR)
		dw8250_tx_wait_empty(p);

	dw8250_serial_out(p, offset, value);
}

static u32 dw8250_serial_in(struct uart_port *p, unsigned int offset)
{
	u32 value = readb(p->membase + (offset << p->regshift));

	return dw8250_modify_msr(p, offset, value);
}

#ifdef CONFIG_64BIT
static u32 dw8250_serial_inq(struct uart_port *p, unsigned int offset)
{
	u8 value = __raw_readq(p->membase + (offset << p->regshift));

	return dw8250_modify_msr(p, offset, value);
}

static void dw8250_serial_outq(struct uart_port *p, unsigned int offset, u32 value)
{
	if (dw8250_can_skip_reg_write(p, offset, value))
		return;

	value &= 0xff;
	__raw_writeq(value, p->membase + (offset << p->regshift));
	/* Read back to ensure register write ordering. */
	__raw_readq(p->membase + (UART_LCR << p->regshift));

	dw8250_check_lcr(p, offset, value);
}
#endif /* CONFIG_64BIT */

static void dw8250_serial_out32(struct uart_port *p, unsigned int offset, u32 value)
{
	if (dw8250_can_skip_reg_write(p, offset, value))
		return;

	writel(value, p->membase + (offset << p->regshift));
	dw8250_check_lcr(p, offset, value);
}

static u32 dw8250_serial_in32(struct uart_port *p, unsigned int offset)
{
	u32 value = readl(p->membase + (offset << p->regshift));

	return dw8250_modify_msr(p, offset, value);
}

static void dw8250_serial_out32be(struct uart_port *p, unsigned int offset, u32 value)
{
	if (dw8250_can_skip_reg_write(p, offset, value))
		return;

	iowrite32be(value, p->membase + (offset << p->regshift));
	dw8250_check_lcr(p, offset, value);
}

static u32 dw8250_serial_in32be(struct uart_port *p, unsigned int offset)
{
       u32 value = ioread32be(p->membase + (offset << p->regshift));

       return dw8250_modify_msr(p, offset, value);
}

/*
 * INTC10EE UART can IRQ storm while reporting IIR_NO_INT. Inducing IIR value
 * change has been observed to break the storm.
 *
 * If Tx is empty (THRE asserted), we use here IER_THRI to cause IIR_NO_INT ->
 * IIR_THRI transition.
 */
static void dw8250_quirk_ier_kick(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	u32 lsr;

	if (up->ier & UART_IER_THRI)
		return;

	lsr = serial_lsr_in(up);
	if (!(lsr & UART_LSR_THRE))
		return;

	serial_port_out(p, UART_IER, up->ier | UART_IER_THRI);
	serial_port_in(p, UART_LCR);		/* safe, no side-effects */
	serial_port_out(p, UART_IER, up->ier);
}

static int dw8250_handle_irq(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	unsigned int iir = serial_port_in(p, UART_IIR);
	bool rx_timeout = (iir & 0x3f) == UART_IIR_RX_TIMEOUT;
	unsigned int quirks = d->pdata->quirks;
	unsigned int status;

	guard(uart_port_lock_check_sysrq_irqsave)(p);

	switch (FIELD_GET(DW_UART_IIR_IID, iir)) {
	case UART_IIR_NO_INT:
		if (d->uart_16550_compatible || up->dma)
			return 0;

		if (quirks & DW_UART_QUIRK_IER_KICK &&
		    d->no_int_count == (DW_UART_QUIRK_IER_KICK_THRES - 1))
			dw8250_quirk_ier_kick(p);
		d->no_int_count = (d->no_int_count + 1) % DW_UART_QUIRK_IER_KICK_THRES;

		return 0;

	case UART_IIR_BUSY:
		/* Clear the USR */
		serial_port_in(p, d->pdata->usr_reg);

		d->no_int_count = 0;

		return 1;
	}

	d->no_int_count = 0;

	/*
	 * There are ways to get Designware-based UARTs into a state where
	 * they are asserting UART_IIR_RX_TIMEOUT but there is no actual
	 * data available.  If we see such a case then we'll do a bogus
	 * read.  If we don't do this then the "RX TIMEOUT" interrupt will
	 * fire forever.
	 *
	 * This problem has only been observed so far when not in DMA mode
	 * so limit the workaround to PIO, including continuous-DMA fallback.
	 */
	if (!(quirks & DW_UART_QUIRK_SKIP_EMPTY_FIFO_READ) && rx_timeout &&
	    (!up->dma || (up->dma->rx_ring && !up->dma->rx_running &&
			 !up->dma->rx_terminating))) {
		status = serial_lsr_in(up);

		/*
		 * Do the bogus read from Shadow RBR (SRBR) if provided.
		 * Both RBR and SRBR are supported ways to workaround
		 * this problem. But read RBR can cause hardware error
		 * (PSLVERR) if hardware choose to implement in this way
		 * (implement REG_TIMEOUT_WIDTH to 0), whereas SRBR won't.
		 */
		if (!(status & (UART_LSR_DR | UART_LSR_BI)))
			serial_port_in(p, d->data.srbr);
	}

	/* Manually stop the Rx DMA transfer when acting as flow controller */
	if (quirks & DW_UART_QUIRK_IS_DMA_FC && up->dma && up->dma->rx_running && rx_timeout) {
		status = serial_lsr_in(up);

		if (status & (UART_LSR_DR | UART_LSR_BI)) {
			dw8250_writel_ext(p, RZN1_UART_RDMACR, 0);
			dw8250_writel_ext(p, DW_UART_DMASA, 1);
		}
	}

	serial8250_handle_irq_locked(p, iir);

	return 1;
}

static void
dw8250_do_pm(struct uart_port *port, unsigned int state, unsigned int old)
{
	if (!state)
		pm_runtime_get_sync(port->dev);

	serial8250_do_pm(port, state, old);

	if (state)
		pm_runtime_put_sync_suspend(port->dev);
}

static void dw8250_set_termios(struct uart_port *p, struct ktermios *termios,
			       const struct ktermios *old)
{
	unsigned long newrate = tty_termios_baud_rate(termios) * 16;
	struct dw8250_data *d = to_dw8250_data(p->private_data);
	long rate;
	int ret;

	clk_disable_unprepare(d->clk);
	rate = clk_round_rate(d->clk, newrate);
	if (rate > 0) {
		ret = clk_set_rate(d->clk, newrate);
		if (!ret)
			p->uartclk = rate;
	}
	clk_prepare_enable(d->clk);

	dw8250_do_set_termios(p, termios, old);
}

static void dw8250_set_ldisc(struct uart_port *p, struct ktermios *termios)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int mcr = serial_port_in(p, UART_MCR);

	if (up->capabilities & UART_CAP_IRDA) {
		if (termios->c_line == N_IRDA)
			mcr |= DW_UART_MCR_SIRE;
		else
			mcr &= ~DW_UART_MCR_SIRE;

		serial_port_out(p, UART_MCR, mcr);
	}
	serial8250_do_set_ldisc(p, termios);
}

/*
 * dw8250_fallback_dma_filter will prevent the UART from getting just any free
 * channel on platforms that have DMA engines, but don't have any channels
 * assigned to the UART.
 *
 * REVISIT: This is a work around for limitation in the DMA Engine API. Once the
 * core problem is fixed, this function is no longer needed.
 */
static bool dw8250_fallback_dma_filter(struct dma_chan *chan, void *param)
{
	return false;
}

static bool dw8250_idma_filter(struct dma_chan *chan, void *param)
{
	return param == chan->device->dev;
}

static void dw8250_setup_dma_filter(struct uart_port *p, struct dw8250_data *data)
{
	/* Platforms with iDMA 64-bit */
	if (platform_get_resource_byname(to_platform_device(p->dev), IORESOURCE_MEM, "lpss_priv")) {
		data->data.dma.rx_param = p->dev->parent;
		data->data.dma.tx_param = p->dev->parent;
		data->data.dma.fn = dw8250_idma_filter;
	} else {
		data->data.dma.fn = dw8250_fallback_dma_filter;
	}
}

static u32 dw8250_rzn1_get_dmacr_burst(int max_burst)
{
	if (max_burst >= 8)
		return RZN1_UART_xDMACR_8_WORD_BURST;
	else if (max_burst >= 4)
		return RZN1_UART_xDMACR_4_WORD_BURST;
	else
		return RZN1_UART_xDMACR_1_WORD_BURST;
}

static void dw8250_prepare_tx_dma(struct uart_8250_port *p)
{
	struct uart_port *up = &p->port;
	struct uart_8250_dma *dma = p->dma;
	u32 val;

	dw8250_writel_ext(up, RZN1_UART_TDMACR, 0);
	val = dw8250_rzn1_get_dmacr_burst(dma->txconf.dst_maxburst) |
	      RZN1_UART_xDMACR_BLK_SZ(dma->tx_size) |
	      RZN1_UART_xDMACR_DMA_EN;
	dw8250_writel_ext(up, RZN1_UART_TDMACR, val);
}

static void dw8250_prepare_rx_dma(struct uart_8250_port *p)
{
	struct uart_port *up = &p->port;
	struct uart_8250_dma *dma = p->dma;
	u32 val;

	dw8250_writel_ext(up, RZN1_UART_RDMACR, 0);
	val = dw8250_rzn1_get_dmacr_burst(dma->rxconf.src_maxburst) |
	      RZN1_UART_xDMACR_BLK_SZ(dma->rx_size) |
	      RZN1_UART_xDMACR_DMA_EN;
	dw8250_writel_ext(up, RZN1_UART_RDMACR, val);
}

static void dw8250_quirks(struct uart_port *p, struct dw8250_data *data)
{
	unsigned int quirks = data->pdata->quirks;
	u32 cpr_value = data->pdata->cpr_value;

	if (quirks & DW_UART_QUIRK_CPR_VALUE)
		data->data.cpr_value = cpr_value;

#ifdef CONFIG_64BIT
	if (quirks & DW_UART_QUIRK_OCTEON) {
		p->serial_in = dw8250_serial_inq;
		p->serial_out = dw8250_serial_outq;
		p->flags = UPF_SKIP_TEST | UPF_SHARE_IRQ | UPF_FIXED_TYPE;
		p->type = PORT_OCTEON;
		data->skip_autocfg = true;
	}
#endif

	if (quirks & DW_UART_QUIRK_ARMADA_38X)
		p->serial_out = dw8250_serial_out38x;
	if (quirks & DW_UART_QUIRK_SKIP_SET_RATE)
		p->set_termios = dw8250_do_set_termios;
	if (quirks & DW_UART_QUIRK_NO_DLF)
		data->data.skip_dlf = true;
	if (data->pdata->clk_ops) {
		p->set_termios = dw8250_shared_clk_set_termios;
		p->startup = dw8250_shared_clk_startup;
	}
	if (quirks & DW_UART_QUIRK_IS_DMA_FC) {
		data->data.dma.txconf.device_fc = 1;
		data->data.dma.rxconf.device_fc = 1;
		data->data.dma.prepare_tx_dma = dw8250_prepare_tx_dma;
		data->data.dma.prepare_rx_dma = dw8250_prepare_rx_dma;
	}
	if (quirks & DW_UART_QUIRK_APMC0D08) {
		p->iotype = UPIO_MEM32;
		p->regshift = 2;
		p->serial_in = dw8250_serial_in32;
		data->uart_16550_compatible = true;
	}
}

static void dw8250_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static void dw8250_shutdown(struct uart_port *port)
{
	struct dw8250_data *d = to_dw8250_data(port->private_data);
	bool shared_clk = d->pdata && d->pdata->clk_ops;
	unsigned long flags;

	if (shared_clk)
		clk_rate_exclusive_get(d->clk);
	serial8250_do_shutdown(port);
	d->no_int_count = 0;
	if (shared_clk) {
		uart_port_lock_irqsave(port, &flags);
		if (!uart_console(port))
			d->baud = 0;
		uart_port_unlock_irqrestore(port, flags);
		clk_rate_exclusive_put(d->clk);
	}
}

static int dw8250_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {}, *up = &uart;
	struct uart_port *p = &up->port;
	struct device *dev = &pdev->dev;
	struct dw8250_data *data;
	struct resource *regs;
	int err;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return dev_err_probe(dev, -EINVAL, "no registers defined\n");

	spin_lock_init(&p->lock);
	p->pm		= dw8250_do_pm;
	p->type		= PORT_8250;
	p->flags	= UPF_FIXED_PORT;
	p->dev		= dev;

	p->set_ldisc	= dw8250_set_ldisc;
	p->set_termios	= dw8250_set_termios;
	p->set_divisor	= dw8250_set_divisor;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	p->private_data = &data->data;

	p->mapbase = regs->start;
	p->mapsize = resource_size(regs);

	p->membase = devm_ioremap(dev, p->mapbase, p->mapsize);
	if (!p->membase)
		return -ENOMEM;

	err = uart_read_port_properties(p);
	/* no interrupt -> fall back to polling */
	if (err == -ENXIO)
		err = 0;
	if (err)
		return err;

	switch (p->iotype) {
	case UPIO_MEM:
		p->serial_in = dw8250_serial_in;
		p->serial_out = dw8250_serial_out;
		break;
	case UPIO_MEM32:
		p->serial_in = dw8250_serial_in32;
		p->serial_out = dw8250_serial_out32;
		break;
	case UPIO_MEM32BE:
		p->serial_in = dw8250_serial_in32be;
		p->serial_out = dw8250_serial_out32be;
		break;
	default:
		return -ENODEV;
	}

	if (device_property_read_bool(dev, "dcd-override")) {
		/* Always report DCD as active */
		data->msr_mask_on |= UART_MSR_DCD;
		data->msr_mask_off |= UART_MSR_DDCD;
	}

	if (device_property_read_bool(dev, "dsr-override")) {
		/* Always report DSR as active */
		data->msr_mask_on |= UART_MSR_DSR;
		data->msr_mask_off |= UART_MSR_DDSR;
	}

	if (device_property_read_bool(dev, "cts-override")) {
		/* Always report CTS as active */
		data->msr_mask_on |= UART_MSR_CTS;
		data->msr_mask_off |= UART_MSR_DCTS;
	}

	if (device_property_read_bool(dev, "ri-override")) {
		/* Always report Ring indicator as inactive */
		data->msr_mask_off |= UART_MSR_RI;
		data->msr_mask_off |= UART_MSR_TERI;
	}

	/* If there is separate baudclk, get the rate from it. */
	data->clk = devm_clk_get_optional_enabled(dev, "baudclk");
	if (data->clk == NULL)
		data->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(data->clk))
		return dev_err_probe(dev, PTR_ERR(data->clk),
				     "failed to get baudclk\n");

	if (data->clk)
		p->uartclk = clk_get_rate(data->clk);

	/* If no clock rate is defined, fail. */
	if (!p->uartclk)
		return dev_err_probe(dev, -EINVAL, "clock rate not defined\n");

	data->pclk = devm_clk_get_optional_enabled(dev, "apb_pclk");
	if (IS_ERR(data->pclk))
		return PTR_ERR(data->pclk);

	data->rst = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(data->rst))
		return PTR_ERR(data->rst);

	err = reset_control_deassert(data->rst);
	if (err)
		return dev_err_probe(dev, err, "failed to deassert resets\n");

	err = devm_add_action_or_reset(dev, dw8250_reset_control_assert, data->rst);
	if (err)
		return err;

	err = pm_runtime_set_active(dev);
	if (err)
		return dev_err_probe(dev, err, "Failed to set the runtime suspend as active\n");

	data->uart_16550_compatible = device_property_read_bool(dev, "snps,uart-16550-compatible");

	data->pdata = device_get_match_data(p->dev);
	if (data->pdata)
		dw8250_quirks(p, data);

	/* If the Busy Functionality is not implemented, don't handle it */
	if (data->uart_16550_compatible) {
		p->handle_irq = NULL;
	} else if (data->pdata) {
		p->handle_irq = dw8250_handle_irq;
		p->shutdown = dw8250_shutdown;
	}

	dw8250_setup_dma_filter(p, data);

	if (!data->skip_autocfg)
		dw8250_setup_port(p);

	if (data->pdata && data->pdata->type) {
		p->type = data->pdata->type;
		p->flags |= UPF_FIXED_TYPE;
	}

	/* If we have a valid fifosize, try hooking up DMA */
	if (p->fifosize) {
		data->data.dma.rxconf.src_maxburst = p->fifosize / 4;
		if (data->pdata && data->pdata->rx_maxburst)
			data->data.dma.rxconf.src_maxburst = data->pdata->rx_maxburst;
		data->data.dma.txconf.dst_maxburst = p->fifosize / 4;
		if (data->pdata && data->pdata->tx_maxburst)
			data->data.dma.txconf.dst_maxburst = data->pdata->tx_maxburst;
		if (data->pdata && data->pdata->cyclic_rx) {
			data->data.dma.cyclic_rx = true;
			data->data.dma.rx_size = SZ_16K;
			/* Ceiling on extra batching latency; the poll interval adapts. */
			data->data.dma.rx_max_delay_ns = NSEC_PER_MSEC / 4;
			data->data.dma.rxconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
			data->data.dma.rxconf.dst_maxburst = 1;
		}
		up->dma = &data->data.dma;
	}

	if (data->pdata && data->pdata->clk_ops) {
		if (!data->clk)
			return -EINVAL;
		p->shutdown = dw8250_shutdown;
		err = clk_rate_exclusive_get(data->clk);
		if (err)
			return err;
	}
	data->data.uport = serial8250_register_8250_port(up);
	if (IS_ERR(data->data.uport)) {
		err = PTR_ERR(data->data.uport);
		goto put_rate;
	}
	if (data->pdata && data->pdata->clk_ops) {
		data->clk_notifier.notifier_call = dw8250_clk_notify;
		err = clk_notifier_register(data->clk, &data->clk_notifier);
		if (err) {
			serial8250_unregister_port(data->data.uport);
			goto put_rate;
		}
	}

	platform_set_drvdata(pdev, data);

	pm_runtime_enable(dev);

	err = 0;
put_rate:
	if (data->pdata && data->pdata->clk_ops)
		clk_rate_exclusive_put(data->clk);
	return err;
}

static void dw8250_remove(struct platform_device *pdev)
{
	struct dw8250_data *data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);

	if (data->pdata && data->pdata->clk_ops) {
		clk_rate_exclusive_get(data->clk);
		clk_notifier_unregister(data->clk, &data->clk_notifier);
	}
	serial8250_unregister_port(data->data.uport);
	if (data->pdata && data->pdata->clk_ops)
		clk_rate_exclusive_put(data->clk);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
}

static int dw8250_suspend(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	serial8250_suspend_port(data->data.uport);

	return 0;
}

static int dw8250_resume(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	serial8250_resume_port(data->data.uport);

	return 0;
}

static int dw8250_runtime_suspend(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	clk_disable_unprepare(data->clk);

	clk_disable_unprepare(data->pclk);

	return 0;
}

static int dw8250_runtime_resume(struct device *dev)
{
	int ret;
	struct dw8250_data *data = dev_get_drvdata(dev);

	ret = clk_prepare_enable(data->pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		clk_disable_unprepare(data->pclk);
		return ret;
	}

	return 0;
}

static _DEFINE_DEV_PM_OPS(dw8250_pm_ops, dw8250_suspend, dw8250_resume,
			  dw8250_runtime_suspend, dw8250_runtime_resume,
			  NULL);

static const struct dw8250_platform_data dw8250_dw_apb = {
	.usr_reg = DW_UART_USR,
};

static const struct dw8250_platform_data dw8250_octeon_3860_data = {
	.usr_reg = OCTEON_UART_USR,
	.quirks = DW_UART_QUIRK_OCTEON,
};

static const struct dw8250_platform_data dw8250_armada_38x_data = {
	.usr_reg = DW_UART_USR,
	.quirks = DW_UART_QUIRK_ARMADA_38X,
};

static const struct dw8250_platform_data dw8250_renesas_rzn1_data = {
	.usr_reg = DW_UART_USR,
	.cpr_value = FIELD_PREP_CONST(DW_UART_CPR_ABP_DATA_WIDTH, 2) |
		     DW_UART_CPR_AFCE_MODE |
		     DW_UART_CPR_THRE_MODE |
		     DW_UART_CPR_ADDITIONAL_FEATURES |
		     DW_UART_CPR_FIFO_ACCESS |
		     DW_UART_CPR_FIFO_STAT |
		     DW_UART_CPR_SHADOW |
		     DW_UART_CPR_DMA_EXTRA |
		     DW_UART_CPR_FIFO_MODE_FROM_SIZE(16),
	.quirks = DW_UART_QUIRK_CPR_VALUE | DW_UART_QUIRK_IS_DMA_FC,
};

static const struct dw8250_platform_data dw8250_skip_set_rate_data = {
	.usr_reg = DW_UART_USR,
	.quirks = DW_UART_QUIRK_SKIP_SET_RATE,
};

static const struct dw8250_platform_data dw8250_h616_data = {
	.usr_reg = DW_UART_USR,
	.cpr_value = DW_UART_CPR_FIFO_MODE_FROM_SIZE(256),
	.type = UART_PORT_SUNXI,
	/* In DMA mode 0, one received byte can assert the request. */
	.rx_maxburst = 1,
	.tx_maxburst = 1,
	.cyclic_rx = true,
	.quirks = DW_UART_QUIRK_CPR_VALUE | DW_UART_QUIRK_SKIP_SET_RATE |
		  DW_UART_QUIRK_NO_DLF,
};

static const struct dw8250_platform_data dw8250_intc10ee = {
	.usr_reg = DW_UART_USR,
	.quirks = DW_UART_QUIRK_IER_KICK,
};

static const struct dw8250_platform_data dw8250_ultrarisc_dp1000_data = {
	.usr_reg = DW_UART_USR,
	.cpr_value = FIELD_PREP_CONST(DW_UART_CPR_ABP_DATA_WIDTH, 2) |
		     DW_UART_CPR_THRE_MODE |
		     DW_UART_CPR_DMA_EXTRA |
		     DW_UART_CPR_FIFO_MODE_FROM_SIZE(32),
	.quirks = DW_UART_QUIRK_CPR_VALUE,
};

static const struct dw8250_platform_data dw8250_tda54 = {
	.usr_reg = DW_UART_USR,
	.quirks = DW_UART_QUIRK_SKIP_EMPTY_FIFO_READ,
};

static const struct of_device_id dw8250_of_match[] = {
	{ .compatible = "snps,dw-apb-uart", .data = &dw8250_dw_apb },
	{ .compatible = "allwinner,sun50i-h616-uart", .data = &dw8250_h616_data },
	{ .compatible = "cavium,octeon-3860-uart", .data = &dw8250_octeon_3860_data },
	{ .compatible = "marvell,armada-38x-uart", .data = &dw8250_armada_38x_data },
	{ .compatible = "renesas,rzn1-uart", .data = &dw8250_renesas_rzn1_data },
	{ .compatible = "sophgo,sg2044-uart", .data = &dw8250_skip_set_rate_data },
	{ .compatible = "starfive,jh7100-uart", .data = &dw8250_skip_set_rate_data },
	{ .compatible = "ti,tda54-uart", .data = &dw8250_tda54 },
	{ .compatible = "ultrarisc,dp1000-uart", .data = &dw8250_ultrarisc_dp1000_data },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw8250_of_match);

static const struct dw8250_platform_data dw8250_apmc0d08 = {
	.usr_reg = DW_UART_USR,
	.quirks = DW_UART_QUIRK_APMC0D08,
};

static const struct acpi_device_id dw8250_acpi_match[] = {
	{ "80860F0A", (kernel_ulong_t)&dw8250_dw_apb },
	{ "8086228A", (kernel_ulong_t)&dw8250_dw_apb },
	{ "AMD0020", (kernel_ulong_t)&dw8250_dw_apb },
	{ "AMDI0020", (kernel_ulong_t)&dw8250_dw_apb },
	{ "AMDI0021", (kernel_ulong_t)&dw8250_dw_apb },
	{ "AMDI0022", (kernel_ulong_t)&dw8250_dw_apb },
	{ "APMC0D08", (kernel_ulong_t)&dw8250_apmc0d08 },
	{ "BRCM2032", (kernel_ulong_t)&dw8250_dw_apb },
	{ "HISI0031", (kernel_ulong_t)&dw8250_dw_apb },
	{ "INT33C4", (kernel_ulong_t)&dw8250_dw_apb },
	{ "INT33C5", (kernel_ulong_t)&dw8250_dw_apb },
	{ "INT3434", (kernel_ulong_t)&dw8250_dw_apb },
	{ "INT3435", (kernel_ulong_t)&dw8250_dw_apb },
	{ "INTC10EE", (kernel_ulong_t)&dw8250_intc10ee },
	{ },
};
MODULE_DEVICE_TABLE(acpi, dw8250_acpi_match);

static struct platform_driver dw8250_platform_driver = {
	.driver = {
		.name		= "dw-apb-uart",
		.pm		= pm_ptr(&dw8250_pm_ops),
		.of_match_table	= dw8250_of_match,
		.acpi_match_table = dw8250_acpi_match,
	},
	.probe			= dw8250_probe,
	.remove			= dw8250_remove,
};

module_platform_driver(dw8250_platform_driver);

MODULE_IMPORT_NS("SERIAL_8250");
MODULE_AUTHOR("Jamie Iles");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Synopsys DesignWare 8250 serial port driver");
MODULE_ALIAS("platform:dw-apb-uart");
