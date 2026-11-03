// SPDX-License-Identifier: GPL-2.0+
/* Synopsys DesignWare 8250 library. */

#include <linux/array_size.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>

#include "8250_dwlib.h"

/* DesignWare FIFO-relative receive thresholds */
#define DW_UART_RXTRIG_QUARTER_DIV	4
#define DW_UART_RXTRIG_HALF_DIV		2
#define DW_UART_RXTRIG_FULL_MARGIN	2

/*
 * divisor = div(I) + div(F)
 * "I" means integer, "F" means fractional
 * quot = div(I) = clk / (16 * baud)
 * frac = div(F) * 2^dlf_size
 *
 * let rem = clk % (16 * baud)
 * we have: div(F) * (16 * baud) = rem
 * so frac = 2^dlf_size * rem / (16 * baud) = (rem << dlf_size) / (16 * baud)
 */
unsigned int dw8250_get_divisor_dlf(struct uart_port *p, unsigned int baud,
				    unsigned int *frac)
{
	unsigned int quot, rem, base_baud = baud * 16;
	struct dw8250_port_data *d = p->private_data;

	quot = p->uartclk / base_baud;
	rem = p->uartclk % base_baud;
	*frac = DIV_ROUND_CLOSEST(rem << d->dlf_size, base_baud);

	return quot;
}
EXPORT_SYMBOL_GPL(dw8250_get_divisor_dlf);

static unsigned int dw8250_get_divisor(struct uart_port *p, unsigned int baud,
				       unsigned int *frac)
{
	return dw8250_get_divisor_dlf(p, baud, frac);
}

void dw8250_set_divisor_dlf(struct uart_port *p, unsigned int quot_frac)
{
	struct dw8250_port_data *pd = p->private_data;

	if (!pd->dlf_size)
		return;

	dw8250_writel_ext(p, DW_UART_DLF, quot_frac);
}
EXPORT_SYMBOL_GPL(dw8250_set_divisor_dlf);

static void dw8250_set_divisor(struct uart_port *p, unsigned int baud,
			       unsigned int quot, unsigned int quot_frac)
{
	dw8250_set_divisor_dlf(p, quot_frac);
	serial8250_do_set_divisor(p, baud, quot);
}

void dw8250_do_set_termios(struct uart_port *p, struct ktermios *termios,
			   const struct ktermios *old)
{
	p->status &= ~UPSTAT_AUTOCTS;
	if (termios->c_cflag & CRTSCTS)
		p->status |= UPSTAT_AUTOCTS;

	serial8250_do_set_termios(p, termios, old);

	/* Filter addresses which have 9th bit set */
	p->ignore_status_mask |= DW_UART_LSR_ADDR_RCVD;
	p->read_status_mask |= DW_UART_LSR_ADDR_RCVD;
}
EXPORT_SYMBOL_GPL(dw8250_do_set_termios);

/*
 * Wait until re is de-asserted for sure. An ongoing receive will keep
 * re asserted until end of frame. Without BUSY indication available,
 * only available course of action is to wait for the time it takes to
 * receive one frame (there might nothing to receive but w/o BUSY the
 * driver cannot know).
 */
static void dw8250_wait_re_deassert(struct uart_port *p)
{
	ndelay(p->frame_time);
}

static void dw8250_update_rar(struct uart_port *p, u32 addr)
{
	u32 re_en = dw8250_readl_ext(p, DW_UART_RE_EN);

	/*
	 * RAR shouldn't be changed while receiving. Thus, de-assert RE_EN
	 * if asserted and wait.
	 */
	if (re_en)
		dw8250_writel_ext(p, DW_UART_RE_EN, 0);
	dw8250_wait_re_deassert(p);
	dw8250_writel_ext(p, DW_UART_RAR, addr);
	if (re_en)
		dw8250_writel_ext(p, DW_UART_RE_EN, re_en);
}

static void dw8250_rs485_set_addr(struct uart_port *p, struct serial_rs485 *rs485,
				  struct ktermios *termios)
{
	u32 lcr = dw8250_readl_ext(p, DW_UART_LCR_EXT);

	if (rs485->flags & SER_RS485_ADDRB) {
		lcr |= DW_UART_LCR_EXT_DLS_E;
		if (termios)
			termios->c_cflag |= ADDRB;

		if (rs485->flags & SER_RS485_ADDR_RECV) {
			u32 delta = p->rs485.flags ^ rs485->flags;

			/*
			 * rs485 (param) is equal to uart_port's rs485 only during init
			 * (during init, delta is not yet applicable).
			 */
			if (unlikely(&p->rs485 == rs485))
				delta = rs485->flags;

			if ((delta & SER_RS485_ADDR_RECV) ||
			    (p->rs485.addr_recv != rs485->addr_recv))
				dw8250_update_rar(p, rs485->addr_recv);
			lcr |= DW_UART_LCR_EXT_ADDR_MATCH;
		} else {
			lcr &= ~DW_UART_LCR_EXT_ADDR_MATCH;
		}
		if (rs485->flags & SER_RS485_ADDR_DEST) {
			/*
			 * Don't skip writes here as another endpoint could
			 * have changed communication line's destination
			 * address in between.
			 */
			dw8250_writel_ext(p, DW_UART_TAR, rs485->addr_dest);
			lcr |= DW_UART_LCR_EXT_SEND_ADDR;
		}
	} else {
		lcr = 0;
	}
	dw8250_writel_ext(p, DW_UART_LCR_EXT, lcr);
}

static int dw8250_rs485_config(struct uart_port *p, struct ktermios *termios,
			       struct serial_rs485 *rs485)
{
	u32 tcr;

	tcr = dw8250_readl_ext(p, DW_UART_TCR);
	tcr &= ~DW_UART_TCR_XFER_MODE;

	if (rs485->flags & SER_RS485_ENABLED) {
		tcr |= DW_UART_TCR_RS485_EN;

		if (rs485->flags & SER_RS485_RX_DURING_TX)
			tcr |= DW_UART_TCR_XFER_MODE_DE_DURING_RE;
		else
			tcr |= DW_UART_TCR_XFER_MODE_DE_OR_RE;
		dw8250_writel_ext(p, DW_UART_DE_EN, 1);
		dw8250_writel_ext(p, DW_UART_RE_EN, 1);
	} else {
		if (termios)
			termios->c_cflag &= ~ADDRB;

		tcr &= ~DW_UART_TCR_RS485_EN;
	}

	/* Reset to default polarity */
	tcr |= DW_UART_TCR_DE_POL;
	tcr &= ~DW_UART_TCR_RE_POL;

	if (!(rs485->flags & SER_RS485_RTS_ON_SEND))
		tcr &= ~DW_UART_TCR_DE_POL;
	if (device_property_read_bool(p->dev, "rs485-rx-active-high"))
		tcr |= DW_UART_TCR_RE_POL;

	dw8250_writel_ext(p, DW_UART_TCR, tcr);

	/* Addressing mode can only be set up after TCR */
	if (rs485->flags & SER_RS485_ENABLED)
		dw8250_rs485_set_addr(p, rs485, termios);

	return 0;
}

/*
 * Tests if RE_EN register can have non-zero value to see if RS-485 HW support
 * is present.
 */
static bool dw8250_detect_rs485_hw(struct uart_port *p)
{
	u32 reg;

	dw8250_writel_ext(p, DW_UART_RE_EN, 1);
	reg = dw8250_readl_ext(p, DW_UART_RE_EN);
	dw8250_writel_ext(p, DW_UART_RE_EN, 0);
	return reg;
}

static const struct serial_rs485 dw8250_rs485_supported = {
	.flags = SER_RS485_ENABLED | SER_RS485_RX_DURING_TX | SER_RS485_RTS_ON_SEND |
		 SER_RS485_RTS_AFTER_SEND | SER_RS485_ADDRB | SER_RS485_ADDR_RECV |
		 SER_RS485_ADDR_DEST,
};

/* The trigger is the masked FCR receive trigger field, in register position */
static unsigned int dw8250_rxtrig_bytes(struct uart_port *p, unsigned int trigger)
{
	switch (trigger) {
	case UART_FCR_TRIGGER_1:
		return 1;
	case UART_FCR_TRIGGER_4:
		return p->fifosize / DW_UART_RXTRIG_QUARTER_DIV;
	case UART_FCR_TRIGGER_8:
		return p->fifosize / DW_UART_RXTRIG_HALF_DIV;
	default:
		return p->fifosize - DW_UART_RXTRIG_FULL_MARGIN;
	}
}

static int dw8250_get_rxtrig(struct uart_port *p)
{
	struct dw8250_port_data *pd = p->private_data;
	struct uart_8250_port *up = up_to_u8250p(p);

	guard(uart_port_lock_irqsave)(p);

	return pd->rxtrig_bytes[UART_FCR_R_TRIG_BITS(up->fcr)];
}

static int dw8250_set_rxtrig(struct uart_port *p, u16 bytes)
{
	struct dw8250_port_data *pd = p->private_data;
	struct uart_8250_port *up = up_to_u8250p(p);
	int fcr, ret;

	fcr = serial8250_bytes_to_fcr(pd->rxtrig_bytes, bytes);
	if (fcr < 0)
		return fcr;

	guard(mutex)(&p->state->port.mutex);

	PM_RUNTIME_ACQUIRE_IF_ENABLED(p->dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(uart_port_lock_irqsave)(p);

	up->fcr &= ~UART_FCR_TRIGGER_MASK;
	up->fcr |= fcr;
	/* Changing the trigger alone preserves the FIFO contents */
	serial_port_out(p, UART_FCR, up->fcr);

	return 0;
}

int dw8250_setup_rxtrig(struct uart_port *p)
{
	struct dw8250_port_data *pd = p->private_data;

	/* Each FIFO-relative threshold must be positive and fit in the table */
	if (p->fifosize < DW_UART_RXTRIG_QUARTER_DIV ||
	    p->fifosize - DW_UART_RXTRIG_FULL_MARGIN > U16_MAX)
		return dev_err_probe(p->dev, -EINVAL,
				     "invalid FIFO size %u\n", p->fifosize);

	for (unsigned int i = 0; i < ARRAY_SIZE(pd->rxtrig_bytes); i++) {
		unsigned int trigger = FIELD_PREP(UART_FCR_TRIGGER_MASK, i);

		pd->rxtrig_bytes[i] = dw8250_rxtrig_bytes(p, trigger);
	}
	p->get_rxtrig = dw8250_get_rxtrig;
	p->set_rxtrig = dw8250_set_rxtrig;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(dw8250_setup_rxtrig, "SERIAL_8250");

int dw8250_setup_port(struct uart_port *p)
{
	struct dw8250_port_data *pd = p->private_data;
	struct uart_8250_port *up = up_to_u8250p(p);
	u32 reg, old_dlf;
	int ret;

	pd->hw_rs485_support = dw8250_detect_rs485_hw(p);
	if (pd->hw_rs485_support) {
		p->rs485_config = dw8250_rs485_config;
		up->lsr_save_mask = LSR_SAVE_FLAGS | DW_UART_LSR_ADDR_RCVD;
		p->rs485_supported = dw8250_rs485_supported;
	} else {
		p->rs485_config = serial8250_em485_config;
		p->rs485_supported = serial8250_em485_supported;
		up->rs485_start_tx = serial8250_em485_start_tx;
		up->rs485_stop_tx = serial8250_em485_stop_tx;
	}
	up->capabilities |= UART_CAP_NOTEMT;

	/* Preserve value written by firmware or bootloader  */
	old_dlf = dw8250_readl_ext(p, DW_UART_DLF);
	dw8250_writel_ext(p, DW_UART_DLF, ~0U);
	reg = dw8250_readl_ext(p, DW_UART_DLF);
	dw8250_writel_ext(p, DW_UART_DLF, old_dlf);

	if (reg) {
		pd->dlf_size = fls(reg);
		p->get_divisor = dw8250_get_divisor;
		p->set_divisor = dw8250_set_divisor;
	}

	reg = dw8250_readl_ext(p, DW_UART_UCV);
	if (reg)
		dev_dbg(p->dev, "Designware UART version %c.%c%c\n",
			(reg >> 24) & 0xff, (reg >> 16) & 0xff, (reg >> 8) & 0xff);

	reg = dw8250_readl_ext(p, DW_UART_CPR);
	if (!reg) {
		reg = pd->cpr_value;
		dev_dbg(p->dev, "CPR is not available, using 0x%08x instead\n", reg);
	}
	if (!reg)
		return 0;

	/* Select the type based on FIFO */
	if (reg & DW_UART_CPR_FIFO_MODE) {
		p->type = PORT_16550A;
		p->flags |= UPF_FIXED_TYPE;
		p->fifosize = DW_UART_CPR_FIFO_SIZE(reg);
		up->capabilities = UART_CAP_FIFO | UART_CAP_NOTEMT;

		ret = dw8250_setup_rxtrig(p);
		if (ret)
			return ret;
	}

	if (reg & DW_UART_CPR_AFCE_MODE)
		up->capabilities |= UART_CAP_AFE;

	if (reg & DW_UART_CPR_SIR_MODE)
		up->capabilities |= UART_CAP_IRDA;

	if (reg & DW_UART_CPR_SHADOW)
		pd->srbr = DW_UART_SRBR_0;
	else
		pd->srbr = UART_RX;

	return 0;
}
EXPORT_SYMBOL_GPL(dw8250_setup_port);
