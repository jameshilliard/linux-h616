// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun4i MUSB Glue Layer
 *
 * Copyright (C) 2015 Hans de Goede <hdegoede@redhat.com>
 *
 * Based on code from
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/extcon.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy-sun4i-usb.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/soc/sunxi/sunxi_sram.h>
#include <linux/usb/musb.h>
#include <linux/usb/of.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/workqueue.h>
#include "musb_core.h"

/*
 * Register offsets, note sunxi musb has a different layout then most
 * musb implementations, we translate the layout in musb_readb & friends.
 */
#define SUNXI_MUSB_POWER			0x0040
#define SUNXI_MUSB_DEVCTL			0x0041
#define SUNXI_MUSB_INDEX			0x0042
#define SUNXI_MUSB_VEND0			0x0043
#define SUNXI_MUSB_INTRTX			0x0044
#define SUNXI_MUSB_INTRRX			0x0046
#define SUNXI_MUSB_INTRTXE			0x0048
#define SUNXI_MUSB_INTRRXE			0x004a
#define SUNXI_MUSB_INTRUSB			0x004c
#define SUNXI_MUSB_INTRUSBE			0x0050
#define SUNXI_MUSB_FRAME			0x0054
#define SUNXI_MUSB_TXFIFOSZ			0x0090
#define SUNXI_MUSB_TXFIFOADD			0x0092
#define SUNXI_MUSB_RXFIFOSZ			0x0094
#define SUNXI_MUSB_RXFIFOADD			0x0096
#define SUNXI_MUSB_FADDR			0x0098
#define SUNXI_MUSB_TXFUNCADDR			0x0098
#define SUNXI_MUSB_TXHUBADDR			0x009a
#define SUNXI_MUSB_TXHUBPORT			0x009b
#define SUNXI_MUSB_RXFUNCADDR			0x009c
#define SUNXI_MUSB_RXHUBADDR			0x009e
#define SUNXI_MUSB_RXHUBPORT			0x009f
#define SUNXI_MUSB_CONFIGDATA			0x00c0

/* VEND0 bits */
#define SUNXI_MUSB_VEND0_PIO_MODE		0
#define SUNXI_MUSB_VEND0_DMA_MODE		BIT(0)

/* DMA register offsets */
#define SUNXI_MUSB_DMA_INTE			0x0000
#define SUNXI_MUSB_DMA_INTS			0x0004
#define SUNXI_MUSB_DMA_CHAN_CFG(n)		(0x0040 + (0x10 * (n)))
#define SUNXI_MUSB_DMA_CHAN_ADDR(n)		(0x0044 + (0x10 * (n)))
#define SUNXI_MUSB_DMA_CHAN_COUNT(n)		(0x0048 + (0x10 * (n)))
#define SUNXI_MUSB_DMA_CHAN_RESIDUAL(n)		(0x004c + (0x10 * (n)))

#define SUNXI_MUSB_DMA_CHANNELS			8
#define SUNXI_MUSB_DMA_ADDR_ALIGN		4
#define SUNXI_MUSB_DMA_MAX_LEN			0x100000
#define SUNXI_MUSB_DMA_CFG_ENABLE		BIT(31)
#define SUNXI_MUSB_DMA_CFG_DIR_RX		BIT(4)
#define SUNXI_MUSB_DMA_CFG_EP_MASK		GENMASK(3, 0)
#define SUNXI_MUSB_DMA_CFG_PKTSIZE_MASK		GENMASK(26, 16)
#define SUNXI_MUSB_DMA_CFG_PKTSIZE_SHIFT	16
#define SUNXI_MUSB_DMA_INT_MASK			GENMASK(SUNXI_MUSB_DMA_CHANNELS - 1, 0)

/* flags */
#define SUNXI_MUSB_FL_ENABLED			0
#define SUNXI_MUSB_FL_HOSTMODE			1
#define SUNXI_MUSB_FL_HOSTMODE_PEND		2
#define SUNXI_MUSB_FL_VBUS_ON			3
#define SUNXI_MUSB_FL_PHY_ON			4
#define SUNXI_MUSB_FL_HAS_SRAM			5
#define SUNXI_MUSB_FL_HAS_RESET			6
#define SUNXI_MUSB_FL_NO_CONFIGDATA		7
#define SUNXI_MUSB_FL_PHY_MODE_PEND		8
#define SUNXI_MUSB_FL_HAS_DMA			9

struct sunxi_musb_cfg {
	const struct musb_hdrc_config *hdrc_config;
	bool has_sram;
	bool has_reset;
	bool no_configdata;
	bool has_dma;
};

/* Our read/write methods need access and do not get passed in a musb ref :| */
static struct musb *sunxi_musb;

struct sunxi_glue {
	struct device		*dev;
	struct musb		*musb;
	struct platform_device	*musb_pdev;
	struct clk		*clk;
	struct reset_control	*rst;
	struct phy		*phy;
	struct platform_device	*usb_phy;
	struct usb_phy		*xceiv;
	enum phy_mode		phy_mode;
	unsigned long		flags;
	struct work_struct	work;
	struct extcon_dev	*extcon;
	struct notifier_block	host_nb;
};

static bool sunxi_musb_dma_interrupt(struct musb *musb);

/* phy_power_on / off may sleep, so we use a workqueue  */
static void sunxi_musb_work(struct work_struct *work)
{
	struct sunxi_glue *glue = container_of(work, struct sunxi_glue, work);
	bool vbus_on, phy_on;

	if (!test_bit(SUNXI_MUSB_FL_ENABLED, &glue->flags))
		return;

	if (test_and_clear_bit(SUNXI_MUSB_FL_HOSTMODE_PEND, &glue->flags)) {
		struct musb *musb = glue->musb;
		unsigned long flags;
		u8 devctl;

		spin_lock_irqsave(&musb->lock, flags);

		devctl = readb(musb->mregs + SUNXI_MUSB_DEVCTL);
		if (test_bit(SUNXI_MUSB_FL_HOSTMODE, &glue->flags)) {
			set_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
			MUSB_HST_MODE(musb);
			devctl |= MUSB_DEVCTL_SESSION;
		} else {
			clear_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
			musb->xceiv->otg->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
			devctl &= ~MUSB_DEVCTL_SESSION;
		}
		writeb(devctl, musb->mregs + SUNXI_MUSB_DEVCTL);

		spin_unlock_irqrestore(&musb->lock, flags);
	}

	vbus_on = test_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
	phy_on = test_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags);

	if (phy_on != vbus_on) {
		if (vbus_on) {
			phy_power_on(glue->phy);
			set_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags);
		} else {
			phy_power_off(glue->phy);
			clear_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags);
		}
	}

	if (test_and_clear_bit(SUNXI_MUSB_FL_PHY_MODE_PEND, &glue->flags))
		phy_set_mode(glue->phy, glue->phy_mode);
}

static void sunxi_musb_set_vbus(struct musb *musb, int is_on)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	if (is_on) {
		set_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
	} else {
		clear_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
	}

	schedule_work(&glue->work);
}

static void sunxi_musb_pre_root_reset_end(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	sun4i_usb_phy_set_squelch_detect(glue->phy, false);
}

static void sunxi_musb_post_root_reset_end(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	sun4i_usb_phy_set_squelch_detect(glue->phy, true);
}

static irqreturn_t sunxi_musb_interrupt(int irq, void *__hci)
{
	struct musb *musb = __hci;
	unsigned long flags;
	irqreturn_t ret;
	bool dma_handled;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = readb(musb->mregs + SUNXI_MUSB_INTRUSB);
	if (musb->int_usb)
		writeb(musb->int_usb, musb->mregs + SUNXI_MUSB_INTRUSB);

	if ((musb->int_usb & MUSB_INTR_RESET) && !is_host_active(musb)) {
		/* ep0 FADDR must be 0 when (re)entering peripheral mode */
		musb_ep_select(musb->mregs, 0);
		musb_writeb(musb->mregs, MUSB_FADDR, 0);
	}

	musb->int_tx = readw(musb->mregs + SUNXI_MUSB_INTRTX);
	if (musb->int_tx)
		writew(musb->int_tx, musb->mregs + SUNXI_MUSB_INTRTX);

	musb->int_rx = readw(musb->mregs + SUNXI_MUSB_INTRRX);
	if (musb->int_rx)
		writew(musb->int_rx, musb->mregs + SUNXI_MUSB_INTRRX);

	ret = musb_interrupt(musb);
	dma_handled = sunxi_musb_dma_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return ret == IRQ_HANDLED || dma_handled ? IRQ_HANDLED : IRQ_NONE;
}

static int sunxi_musb_host_notifier(struct notifier_block *nb,
				    unsigned long event, void *ptr)
{
	struct sunxi_glue *glue = container_of(nb, struct sunxi_glue, host_nb);

	if (event)
		set_bit(SUNXI_MUSB_FL_HOSTMODE, &glue->flags);
	else
		clear_bit(SUNXI_MUSB_FL_HOSTMODE, &glue->flags);

	set_bit(SUNXI_MUSB_FL_HOSTMODE_PEND, &glue->flags);
	schedule_work(&glue->work);

	return NOTIFY_DONE;
}

static int sunxi_musb_init(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	int ret;

	sunxi_musb = musb;
	musb->phy = glue->phy;
	musb->xceiv = glue->xceiv;

	if (test_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags)) {
		ret = sunxi_sram_claim(musb->controller->parent);
		if (ret)
			return ret;
	}

	ret = clk_prepare_enable(glue->clk);
	if (ret)
		goto error_sram_release;

	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags)) {
		ret = reset_control_deassert(glue->rst);
		if (ret)
			goto error_clk_disable;
	}

	if (test_bit(SUNXI_MUSB_FL_HAS_DMA, &glue->flags))
		writeb(SUNXI_MUSB_VEND0_DMA_MODE,
		       musb->mregs + SUNXI_MUSB_VEND0);
	else
		writeb(SUNXI_MUSB_VEND0_PIO_MODE,
		       musb->mregs + SUNXI_MUSB_VEND0);

	/* Register notifier before calling phy_init() */
	ret = devm_extcon_register_notifier(glue->dev, glue->extcon,
					EXTCON_USB_HOST, &glue->host_nb);
	if (ret)
		goto error_reset_assert;

	ret = phy_init(glue->phy);
	if (ret)
		goto error_reset_assert;

	musb->isr = sunxi_musb_interrupt;

	/* Stop the musb-core from doing runtime pm (not supported on sunxi) */
	pm_runtime_get(musb->controller);

	return 0;

error_reset_assert:
	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags))
		reset_control_assert(glue->rst);
error_clk_disable:
	clk_disable_unprepare(glue->clk);
error_sram_release:
	if (test_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags))
		sunxi_sram_release(musb->controller->parent);
	return ret;
}

static int sunxi_musb_exit(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	pm_runtime_put(musb->controller);

	cancel_work_sync(&glue->work);
	if (test_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags))
		phy_power_off(glue->phy);

	phy_exit(glue->phy);

	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags))
		reset_control_assert(glue->rst);

	clk_disable_unprepare(glue->clk);
	if (test_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags))
		sunxi_sram_release(musb->controller->parent);

	return 0;
}

static void sunxi_musb_enable(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	glue->musb = musb;

	/* musb_core does not call us in a balanced manner */
	if (test_and_set_bit(SUNXI_MUSB_FL_ENABLED, &glue->flags))
		return;

	schedule_work(&glue->work);
}

static void sunxi_musb_disable(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	clear_bit(SUNXI_MUSB_FL_ENABLED, &glue->flags);
}

struct sunxi_musb_dma_controller;

struct sunxi_musb_dma_channel {
	struct dma_channel channel;
	struct sunxi_musb_dma_controller *controller;
	u32 len;
	u8 idx;
	u8 epnum;
	u8 transmit;
};

struct sunxi_musb_dma_controller {
	struct dma_controller controller;
	struct sunxi_musb_dma_channel channel[SUNXI_MUSB_DMA_CHANNELS];
	void __iomem *base;
	u8 used;
};

#ifdef CONFIG_USB_SUNXI_MUSB_DMA
static void sunxi_musb_dma_stop_channel(struct sunxi_musb_dma_channel *chan)
{
	struct sunxi_musb_dma_controller *controller = chan->controller;
	void __iomem *base = controller->base;
	u32 val;

	val = readl(base + SUNXI_MUSB_DMA_CHAN_CFG(chan->idx));
	val &= ~SUNXI_MUSB_DMA_CFG_ENABLE;
	writel(val, base + SUNXI_MUSB_DMA_CHAN_CFG(chan->idx));

	val = readl(base + SUNXI_MUSB_DMA_INTE);
	val &= ~BIT(chan->idx);
	writel(val, base + SUNXI_MUSB_DMA_INTE);

	writel(BIT(chan->idx), base + SUNXI_MUSB_DMA_INTS);
}

static size_t sunxi_musb_dma_actual_len(struct sunxi_musb_dma_channel *chan)
{
	struct sunxi_musb_dma_controller *controller = chan->controller;
	u32 residual;

	residual = readl(controller->base +
			 SUNXI_MUSB_DMA_CHAN_RESIDUAL(chan->idx));
	residual = min(residual, chan->len);

	return chan->len - residual;
}

static struct dma_channel *
sunxi_musb_dma_channel_alloc(struct dma_controller *c,
			     struct musb_hw_ep *hw_ep, u8 transmit)
{
	struct sunxi_musb_dma_controller *controller;
	struct sunxi_musb_dma_channel *chan;
	unsigned int i;

	if (!hw_ep->epnum)
		return NULL;

	controller = container_of(c, struct sunxi_musb_dma_controller,
				  controller);

	for (i = 0; i < SUNXI_MUSB_DMA_CHANNELS; i++) {
		if (controller->used & BIT(i))
			continue;

		controller->used |= BIT(i);
		chan = &controller->channel[i];
		chan->controller = controller;
		chan->idx = i;
		chan->epnum = hw_ep->epnum;
		chan->transmit = transmit;
		chan->len = 0;
		chan->channel.max_len = SUNXI_MUSB_DMA_MAX_LEN;
		chan->channel.actual_len = 0;
		chan->channel.status = MUSB_DMA_STATUS_FREE;
		chan->channel.desired_mode = transmit;

		return &chan->channel;
	}

	return NULL;
}

static void sunxi_musb_dma_channel_release(struct dma_channel *channel)
{
	struct sunxi_musb_dma_channel *chan = channel->private_data;

	if (channel->status == MUSB_DMA_STATUS_BUSY)
		sunxi_musb_dma_stop_channel(chan);

	channel->actual_len = 0;
	channel->status = MUSB_DMA_STATUS_UNKNOWN;
	chan->len = 0;
	chan->controller->used &= ~BIT(chan->idx);
}

static int sunxi_musb_dma_channel_program(struct dma_channel *channel,
					  u16 maxpacket, u8 mode,
					  dma_addr_t dma_addr, u32 len)
{
	struct sunxi_musb_dma_channel *chan = channel->private_data;
	struct sunxi_musb_dma_controller *controller = chan->controller;
	u32 cfg;

	if (channel->status == MUSB_DMA_STATUS_UNKNOWN ||
	    channel->status == MUSB_DMA_STATUS_BUSY)
		return false;

	if (!len || !IS_ALIGNED(dma_addr, SUNXI_MUSB_DMA_ADDR_ALIGN))
		return false;

	len = min_t(u32, len, SUNXI_MUSB_DMA_MAX_LEN);
	channel->actual_len = 0;
	channel->desired_mode = mode;
	channel->status = MUSB_DMA_STATUS_BUSY;
	chan->len = len;

	cfg = chan->epnum & SUNXI_MUSB_DMA_CFG_EP_MASK;
	cfg |= (maxpacket << SUNXI_MUSB_DMA_CFG_PKTSIZE_SHIFT) &
	       SUNXI_MUSB_DMA_CFG_PKTSIZE_MASK;
	if (!chan->transmit)
		cfg |= SUNXI_MUSB_DMA_CFG_DIR_RX;

	writel(0, controller->base + SUNXI_MUSB_DMA_CHAN_CFG(chan->idx));
	writel(lower_32_bits(dma_addr),
	       controller->base + SUNXI_MUSB_DMA_CHAN_ADDR(chan->idx));
	writel(len, controller->base + SUNXI_MUSB_DMA_CHAN_COUNT(chan->idx));
	writel(BIT(chan->idx), controller->base + SUNXI_MUSB_DMA_INTS);

	cfg |= SUNXI_MUSB_DMA_CFG_ENABLE;
	writel(readl(controller->base + SUNXI_MUSB_DMA_INTE) | BIT(chan->idx),
	       controller->base + SUNXI_MUSB_DMA_INTE);
	writel(cfg, controller->base + SUNXI_MUSB_DMA_CHAN_CFG(chan->idx));

	return true;
}

static int sunxi_musb_dma_channel_abort(struct dma_channel *channel)
{
	struct sunxi_musb_dma_channel *chan = channel->private_data;
	struct sunxi_musb_dma_controller *controller = chan->controller;
	struct musb *musb = controller->controller.musb;
	void __iomem *epio = musb->endpoints[chan->epnum].regs;
	u16 csr;

	if (channel->status != MUSB_DMA_STATUS_BUSY &&
	    channel->status != MUSB_DMA_STATUS_BUS_ABORT &&
	    channel->status != MUSB_DMA_STATUS_CORE_ABORT)
		return 0;

	if (chan->transmit) {
		csr = musb_readw(epio, MUSB_TXCSR);
		csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB |
			 MUSB_TXCSR_DMAMODE);
		musb_writew(epio, MUSB_TXCSR,
			    csr | (is_host_active(musb) ?
				   MUSB_TXCSR_H_WZC_BITS :
				   MUSB_TXCSR_P_WZC_BITS));
	} else {
		csr = musb_readw(epio, MUSB_RXCSR);
		csr &= ~(MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAENAB |
			 MUSB_RXCSR_DMAMODE);
		musb_writew(epio, MUSB_RXCSR,
			    csr | (is_host_active(musb) ?
				   MUSB_RXCSR_H_WZC_BITS :
				   MUSB_RXCSR_P_WZC_BITS));
	}

	channel->actual_len = sunxi_musb_dma_actual_len(chan);
	sunxi_musb_dma_stop_channel(chan);
	channel->status = MUSB_DMA_STATUS_FREE;

	return 0;
}

static int sunxi_musb_dma_is_compatible(struct dma_channel *channel,
					u16 maxpacket, void *buf, u32 len)
{
	return len && IS_ALIGNED((unsigned long)buf, SUNXI_MUSB_DMA_ADDR_ALIGN);
}

static struct dma_controller *
sunxi_musb_dma_controller_create(struct musb *musb, void __iomem *base)
{
	struct platform_device *pdev = to_platform_device(musb->controller);
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	struct sunxi_musb_dma_controller *controller;
	struct resource *res;
	unsigned int i;

	if (!test_bit(SUNXI_MUSB_FL_HAS_DMA, &glue->flags))
		return NULL;

	controller = kzalloc_obj(*controller, GFP_KERNEL);
	if (!controller)
		return ERR_PTR(-ENOMEM);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	if (!res) {
		kfree(controller);
		return NULL;
	}

	controller->base = devm_ioremap_resource(musb->controller, res);
	if (IS_ERR(controller->base)) {
		struct dma_controller *ret = ERR_CAST(controller->base);

		kfree(controller);
		return ret;
	}

	controller->controller.musb = musb;
	controller->controller.channel_alloc = sunxi_musb_dma_channel_alloc;
	controller->controller.channel_release = sunxi_musb_dma_channel_release;
	controller->controller.channel_program = sunxi_musb_dma_channel_program;
	controller->controller.channel_abort = sunxi_musb_dma_channel_abort;
	controller->controller.is_compatible = sunxi_musb_dma_is_compatible;

	for (i = 0; i < SUNXI_MUSB_DMA_CHANNELS; i++)
		controller->channel[i].channel.private_data =
			&controller->channel[i];

	writel(0, controller->base + SUNXI_MUSB_DMA_INTE);
	writel(SUNXI_MUSB_DMA_INT_MASK,
	       controller->base + SUNXI_MUSB_DMA_INTS);

	return &controller->controller;
}

static void sunxi_musb_dma_controller_destroy(struct dma_controller *c)
{
	struct sunxi_musb_dma_controller *controller;
	unsigned int i;

	if (!c)
		return;

	controller = container_of(c, struct sunxi_musb_dma_controller,
				  controller);

	writel(0, controller->base + SUNXI_MUSB_DMA_INTE);
	for (i = 0; i < SUNXI_MUSB_DMA_CHANNELS; i++)
		writel(0, controller->base + SUNXI_MUSB_DMA_CHAN_CFG(i));
	writel(SUNXI_MUSB_DMA_INT_MASK,
	       controller->base + SUNXI_MUSB_DMA_INTS);

	kfree(controller);
}

static bool sunxi_musb_dma_interrupt(struct musb *musb)
{
	struct sunxi_musb_dma_controller *controller;
	u32 pending;
	unsigned int i;

	if (!musb->dma_controller || !musb_dma_sunxi(musb))
		return false;

	controller = container_of(musb->dma_controller,
				  struct sunxi_musb_dma_controller, controller);
	pending = readl(controller->base + SUNXI_MUSB_DMA_INTS) &
		  SUNXI_MUSB_DMA_INT_MASK;
	if (!pending)
		return false;

	for (i = 0; i < SUNXI_MUSB_DMA_CHANNELS; i++) {
		struct sunxi_musb_dma_channel *chan = &controller->channel[i];
		struct dma_channel *channel = &chan->channel;

		if (!(pending & BIT(i)))
			continue;

		if (channel->status != MUSB_DMA_STATUS_BUSY) {
			writel(BIT(i), controller->base + SUNXI_MUSB_DMA_INTS);
			continue;
		}

		channel->actual_len = sunxi_musb_dma_actual_len(chan);
		sunxi_musb_dma_stop_channel(chan);
		channel->status = MUSB_DMA_STATUS_FREE;
		musb_dma_completion(musb, chan->epnum, chan->transmit);
	}

	return true;
}
#else
static struct dma_controller *
sunxi_musb_dma_controller_create(struct musb *musb, void __iomem *base)
{
	return NULL;
}

static void sunxi_musb_dma_controller_destroy(struct dma_controller *c)
{
}

static bool sunxi_musb_dma_interrupt(struct musb *musb)
{
	return false;
}
#endif

static int sunxi_musb_set_mode(struct musb *musb, u8 mode)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	enum phy_mode new_mode;

	switch (mode) {
	case MUSB_HOST:
		new_mode = PHY_MODE_USB_HOST;
		break;
	case MUSB_PERIPHERAL:
		new_mode = PHY_MODE_USB_DEVICE;
		break;
	case MUSB_OTG:
		new_mode = PHY_MODE_USB_OTG;
		break;
	default:
		dev_err(musb->controller->parent,
			"Error requested mode not supported by this kernel\n");
		return -EINVAL;
	}

	if (glue->phy_mode == new_mode)
		return 0;

	if (musb->port_mode != MUSB_OTG) {
		dev_err(musb->controller->parent,
			"Error changing modes is only supported in dual role mode\n");
		return -EINVAL;
	}

	if (musb->port1_status & USB_PORT_STAT_ENABLE)
		musb_root_disconnect(musb);

	/*
	 * phy_set_mode may sleep, and we're called with a spinlock held,
	 * so let sunxi_musb_work deal with it.
	 */
	glue->phy_mode = new_mode;
	set_bit(SUNXI_MUSB_FL_PHY_MODE_PEND, &glue->flags);
	schedule_work(&glue->work);

	return 0;
}

static int sunxi_musb_recover(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	/*
	 * Schedule a phy_set_mode with the current glue->phy_mode value,
	 * this will force end the current session.
	 */
	set_bit(SUNXI_MUSB_FL_PHY_MODE_PEND, &glue->flags);
	schedule_work(&glue->work);

	return 0;
}

/*
 * sunxi musb register layout
 * 0x00 - 0x17	fifo regs, 1 long per fifo
 * 0x40 - 0x57	generic control regs (power - frame)
 * 0x80 - 0x8f	ep control regs (addressed through hw_ep->regs, indexed)
 * 0x90 - 0x97	fifo control regs (indexed)
 * 0x98 - 0x9f	multipoint / busctl regs (indexed)
 * 0xc0		configdata reg
 */

static u32 sunxi_musb_fifo_offset(u8 epnum)
{
	return (epnum * 4);
}

static u32 sunxi_musb_ep_offset(u8 epnum, u16 offset)
{
	WARN_ONCE(offset != 0,
		  "sunxi_musb_ep_offset called with non 0 offset\n");

	return 0x80; /* indexed, so ignore epnum */
}

static u32 sunxi_musb_busctl_offset(u8 epnum, u16 offset)
{
	return SUNXI_MUSB_TXFUNCADDR + offset;
}

static u8 sunxi_musb_readb(void __iomem *addr, u32 offset)
{
	struct sunxi_glue *glue;

	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_FADDR:
			return readb(addr + SUNXI_MUSB_FADDR);
		case MUSB_POWER:
			return readb(addr + SUNXI_MUSB_POWER);
		case MUSB_INTRUSB:
			return readb(addr + SUNXI_MUSB_INTRUSB);
		case MUSB_INTRUSBE:
			return readb(addr + SUNXI_MUSB_INTRUSBE);
		case MUSB_INDEX:
			return readb(addr + SUNXI_MUSB_INDEX);
		case MUSB_TESTMODE:
			return 0; /* No testmode on sunxi */
		case MUSB_DEVCTL:
			return readb(addr + SUNXI_MUSB_DEVCTL);
		case MUSB_TXFIFOSZ:
			return readb(addr + SUNXI_MUSB_TXFIFOSZ);
		case MUSB_RXFIFOSZ:
			return readb(addr + SUNXI_MUSB_RXFIFOSZ);
		case MUSB_CONFIGDATA + 0x10: /* See musb_read_configdata() */
			glue = dev_get_drvdata(sunxi_musb->controller->parent);
			/* A33 saves a reg, and we get to hardcode this */
			if (test_bit(SUNXI_MUSB_FL_NO_CONFIGDATA,
				     &glue->flags))
				return 0xde;

			return readb(addr + SUNXI_MUSB_CONFIGDATA);
		case MUSB_ULPI_BUSCONTROL:
			dev_warn(sunxi_musb->controller->parent,
				"sunxi-musb does not have ULPI bus control register\n");
			return 0;
		/* Offset for these is fixed by sunxi_musb_busctl_offset() */
		case SUNXI_MUSB_TXFUNCADDR:
		case SUNXI_MUSB_TXHUBADDR:
		case SUNXI_MUSB_TXHUBPORT:
		case SUNXI_MUSB_RXFUNCADDR:
		case SUNXI_MUSB_RXHUBADDR:
		case SUNXI_MUSB_RXHUBPORT:
			/* multipoint / busctl reg access */
			return readb(addr + offset);
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown readb offset %u\n", offset);
			return 0;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		/* sunxi has a 2 byte hole before the txtype register */
		if (offset >= MUSB_TXTYPE)
			offset += 2;
		return readb(addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown readb at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
	return 0;
}

static void sunxi_musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_FADDR:
			return writeb(data, addr + SUNXI_MUSB_FADDR);
		case MUSB_POWER:
			return writeb(data, addr + SUNXI_MUSB_POWER);
		case MUSB_INTRUSB:
			return writeb(data, addr + SUNXI_MUSB_INTRUSB);
		case MUSB_INTRUSBE:
			return writeb(data, addr + SUNXI_MUSB_INTRUSBE);
		case MUSB_INDEX:
			return writeb(data, addr + SUNXI_MUSB_INDEX);
		case MUSB_TESTMODE:
			if (data)
				dev_warn(sunxi_musb->controller->parent,
					"sunxi-musb does not have testmode\n");
			return;
		case MUSB_DEVCTL:
			return writeb(data, addr + SUNXI_MUSB_DEVCTL);
		case MUSB_TXFIFOSZ:
			return writeb(data, addr + SUNXI_MUSB_TXFIFOSZ);
		case MUSB_RXFIFOSZ:
			return writeb(data, addr + SUNXI_MUSB_RXFIFOSZ);
		case MUSB_ULPI_BUSCONTROL:
			dev_warn(sunxi_musb->controller->parent,
				"sunxi-musb does not have ULPI bus control register\n");
			return;
		/* Offset for these is fixed by sunxi_musb_busctl_offset() */
		case SUNXI_MUSB_TXFUNCADDR:
		case SUNXI_MUSB_TXHUBADDR:
		case SUNXI_MUSB_TXHUBPORT:
		case SUNXI_MUSB_RXFUNCADDR:
		case SUNXI_MUSB_RXHUBADDR:
		case SUNXI_MUSB_RXHUBPORT:
			/* multipoint / busctl reg access */
			return writeb(data, addr + offset);
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown writeb offset %u\n", offset);
			return;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		if (offset >= MUSB_TXTYPE)
			offset += 2;
		return writeb(data, addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown writeb at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
}

static u16 sunxi_musb_readw(void __iomem *addr, u32 offset)
{
	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_INTRTX:
			return readw(addr + SUNXI_MUSB_INTRTX);
		case MUSB_INTRRX:
			return readw(addr + SUNXI_MUSB_INTRRX);
		case MUSB_INTRTXE:
			return readw(addr + SUNXI_MUSB_INTRTXE);
		case MUSB_INTRRXE:
			return readw(addr + SUNXI_MUSB_INTRRXE);
		case MUSB_FRAME:
			return readw(addr + SUNXI_MUSB_FRAME);
		case MUSB_TXFIFOADD:
			return readw(addr + SUNXI_MUSB_TXFIFOADD);
		case MUSB_RXFIFOADD:
			return readw(addr + SUNXI_MUSB_RXFIFOADD);
		case MUSB_HWVERS:
			return 0; /* sunxi musb version is not known */
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown readw offset %u\n", offset);
			return 0;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		return readw(addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown readw at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
	return 0;
}

static void sunxi_musb_writew(void __iomem *addr, unsigned offset, u16 data)
{
	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_INTRTX:
			return writew(data, addr + SUNXI_MUSB_INTRTX);
		case MUSB_INTRRX:
			return writew(data, addr + SUNXI_MUSB_INTRRX);
		case MUSB_INTRTXE:
			return writew(data, addr + SUNXI_MUSB_INTRTXE);
		case MUSB_INTRRXE:
			return writew(data, addr + SUNXI_MUSB_INTRRXE);
		case MUSB_FRAME:
			return writew(data, addr + SUNXI_MUSB_FRAME);
		case MUSB_TXFIFOADD:
			return writew(data, addr + SUNXI_MUSB_TXFIFOADD);
		case MUSB_RXFIFOADD:
			return writew(data, addr + SUNXI_MUSB_RXFIFOADD);
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown writew offset %u\n", offset);
			return;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		return writew(data, addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown writew at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
}

static const struct musb_platform_ops sunxi_musb_ops = {
	.quirks		= MUSB_INDEXED_EP,
	.init		= sunxi_musb_init,
	.exit		= sunxi_musb_exit,
	.enable		= sunxi_musb_enable,
	.disable	= sunxi_musb_disable,
	.fifo_offset	= sunxi_musb_fifo_offset,
	.ep_offset	= sunxi_musb_ep_offset,
	.busctl_offset	= sunxi_musb_busctl_offset,
	.readb		= sunxi_musb_readb,
	.writeb		= sunxi_musb_writeb,
	.readw		= sunxi_musb_readw,
	.writew		= sunxi_musb_writew,
	.dma_init	= sunxi_musb_dma_controller_create,
	.dma_exit	= sunxi_musb_dma_controller_destroy,
	.set_mode	= sunxi_musb_set_mode,
	.recover	= sunxi_musb_recover,
	.set_vbus	= sunxi_musb_set_vbus,
	.pre_root_reset_end = sunxi_musb_pre_root_reset_end,
	.post_root_reset_end = sunxi_musb_post_root_reset_end,
};

static const struct musb_platform_ops sunxi_musb_dma_ops = {
	.quirks		= MUSB_INDEXED_EP | MUSB_DMA_SUNXI,
	.init		= sunxi_musb_init,
	.exit		= sunxi_musb_exit,
	.enable		= sunxi_musb_enable,
	.disable	= sunxi_musb_disable,
	.fifo_offset	= sunxi_musb_fifo_offset,
	.ep_offset	= sunxi_musb_ep_offset,
	.busctl_offset	= sunxi_musb_busctl_offset,
	.readb		= sunxi_musb_readb,
	.writeb		= sunxi_musb_writeb,
	.readw		= sunxi_musb_readw,
	.writew		= sunxi_musb_writew,
	.dma_init	= sunxi_musb_dma_controller_create,
	.dma_exit	= sunxi_musb_dma_controller_destroy,
	.set_mode	= sunxi_musb_set_mode,
	.recover	= sunxi_musb_recover,
	.set_vbus	= sunxi_musb_set_vbus,
	.pre_root_reset_end = sunxi_musb_pre_root_reset_end,
	.post_root_reset_end = sunxi_musb_post_root_reset_end,
};

#define SUNXI_MUSB_RAM_BITS	11

/* Allwinner OTG supports up to 5 endpoints */
static const struct musb_fifo_cfg sunxi_musb_mode_cfg_5eps[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_RX, 512),
};

/* H3/V3s OTG supports only 4 endpoints */
static const struct musb_fifo_cfg sunxi_musb_mode_cfg_4eps[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
};

static const struct musb_hdrc_config sunxi_musb_hdrc_config_5eps = {
	.fifo_cfg       = sunxi_musb_mode_cfg_5eps,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg_5eps),
	.multipoint	= true,
	.dyn_fifo	= true,
	/* Two FIFOs per endpoint, plus ep_0. */
	.num_eps	= (ARRAY_SIZE(sunxi_musb_mode_cfg_5eps) / 2) + 1,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
};

static const struct musb_hdrc_config sunxi_musb_hdrc_config_4eps = {
	.fifo_cfg       = sunxi_musb_mode_cfg_4eps,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg_4eps),
	.multipoint	= true,
	.dyn_fifo	= true,
	/* Two FIFOs per endpoint, plus ep_0. */
	.num_eps	= (ARRAY_SIZE(sunxi_musb_mode_cfg_4eps) / 2) + 1,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
};

static int sunxi_musb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	pdata;
	struct platform_device_info	pinfo;
	struct sunxi_glue		*glue;
	struct device_node		*np = pdev->dev.of_node;
	const struct sunxi_musb_cfg	*cfg;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Error no device tree node found\n");
		return -EINVAL;
	}

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	memset(&pdata, 0, sizeof(pdata));
	switch (usb_get_dr_mode(&pdev->dev)) {
#if defined CONFIG_USB_MUSB_DUAL_ROLE || defined CONFIG_USB_MUSB_HOST
	case USB_DR_MODE_HOST:
		pdata.mode = MUSB_HOST;
		glue->phy_mode = PHY_MODE_USB_HOST;
		break;
#endif
#if defined CONFIG_USB_MUSB_DUAL_ROLE || defined CONFIG_USB_MUSB_GADGET
	case USB_DR_MODE_PERIPHERAL:
		pdata.mode = MUSB_PERIPHERAL;
		glue->phy_mode = PHY_MODE_USB_DEVICE;
		break;
#endif
#ifdef CONFIG_USB_MUSB_DUAL_ROLE
	case USB_DR_MODE_OTG:
		pdata.mode = MUSB_OTG;
		glue->phy_mode = PHY_MODE_USB_OTG;
		break;
#endif
	default:
		dev_err(&pdev->dev, "Invalid or missing 'dr_mode' property\n");
		return -EINVAL;
	}
	pdata.platform_ops	= &sunxi_musb_ops;

	cfg = of_device_get_match_data(&pdev->dev);
	if (!cfg)
		return -EINVAL;

	pdata.config = cfg->hdrc_config;

	glue->dev = &pdev->dev;
	INIT_WORK(&glue->work, sunxi_musb_work);
	glue->host_nb.notifier_call = sunxi_musb_host_notifier;

	if (cfg->has_sram)
		set_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags);

	if (cfg->has_reset)
		set_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags);

	if (cfg->no_configdata)
		set_bit(SUNXI_MUSB_FL_NO_CONFIGDATA, &glue->flags);

	if (cfg->has_dma && IS_ENABLED(CONFIG_USB_SUNXI_MUSB_DMA)) {
		set_bit(SUNXI_MUSB_FL_HAS_DMA, &glue->flags);
		pdata.platform_ops = &sunxi_musb_dma_ops;
	}

	glue->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(glue->clk)) {
		dev_err(&pdev->dev, "Error getting clock: %ld\n",
			PTR_ERR(glue->clk));
		return PTR_ERR(glue->clk);
	}

	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags)) {
		glue->rst = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(glue->rst))
			return dev_err_probe(&pdev->dev, PTR_ERR(glue->rst),
					     "Error getting reset\n");
	}

	glue->extcon = extcon_get_edev_by_phandle(&pdev->dev, 0);
	if (IS_ERR(glue->extcon))
		return dev_err_probe(&pdev->dev, PTR_ERR(glue->extcon),
				     "Invalid or missing extcon\n");

	glue->phy = devm_phy_get(&pdev->dev, "usb");
	if (IS_ERR(glue->phy))
		return dev_err_probe(&pdev->dev, PTR_ERR(glue->phy),
				     "Error getting phy\n");

	glue->usb_phy = usb_phy_generic_register();
	if (IS_ERR(glue->usb_phy)) {
		dev_err(&pdev->dev, "Error registering usb-phy %ld\n",
			PTR_ERR(glue->usb_phy));
		return PTR_ERR(glue->usb_phy);
	}

	glue->xceiv = devm_usb_get_phy(&pdev->dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(glue->xceiv)) {
		ret = PTR_ERR(glue->xceiv);
		dev_err(&pdev->dev, "Error getting usb-phy %d\n", ret);
		goto err_unregister_usb_phy;
	}

	platform_set_drvdata(pdev, glue);

	memset(&pinfo, 0, sizeof(pinfo));
	pinfo.name	 = "musb-hdrc";
	pinfo.id	= PLATFORM_DEVID_AUTO;
	pinfo.parent	= &pdev->dev;
	pinfo.fwnode	= of_fwnode_handle(pdev->dev.of_node);
	pinfo.of_node_reused = true;
	pinfo.res	= pdev->resource;
	pinfo.num_res	= pdev->num_resources;
	pinfo.data	= &pdata;
	pinfo.size_data = sizeof(pdata);
	if (test_bit(SUNXI_MUSB_FL_HAS_DMA, &glue->flags))
		pinfo.dma_mask = DMA_BIT_MASK(32);

	glue->musb_pdev = platform_device_register_full(&pinfo);
	if (IS_ERR(glue->musb_pdev)) {
		ret = PTR_ERR(glue->musb_pdev);
		dev_err(&pdev->dev, "Error registering musb dev: %d\n", ret);
		goto err_unregister_usb_phy;
	}

	return 0;

err_unregister_usb_phy:
	usb_phy_generic_unregister(glue->usb_phy);
	return ret;
}

static void sunxi_musb_remove(struct platform_device *pdev)
{
	struct sunxi_glue *glue = platform_get_drvdata(pdev);
	struct platform_device *usb_phy = glue->usb_phy;

	platform_device_unregister(glue->musb_pdev);
	usb_phy_generic_unregister(usb_phy);
}

static const struct sunxi_musb_cfg sun4i_a10_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_sram = true,
};

static const struct sunxi_musb_cfg sun6i_a31_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_reset = true,
};

static const struct sunxi_musb_cfg sun8i_a33_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_reset = true,
	.no_configdata = true,
};

static const struct sunxi_musb_cfg sun8i_h3_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_4eps,
	.has_reset = true,
	.no_configdata = true,
};

static const struct sunxi_musb_cfg sun50i_h616_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_4eps,
	.has_reset = true,
	.no_configdata = true,
	.has_dma = true,
};

static const struct sunxi_musb_cfg suniv_f1c100s_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_sram = true,
	.has_reset = true,
	.no_configdata = true,
};

static const struct of_device_id sunxi_musb_match[] = {
	{ .compatible = "allwinner,sun4i-a10-musb",
	  .data = &sun4i_a10_musb_cfg, },
	{ .compatible = "allwinner,sun6i-a31-musb",
	  .data = &sun6i_a31_musb_cfg, },
	{ .compatible = "allwinner,sun8i-a33-musb",
	  .data = &sun8i_a33_musb_cfg, },
	{ .compatible = "allwinner,sun50i-h616-musb",
	  .data = &sun50i_h616_musb_cfg, },
	{ .compatible = "allwinner,sun8i-h3-musb",
	  .data = &sun8i_h3_musb_cfg, },
	{ .compatible = "allwinner,suniv-f1c100s-musb",
	  .data = &suniv_f1c100s_musb_cfg, },
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_musb_match);

static struct platform_driver sunxi_musb_driver = {
	.probe = sunxi_musb_probe,
	.remove = sunxi_musb_remove,
	.driver = {
		.name = "musb-sunxi",
		.of_match_table = sunxi_musb_match,
	},
};
module_platform_driver(sunxi_musb_driver);

MODULE_DESCRIPTION("Allwinner sunxi MUSB Glue Layer");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL v2");
