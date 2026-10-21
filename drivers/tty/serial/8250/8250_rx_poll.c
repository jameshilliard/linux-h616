// SPDX-License-Identifier: GPL-2.0
/* Interrupt-triggered coalescing for opt-in continuous 8250 RX DMA. */
#include "8250.h"
#include "8250_rx_poll.h"

/* irq_poll's shared fairness budget counts these units, not DMA descriptors. */
#define RX_POLL_BYTES_PER_UNIT	256

/* Call only after completing irq_poll ownership, or from an IRQ notification. */
static void serial8250_rx_poll_schedule(struct serial8250_rx_poll *rx)
{
	if (ktime_before(ktime_get(), rx->eligible)) {
		/* A close follow-up starts coalescing, without a learning period. */
		rx->coalescing = true;
		rx->state = SERIAL8250_RX_WAITING;
		hrtimer_start(&rx->timer, rx->eligible, HRTIMER_MODE_ABS_PINNED_SOFT);
	} else {
		rx->state = SERIAL8250_RX_QUEUED;
		rx->service_started = ktime_get_ns();
		irq_poll_sched(&rx->poll);
	}
}

static void serial8250_rx_poll_arm(struct serial8250_rx_poll *rx)
{
	int ret = rx->ops->arm(rx->up);

	if (ret > 0)
		serial8250_rx_poll_schedule(rx);
	else if (!ret) {
		rx->state = SERIAL8250_RX_ARMED;
		if (rx->max_idle_ns)
			hrtimer_start(&rx->timer, ktime_add_ns(ktime_get(), rx->max_idle_ns),
				      HRTIMER_MODE_ABS_PINNED_SOFT);
	} else
		rx->state = SERIAL8250_RX_STOPPED;
}

static bool serial8250_rx_poll_live(struct serial8250_rx_poll *rx)
{
	struct uart_8250_dma *dma = rx->up->dma;

	return rx->state != SERIAL8250_RX_STOPPED && dma &&
	       !dma->shutting_down && !dma->rx_terminating && dma->rx_running;
}

static ktime_t serial8250_rx_poll_deadline(struct serial8250_rx_poll *rx,
					   unsigned int pending, u64 sampled_at, u64 now)
{
	unsigned int space = rx->up->dma->rx_size - rx->reserve_bytes;
	u64 cost = now - sampled_at;
	u64 headroom_ns, delay;

	if (pending >= space)
		return ns_to_ktime(now);
	/* frame_time is rounded up; do not overestimate time available per byte. */
	headroom_ns = (u64)(space - pending) * (rx->up->port.frame_time - 1);
	headroom_ns /= 2;
	if (cost >= headroom_ns)
		return ns_to_ktime(now);
	headroom_ns -= cost;
	/*
	 * For service cost C and a requested 1/R duty cycle, leave (R - 1)C
	 * between services. Cheap, infrequent traffic therefore stays interrupt
	 * driven; the latency ceiling is not the ordinary polling interval.
	 * Bound before multiplying, including after a long scheduling delay.
	 */
	delay = min_t(u64, max_t(u64, cost, 1), rx->coalesce_ns);
	delay *= rx->service_ratio - 1;
	/*
	 * Once a close follow-up required coalescing, amortize dispatch and TTY
	 * work over a FIFO-sized batch. Copy time alone does not account for
	 * that work and would make faster copies generate more callbacks.
	 */
	if (rx->coalescing)
		delay = max_t(u64, delay, (u64)rx->up->port.fifosize *
					 (rx->up->port.frame_time - 1));
	delay = min_t(u64, delay, rx->coalesce_ns);
	return ns_to_ktime(now + min(delay, headroom_ns));
}

static int serial8250_rx_poll_run(struct irq_poll *poll, int budget)
{
	struct serial8250_rx_poll *rx = container_of(poll, typeof(*rx), poll);
	unsigned int quota = budget * RX_POLL_BYTES_PER_UNIT;
	unsigned int pending = 0;
	unsigned long flags;
	u64 now, sampled_at;
	int count, work = 0;

	uart_port_lock_irqsave(&rx->up->port, &flags);
	if (!serial8250_rx_poll_live(rx))
		goto stopped;

	/* Include dispatch overhead, not only the cheaper buffer copy. */
	sampled_at = rx->service_started;
	count = rx->ops->poll(rx->up, quota, &pending);
	if (count < 0 || !serial8250_rx_poll_live(rx))
		goto stopped;
	if (WARN_ON_ONCE(count > quota))
		goto stopped;

	now = ktime_get_ns();
	work = DIV_ROUND_UP(count, RX_POLL_BYTES_PER_UNIT);
	rx->eligible = serial8250_rx_poll_deadline(rx, pending, sampled_at, now);

	/* A few arrivals during the copy do not justify another immediate poll. */
	if (pending && ((count == quota && pending >= quota) ||
			!ktime_before(ns_to_ktime(now), rx->eligible))) {
		/* Charge a full turn when ring headroom requires immediate service. */
		work = budget;
		rx->service_started = now;
		goto unlock;
	}

	irq_poll_complete(poll);
	/* irq_poll may examine poll again if we return budget after completion. */
	work = min(work, budget - 1);
	if (!count && !pending) {
		/* End coalescing after one empty drain. */
		rx->coalescing = false;
		rx->eligible = ns_to_ktime(now);
		serial8250_rx_poll_arm(rx);
		goto unlock;
	}

	if (rx->coalescing || pending) {
		/* Keep notifications masked while small follow-up batches accumulate. */
		serial8250_rx_poll_schedule(rx);
	} else {
		/*
		 * An isolated arrival needs no timer. A notification or rearm race
		 * before eligible starts coalescing at that same deadline.
		 */
		serial8250_rx_poll_arm(rx);
	}
	goto unlock;

stopped:
	serial8250_rx_poll_stop(rx);
	irq_poll_complete(poll);
unlock:
	uart_port_unlock_irqrestore(&rx->up->port, flags);
	return work;
}

static enum hrtimer_restart serial8250_rx_poll_timer(struct hrtimer *timer)
{
	struct serial8250_rx_poll *rx = container_of(timer, typeof(*rx), timer);
	u64 started = ktime_get_ns();
	unsigned long flags;

	uart_port_lock_irqsave(&rx->up->port, &flags);
	/* An IRQ may have reprogrammed this timer while its callback waited. */
	if (ktime_before(ns_to_ktime(started), hrtimer_get_expires(timer)))
		goto unlock;
	if ((rx->state == SERIAL8250_RX_WAITING || rx->state == SERIAL8250_RX_ARMED) &&
	    serial8250_rx_poll_live(rx)) {
		if (rx->state == SERIAL8250_RX_ARMED)
			rx->ops->mask(rx->up);
		rx->state = SERIAL8250_RX_QUEUED;
		rx->service_started = started;
		irq_poll_sched(&rx->poll);
	} else if (rx->state == SERIAL8250_RX_WAITING) {
		serial8250_rx_poll_stop(rx);
	}
unlock:
	uart_port_unlock_irqrestore(&rx->up->port, flags);
	return HRTIMER_NORESTART;
}

int serial8250_rx_poll_init(struct serial8250_rx_poll *rx,
			    struct uart_8250_port *up,
			    const struct serial8250_rx_poll_ops *ops,
			    unsigned int budget, unsigned int coalesce_ns,
			    unsigned int reserve_bytes, unsigned int service_ratio)
{
	if (!ops || !ops->mask || !ops->poll || !ops->arm ||
	    budget < RX_POLL_BYTES_PER_UNIT || budget > INT_MAX / 2 ||
	    !coalesce_ns || coalesce_ns > NSEC_PER_SEC ||
	    !reserve_bytes || reserve_bytes < up->port.fifosize ||
	    service_ratio < 2 || service_ratio > 100)
		return -EINVAL;

	memset(rx, 0, sizeof(*rx));
	rx->up = up;
	rx->ops = ops;
	rx->coalesce_ns = coalesce_ns;
	rx->reserve_bytes = reserve_bytes;
	rx->service_ratio = service_ratio;
	irq_poll_init(&rx->poll, budget / RX_POLL_BYTES_PER_UNIT,
		      serial8250_rx_poll_run);
	hrtimer_setup(&rx->timer, serial8250_rx_poll_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_ABS_PINNED_SOFT);
	return 0;
}

int serial8250_rx_poll_start(struct serial8250_rx_poll *rx)
{
	lockdep_assert_held_once(&rx->up->port.lock);
	if (rx->state != SERIAL8250_RX_STOPPED)
		return -EBUSY;
	if (!rx->up->dma || rx->up->dma->rx_size <= rx->reserve_bytes ||
	    rx->up->dma->rx_size > UINT_MAX || rx->up->port.frame_time <= 1)
		return -EINVAL;
	rx->coalescing = false;
	rx->eligible = ktime_get();
	serial8250_rx_poll_arm(rx);
	return rx->state == SERIAL8250_RX_STOPPED ? -EIO : 0;
}

void serial8250_rx_poll_notify(struct serial8250_rx_poll *rx)
{
	lockdep_assert_held_once(&rx->up->port.lock);
	if (!serial8250_rx_poll_live(rx))
		return;

	if (rx->state != SERIAL8250_RX_ARMED)
		return;
	/* No synchronous cancellation while holding the callback's port lock. */
	hrtimer_try_to_cancel(&rx->timer);
	rx->ops->mask(rx->up);
	serial8250_rx_poll_schedule(rx);
}

void serial8250_rx_poll_stop(struct serial8250_rx_poll *rx)
{
	lockdep_assert_held_once(&rx->up->port.lock);
	if (rx->state == SERIAL8250_RX_STOPPED)
		return;
	rx->state = SERIAL8250_RX_STOPPED;
	rx->ops->mask(rx->up);
}

void serial8250_rx_poll_sync(struct serial8250_rx_poll *rx)
{
	unsigned long flags;

	hrtimer_cancel(&rx->timer);
	irq_poll_disable(&rx->poll);
	/* complete() drops irq_poll ownership before the callback unlocks. */
	uart_port_lock_irqsave(&rx->up->port, &flags);
	uart_port_unlock_irqrestore(&rx->up->port, flags);
	irq_poll_enable(&rx->poll);
}
