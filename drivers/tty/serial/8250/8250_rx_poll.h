/* SPDX-License-Identifier: GPL-2.0 */
#ifndef SERIAL8250_RX_POLL_H
#define SERIAL8250_RX_POLL_H

#include <linux/hrtimer.h>
#include <linux/irq_poll.h>

struct uart_8250_port;

/*
 * Opt-in continuous RX only. All callbacks run with the UART port lock held.
 * Masking notifications must not stop reception. poll() consumes at most
 * budget bytes and reports bytes still pending after that consumption.
 * arm() must enable notifications AND recheck progress: return 1 with them
 * masked if data raced with rearming, 0 if armed, or a negative error if the
 * transfer stopped. No callback may sleep or synchronously notify the poller.
 * The backend owns DMA progress, memory ordering, overrun and line errors.
 */
struct serial8250_rx_poll_ops {
	void (*mask)(struct uart_8250_port *up);
	int (*poll)(struct uart_8250_port *up, unsigned int budget,
		    unsigned int *pending);
	int (*arm)(struct uart_8250_port *up);
};

enum serial8250_rx_poll_state {
	SERIAL8250_RX_STOPPED,
	SERIAL8250_RX_ARMED,
	SERIAL8250_RX_QUEUED,
	SERIAL8250_RX_WAITING,
};

/* State and backend operations are serialized by the port lock. */
struct serial8250_rx_poll {
	struct uart_8250_port *up;
	const struct serial8250_rx_poll_ops *ops;
	struct irq_poll poll;
	struct hrtimer timer;
	ktime_t eligible;
	u64 service_started;
	u64 max_idle_ns;
	unsigned int coalesce_ns;
	unsigned int reserve_bytes;
	unsigned int service_ratio;
	bool coalescing;
	enum serial8250_rx_poll_state state;
};

/*
 * Initialize before enabling DMA callbacks. budget is bytes, not descriptors.
 * service_ratio requests a service duty cycle of 1 / service_ratio. After a
 * drain taking C nanoseconds, defer follow-ups for (service_ratio - 1) * C,
 * capped by coalesce_ns and ring headroom. An isolated arrival is serviced
 * immediately; a follow-up inside this interval starts coalescing
 * until an empty drain rearms notifications. While coalescing, the interval
 * also targets one FIFO worth of wire time, subject to those same caps, to
 * amortize dispatch and TTY costs which are not proportional to copy time.
 * Full-budget drains with another
 * full budget pending can continue immediately to make room in the ring.
 *
 * Service time includes notification dispatch through the completed drain,
 * but not subsequent TTY work. Queueing delays are conservatively included.
 * It is a pacing policy, not a CPU reservation: latency and ring headroom
 * take priority when the requested duty cycle cannot be met.
 * coalesce_ns is a latency-policy ceiling, not permission to fill the ring.
 * reserve_bytes covers the hardware FIFO and DMA bytes not yet represented
 * by pending. Each deadline uses at most half the remaining ring space at
 * port.frame_time, leaving the other half for scheduling and service delay.
 * The calculation starts before poll(), so callback time consumes headroom
 * rather than extending the deadline. This is not a guarantee against an
 * arbitrarily delayed CPU; the backend must still detect and report overruns.
 * Stop and synchronize reception before changing the ring or baud rate.
 * A backend with a wrapping progress counter may set max_idle_ns before
 * start(). This schedules an occasional idle sample to keep that counter
 * unambiguous; zero disables idle sampling. It is independent of coalescing.
 */
int serial8250_rx_poll_init(struct serial8250_rx_poll *rx,
			    struct uart_8250_port *up,
			    const struct serial8250_rx_poll_ops *ops,
			    unsigned int budget, unsigned int coalesce_ns,
			    unsigned int reserve_bytes, unsigned int service_ratio);

/*
 * The caller holds the port lock for start, notify and stop. Before restarting,
 * stop and synchronize both this helper and the previous DMA transfer.
 */
int serial8250_rx_poll_start(struct serial8250_rx_poll *rx);
void serial8250_rx_poll_notify(struct serial8250_rx_poll *rx);
void serial8250_rx_poll_stop(struct serial8250_rx_poll *rx);

/*
 * After stop(), drain outside the port lock, with start() excluded. Also
 * synchronize the backend's DMA callbacks before freeing rx or its port.
 */
void serial8250_rx_poll_sync(struct serial8250_rx_poll *rx);

#endif
