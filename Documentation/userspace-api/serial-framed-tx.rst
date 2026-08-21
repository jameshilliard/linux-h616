.. SPDX-License-Identifier: GPL-2.0

===============================
Framed serial transmit messages
===============================

The framed transmit interface sends an ordered array of opaque byte frames on
a UART.  It is intended for devices whose protocol uses an idle period as a
frame delimiter.  Ordinary ``write(2)`` boundaries do not have this meaning:
the TTY and UART drivers may split or combine writes, and successful return
only means that the bytes were accepted for transmission.

The ioctl is synchronous; the equivalent io_uring command completes through a
CQE.  Both submission paths execute the same serialized, transmit-only
operation.  They do not add a checksum, escape bytes, change the receive path,
or define a protocol.  One transfer is one complete frame supplied by
userspace.

Availability
============

Include ``<sys/ioctl.h>`` and ``<linux/serial.h>``, then query a serial file
descriptor with ``TIOCGSERMSGCAPS``.  A successful query returns
``struct serial_ioc_message_caps`` with ``version`` set to
``SERIAL_MSG_ABI_VERSION`` and the limits supported by the port, including
the maximum explicit whole-message timeout.

``max_frame_bytes`` is the largest frame that the driver can make available
to its transmitter without a scheduler-dependent refill.  It can depend on
the active hardware path, such as the FIFO depth for PIO or the size accepted
by a DMA submission.  The operation is not supported when this value is zero.

``flags`` describes optional driver capabilities.  A port which sets
``SERIAL_MSG_CAP_RS485`` can execute framed messages while Linux RS-485 mode
is enabled.  Capability flags describe the driver; message and transfer flags
remain zero in version 1.

A non-serial TTY returns ``ENOTTY``.  A UART driver which does not implement
the required wire guarantees returns ``EOPNOTSUPP``.

Submitting a message
====================

``TIOCSERWRITEMSG`` takes a pointer to ``struct serial_ioc_message``.  Set
``version`` to ``SERIAL_MSG_ABI_VERSION`` and ``transfers`` to an array of
``struct serial_ioc_transfer``.  Version 1 defines no flags; all flag and
reserved fields must be zero.

For every transfer:

``tx_buf``
    Points to the complete frame.  ``len`` must be nonzero and no greater
    than ``max_frame_bytes``.

``guard_ns``
    Is the minimum idle time after the transmitter becomes physically empty.

``min_interval_ns``
    Is the minimum time from the start of this frame to its completed timing
    boundary.  It can express a device processing delay independently of the
    physical idle guard.

The kernel copies and validates the message header, every descriptor, and all
frame data before it changes the UART state.  Userspace can therefore reuse or
free the input buffers once the ioctl returns, and a later userspace fault
cannot transmit an unexpected prefix.

On success, the ioctl returns the total number of transmitted bytes.  This is
a positive value and is limited to ``INT_MAX``.  ``completed_transfers`` and
``completed_bytes`` are also updated in the message header.

io_uring submission
===================

The same operation is available through ``IORING_OP_URING_CMD`` from
``<linux/io_uring.h>``.  Prepare an SQE with the serial file descriptor, set
``cmd_op`` to ``SERIAL_URING_CMD_WRITE_MSG``, and place the userspace pointer to
``struct serial_ioc_message`` in ``addr``.  ``ioprio``, ``len``,
``uring_cmd_flags``, ``buf_index``, ``file_index``, ``addr3``, and ``__pad2``
must be zero.  In particular, fixed-buffer and multishot uring commands are
not supported by version 1.

The CQE ``res`` has the same meaning as the ioctl return value, and the kernel
copies progress to the pointed-to message header before posting the CQE.  The
message header, descriptors, and frame buffers must remain valid and unchanged
until that CQE is observed.  They are copied before the command changes UART
state, so they can be reused immediately after completion.

Each uring command contains one complete message and is serialized with ioctl
submissions and ordinary writes.  Independently submitted commands have no
additional ordering guarantee beyond that serialization.  Use one message or
``IOSQE_IO_LINK`` when the order between batches matters.

A command which has not started UART execution can be cancelled by the normal
io_uring cancellation mechanisms and reports ``ECANCELED``.  Once it starts,
cancellation is best effort: an interruptible wait is stopped, but a frame
which has started is always completed through its safe boundary.  The command
can therefore complete normally if cancellation races with its final frame,
or report ``EINTR`` after a completed prefix.  It never reports cancellation
by cutting a frame short.  This also applies when ``IOSQE_ASYNC`` forces the
initial command issue through io-wq.

TTY job control is checked in the submitting task before the asynchronous
UART work starts.  The command is handed to that task before this check even
when an ``IORING_SETUP_SQPOLL`` ring performed the initial issue.

Wire timing
===========

For transfer *i*, define:

``start_i``
    The monotonic time immediately before its first byte is made available to
    the transmitter.

``temt_i``
    The first observed time after the final byte has left both the hardware
    FIFO and shift register.

The next frame can start, or the operation can complete after its last frame, no
earlier than::

    max(temt_i + guard_ns_i, start_i + min_interval_ns_i)

Before the first frame, the kernel drains ordinary output through physical
transmitter-empty and then waits ``initial_guard_ns``.  Once the port is
claimed, normal writes and priority characters are deferred.  They resume
only after the framed message reaches its final safe boundary.  Output queued
while the initial drain is in progress is either included in that drain or,
if it loses the port-claim race, deferred until the message completes.
An output-stop request made after the port is claimed likewise takes effect
after the complete message; it cannot truncate a frame or split the atomic
operation.

RS-485 and Modbus RTU
=====================

When both ``SERIAL_MSG_CAP_RS485`` and ``SER_RS485_ENABLED`` are set, one
transfer is also one RS-485 transmit burst.  The driver asserts the configured
send direction, waits ``delay_rts_before_send``, commits the complete frame,
waits for physical transmitter-empty, waits ``delay_rts_after_send``, and
releases the send direction.  The next frame cannot start and the operation
cannot complete until both the requested timing boundary and that
direction-release sequence have completed.  The RS-485 delays are included in
a kernel-computed message timeout.

``guard_ns`` remains relative to physical transmitter-empty, not to direction
release.  Consequently, the post-send delay and guard overlap; the effective
boundary is no earlier than both.  A pending post-send operation from ordinary
output is likewise completed before a framed transfer asserts send direction.
``SER_RS485_RX_DURING_TX`` and the configured RTS polarity retain their normal
meaning.  RS-485 address-bit mode is not supported by version 1.

This maps to the transmit side of Modbus RTU.  Set
``initial_guard_ns`` and every transfer's ``guard_ns`` to at least the Modbus
``t3.5`` silent interval.  At baud rates up to 19,200, compute it from the
actual termios character width, including start, parity and stop bits::

    t3.5_ns = ceil(3.5 * bits_per_character * 1,000,000,000 / baud)

The Modbus Serial Line guide recommends a fixed 1.750 ms ``t3.5`` above
19,200 baud.  A Modbus RTU ADU is at most 256 bytes, so a port advertising a
256-byte frame limit can transmit every valid ADU in one operation.  The
contiguous-frame guarantee also prevents a scheduler-dependent idle gap from
splitting an ADU on the wire.

``initial_guard_ns`` is measured after local transmitter-empty; it does not
observe receive traffic or prove that a multidrop bus has otherwise been
silent.  Userspace must account for the end of the preceding received frame
before submitting a new request when that distinction matters.

The interface remains protocol-neutral and transmit-only.  Userspace still
constructs the server address, function data and CRC, detects receive-frame
boundaries, and implements response timeouts and retries.  A request/response
client will normally submit one request frame and then read its response;
arrays are most useful for broadcasts or other transmit-only sequences.

Interruption, timeout, and progress
===================================

The initial drain and guard are interruptible.  If no frame has started, a
signal can produce a restartable ioctl error; io_uring reports ``EINTR`` in its
CQE.  Once a frame starts, the kernel waits uninterruptibly until that frame
reaches physical transmitter-empty and both of its timing requirements have
elapsed.  A pending signal is then reported as ``EINTR`` before another frame
is started.

``timeout_ns`` is a whole-message deadline.  Zero selects a kernel-computed
deadline based on the configured character time, requested delays, and a
scheduling allowance.  The deadline starts when UART execution begins, after
the request has been copied and serialized with other writers.  The kernel
does not start a frame which cannot be expected to reach its safe boundary by
the deadline.  A frame which has already started is nevertheless taken to a
safe boundary before completion, so the request can finish slightly after the
requested deadline.  If that safe boundary itself is later than the deadline,
the completed frame is included in the progress fields and the request reports
``ETIMEDOUT``.  A task which is scheduled after an on-time boundary does not
retroactively time out.

``completed_transfers`` counts only frames which reached physical empty and
both timing requirements.  ``completed_bytes`` is the corresponding byte
count.  The kernel makes a best-effort copy of these fields on every error
after input validation.  A frame after the reported prefix might have started
but failed before a safe boundary, so userspace must not retry the complete
message blindly.

If hardware does not report transmitter-empty by the earlier of the
whole-message deadline and one second after the calculated wire completion
time, the kernel leaves ordinary transmission blocked rather than risk
appending data to an incomplete frame.  Close and reopen the port to recover
it.  In software-controlled RS-485 mode this also leaves send direction
asserted because no safe release point was observed.  A result-copy fault is
reported as ``EFAULT`` even if data was already transmitted; such a request
also must not be retried without protocol-level recovery.

Limits and unsupported modes
============================

Version 1 has these global limits in addition to the queried driver limit:

* 1,024 transfers;
* 1 MiB of copied frame data;
* one second for each guard or minimum interval; and
* 60 seconds for an explicit whole-message timeout.

Both submission paths currently require the ``N_TTY`` line discipline.  They
are rejected on a registered console and while ISO 7816, software flow
control, RTS/CTS flow control, unsupported RS-485, or RS-485 address-bit mode
is enabled.  They are also rejected if output is stopped, the port is
suspended or unavailable, another framed message owns the port, or the
low-level driver cannot provide the contiguous-frame operation.

The message bypasses output post-processing: bytes in ``tx_buf`` are the bytes
presented to the UART.  A configured transmit DMA channel does not by itself
disable the interface.  The 8250 implementation drains any in-flight ordinary
DMA transfer before claiming the port.  It uses one DMA submission backed by a
dedicated buffer for each complete frame when framed DMA storage was acquired;
otherwise frames which fit in the transmit FIFO use PIO.  The dedicated buffer
is not the ordinary TTY transmit ring, which writers may continue filling while
the framed message owns the hardware.  Ordinary TX DMA resumes afterwards,
while receive configuration and receive DMA remain active.
