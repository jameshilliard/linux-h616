// SPDX-License-Identifier: GPL-2.0

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <pty.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <termios.h>
#include <unistd.h>

#include <linux/io_uring.h>
#include <linux/serial.h>

#include "../kselftest_harness.h"

static_assert(sizeof(struct serial_ioc_transfer) == 48, "transfer size");
static_assert(offsetof(struct serial_ioc_transfer, tx_buf) == 0, "tx_buf offset");
static_assert(offsetof(struct serial_ioc_transfer, len) == 8, "len offset");
static_assert(offsetof(struct serial_ioc_transfer, guard_ns) == 16, "guard offset");
static_assert(offsetof(struct serial_ioc_transfer, min_interval_ns) == 24,
	      "interval offset");

static_assert(sizeof(struct serial_ioc_message) == 80, "message size");
static_assert(offsetof(struct serial_ioc_message, transfers) == 8,
	      "transfers offset");
static_assert(offsetof(struct serial_ioc_message, transfer_count) == 16,
	      "transfer_count offset");
static_assert(offsetof(struct serial_ioc_message, completed_bytes) == 24,
	      "completed_bytes offset");
static_assert(offsetof(struct serial_ioc_message, timeout_ns) == 40,
	      "timeout offset");

static_assert(sizeof(struct serial_ioc_message_caps) == 72, "caps size");
static_assert(offsetof(struct serial_ioc_message_caps, max_message_bytes) == 16,
	      "max_message_bytes offset");
static_assert(offsetof(struct serial_ioc_message_caps, max_timeout_ns) == 40,
	      "max_timeout offset");
static_assert(offsetof(struct serial_ioc_message_caps, reserved) == 48,
	      "caps reserved offset");
static_assert(SERIAL_URING_CMD_WRITE_MSG == 1, "uring command value");

struct serial_uring {
	int fd;
	unsigned int setup_flags;
	void *sq_ring;
	void *cq_ring;
	size_t sq_ring_size;
	size_t cq_ring_size;
	unsigned int sq_entries;
	struct io_uring_sqe *sqes;
	unsigned int *sq_head;
	unsigned int *sq_tail;
	unsigned int *sq_mask;
	unsigned int *sq_flags;
	unsigned int *sq_array;
	unsigned int *cq_head;
	unsigned int *cq_tail;
	unsigned int *cq_mask;
	struct io_uring_cqe *cqes;
};

static void serial_uring_destroy(struct serial_uring *ring)
{
	if (ring->sqes && ring->sqes != MAP_FAILED)
		munmap(ring->sqes, ring->sq_entries * sizeof(*ring->sqes));
	if (ring->cq_ring && ring->cq_ring != MAP_FAILED &&
	    ring->cq_ring != ring->sq_ring)
		munmap(ring->cq_ring, ring->cq_ring_size);
	if (ring->sq_ring && ring->sq_ring != MAP_FAILED)
		munmap(ring->sq_ring, ring->sq_ring_size);
	if (ring->fd >= 0)
		close(ring->fd);
	memset(ring, 0, sizeof(*ring));
	ring->fd = -1;
}

static int serial_uring_init_flags(struct serial_uring *ring,
				   unsigned int flags)
{
	struct io_uring_params params = {
		.flags = flags,
		.sq_thread_idle = 1000,
	};
	void *sq_ring;
	void *cq_ring;
	int error;

	memset(ring, 0, sizeof(*ring));
	ring->fd = -1;
	ring->fd = syscall(__NR_io_uring_setup, 4, &params);
	if (ring->fd < 0)
		return -errno;

	ring->setup_flags = params.flags;
	ring->sq_ring_size = params.sq_off.array +
		params.sq_entries * sizeof(*ring->sq_array);
	ring->sq_entries = params.sq_entries;
	ring->cq_ring_size = params.cq_off.cqes +
		params.cq_entries * sizeof(*ring->cqes);
	if (params.features & IORING_FEAT_SINGLE_MMAP) {
		if (ring->cq_ring_size > ring->sq_ring_size)
			ring->sq_ring_size = ring->cq_ring_size;
		ring->cq_ring_size = ring->sq_ring_size;
	}

	sq_ring = mmap(NULL, ring->sq_ring_size, PROT_READ | PROT_WRITE,
		       MAP_SHARED, ring->fd, IORING_OFF_SQ_RING);
	if (sq_ring == MAP_FAILED)
		goto error;
	ring->sq_ring = sq_ring;

	if (params.features & IORING_FEAT_SINGLE_MMAP) {
		cq_ring = sq_ring;
	} else {
		cq_ring = mmap(NULL, ring->cq_ring_size,
			       PROT_READ | PROT_WRITE, MAP_SHARED,
			       ring->fd, IORING_OFF_CQ_RING);
		if (cq_ring == MAP_FAILED)
			goto error;
	}
	ring->cq_ring = cq_ring;

	ring->sqes = mmap(NULL, params.sq_entries * sizeof(*ring->sqes),
			  PROT_READ | PROT_WRITE, MAP_SHARED, ring->fd,
			  IORING_OFF_SQES);
	if (ring->sqes == MAP_FAILED)
		goto error;

	ring->sq_head = sq_ring + params.sq_off.head;
	ring->sq_tail = sq_ring + params.sq_off.tail;
	ring->sq_mask = sq_ring + params.sq_off.ring_mask;
	ring->sq_flags = sq_ring + params.sq_off.flags;
	ring->sq_array = sq_ring + params.sq_off.array;
	ring->cq_head = cq_ring + params.cq_off.head;
	ring->cq_tail = cq_ring + params.cq_off.tail;
	ring->cq_mask = cq_ring + params.cq_off.ring_mask;
	ring->cqes = cq_ring + params.cq_off.cqes;

	return 0;

error:
	error = errno;
	serial_uring_destroy(ring);
	return -error;
}

static int serial_uring_init(struct serial_uring *ring)
{
	return serial_uring_init_flags(ring, 0);
}

static struct io_uring_sqe *serial_uring_get_sqe(struct serial_uring *ring)
{
	unsigned int head = __atomic_load_n(ring->sq_head, __ATOMIC_ACQUIRE);
	unsigned int tail = __atomic_load_n(ring->sq_tail, __ATOMIC_RELAXED);
	unsigned int index;

	if (tail - head >= ring->sq_entries)
		return NULL;
	index = tail & *ring->sq_mask;
	memset(&ring->sqes[index], 0, sizeof(ring->sqes[index]));
	ring->sq_array[index] = index;
	__atomic_store_n(ring->sq_tail, tail + 1, __ATOMIC_RELEASE);

	return &ring->sqes[index];
}

static int serial_uring_submit(struct serial_uring *ring, unsigned int count,
			       int *results)
{
	unsigned int completed = 0;
	unsigned int enter_flags = IORING_ENTER_GETEVENTS;
	int ret;

	if (__atomic_load_n(ring->sq_flags, __ATOMIC_ACQUIRE) &
	    IORING_SQ_NEED_WAKEUP)
		enter_flags |= IORING_ENTER_SQ_WAKEUP;
	ret = syscall(__NR_io_uring_enter, ring->fd, count, count,
		      enter_flags, NULL, 0);
	if (ret < 0)
		return -errno;
	if (!(ring->setup_flags & IORING_SETUP_SQPOLL) && ret != (int)count)
		return -EIO;

	while (completed < count) {
		unsigned int head = __atomic_load_n(ring->cq_head,
						    __ATOMIC_RELAXED);
		unsigned int tail = __atomic_load_n(ring->cq_tail,
						    __ATOMIC_ACQUIRE);

		while (head != tail && completed < count) {
			struct io_uring_cqe *cqe =
				&ring->cqes[head & *ring->cq_mask];

			if (!cqe->user_data || cqe->user_data > count)
				return -EIO;
			results[cqe->user_data - 1] = cqe->res;
			completed++;
			head++;
		}
		__atomic_store_n(ring->cq_head, head, __ATOMIC_RELEASE);
		if (completed == count)
			break;
		ret = syscall(__NR_io_uring_enter, ring->fd, 0,
			      count - completed, IORING_ENTER_GETEVENTS,
			      NULL, 0);
		if (ret < 0)
			return -errno;
	}

	return 0;
}

static struct io_uring_sqe *
serial_uring_prep_message(struct serial_uring *ring, int fd,
			  struct serial_ioc_message *message,
			  unsigned int index)
{
	struct io_uring_sqe *sqe = serial_uring_get_sqe(ring);

	if (!sqe)
		return NULL;
	sqe->opcode = IORING_OP_URING_CMD;
	sqe->fd = fd;
	sqe->cmd_op = SERIAL_URING_CMD_WRITE_MSG;
	sqe->addr = (uintptr_t)message;
	sqe->user_data = index + 1;

	return sqe;
}

TEST(serial_framed_tx_layout)
{
	ASSERT_EQ(_IOC_SIZE(TIOCGSERMSGCAPS),
		  sizeof(struct serial_ioc_message_caps));
	ASSERT_EQ(_IOC_SIZE(TIOCSERWRITEMSG), sizeof(struct serial_ioc_message));
}

TEST(serial_framed_tx_rejects_pty)
{
	struct serial_ioc_message_caps caps = {};
	int master;
	int slave;

	ASSERT_EQ(openpty(&master, &slave, NULL, NULL, NULL), 0);
	ASSERT_EQ(ioctl(slave, TIOCGSERMSGCAPS, &caps), -1);
	EXPECT_EQ(errno, ENOTTY);
	ASSERT_EQ(close(slave), 0);
	ASSERT_EQ(close(master), 0);
}

TEST(serial_framed_tx_uring_rejects_pty)
{
	struct serial_ioc_message message = {};
	struct serial_uring ring;
	struct io_uring_sqe *sqe;
	int results[1];
	int ret;
	int master;
	int slave;

	ret = serial_uring_init(&ring);
	if (ret)
		SKIP(return, "io_uring setup failed: %s", strerror(-ret));
	ASSERT_EQ(openpty(&master, &slave, NULL, NULL, NULL), 0);
	sqe = serial_uring_prep_message(&ring, slave, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], -EOPNOTSUPP);
	ASSERT_EQ(close(slave), 0);
	ASSERT_EQ(close(master), 0);
	serial_uring_destroy(&ring);
}

static int serial_framed_tx_speed(speed_t *speed)
{
	static const struct {
		const char *name;
		speed_t speed;
	} speeds[] = {
		{ "9600", B9600 },
		{ "115200", B115200 },
		{ "1000000", B1000000 },
		{ "1500000", B1500000 },
		{ "2000000", B2000000 },
		{ "3000000", B3000000 },
	};
	const char *value = getenv("SERIAL_FRAMED_TX_BAUD");
	size_t i;

	if (!value)
		return 0;

	for (i = 0; i < ARRAY_SIZE(speeds); i++) {
		if (!strcmp(value, speeds[i].name)) {
			*speed = speeds[i].speed;
			return 1;
		}
	}

	return -1;
}

static int open_framed_serial(struct serial_ioc_message_caps *caps)
{
	const char *path = getenv("SERIAL_FRAMED_TX_DEVICE");
	struct termios termios;
	speed_t speed;
	int set_speed;
	int error;
	int fd;

	if (!path)
		return -ENOENT;

	fd = open(path, O_RDWR | O_CLOEXEC | O_NOCTTY);
	if (fd < 0)
		return -errno;
	if (tcgetattr(fd, &termios))
		goto error;
	termios.c_iflag &= ~(IXON | IXOFF | IXANY);
	termios.c_cflag &= ~CRTSCTS;
	set_speed = serial_framed_tx_speed(&speed);
	if (set_speed < 0) {
		errno = EINVAL;
		goto error;
	}
	if ((set_speed && (cfsetispeed(&termios, speed) ||
			   cfsetospeed(&termios, speed))) ||
	    tcsetattr(fd, TCSANOW, &termios))
		goto error;
	if (ioctl(fd, TIOCGSERMSGCAPS, caps))
		goto error;

	return fd;

error:
	error = errno;
	close(fd);
	return -error;
}

TEST(serial_framed_tx_caps)
{
	struct serial_ioc_message_caps caps = {};
	struct serial_ioc_message_caps reopened = {};
	size_t i;
	int fd = open_framed_serial(&caps);

	if (fd == -ENOENT)
		SKIP(return, "SERIAL_FRAMED_TX_DEVICE is not set");
	ASSERT_GE(fd, 0);
	EXPECT_EQ(caps.version, (uint32_t)SERIAL_MSG_ABI_VERSION);
	EXPECT_EQ(caps.flags & ~SERIAL_MSG_CAP_RS485, 0U);
	EXPECT_GT(caps.max_transfers, 0U);
	EXPECT_GT(caps.max_frame_bytes, 0U);
	EXPECT_GE(caps.max_message_bytes, caps.max_frame_bytes);
	EXPECT_GT(caps.max_guard_ns, 0ULL);
	EXPECT_GT(caps.max_interval_ns, 0ULL);
	EXPECT_GT(caps.max_timeout_ns, 0ULL);
	for (i = 0; i < ARRAY_SIZE(caps.reserved); i++)
		EXPECT_EQ(caps.reserved[i], 0ULL);
	TH_LOG("caps: %u transfers, %u-byte frames, %llu-byte messages",
	       caps.max_transfers, caps.max_frame_bytes,
	       (unsigned long long)caps.max_message_bytes);
	ASSERT_EQ(close(fd), 0);

	fd = open_framed_serial(&reopened);
	ASSERT_GE(fd, 0);
	EXPECT_EQ(reopened.max_frame_bytes, caps.max_frame_bytes);
	ASSERT_EQ(close(fd), 0);
}

TEST(serial_framed_tx_validation)
{
	struct serial_ioc_message_caps caps = {};
	uint8_t *oversized_frame;
	uint8_t frame = 0;
	struct serial_ioc_transfer oversized[2] = {
		{ .tx_buf = (uintptr_t)&frame },
		{ .tx_buf = (uintptr_t)&frame, .len = 1 },
	};
	struct serial_ioc_transfer transfer = {
		.tx_buf = (uintptr_t)&frame,
		.len = sizeof(frame),
	};
	struct serial_ioc_message message = {
		.version = SERIAL_MSG_ABI_VERSION,
		.transfers = (uintptr_t)&transfer,
		.transfer_count = 1,
	};
	int fd = open_framed_serial(&caps);

	if (fd == -ENOENT)
		SKIP(return, "SERIAL_FRAMED_TX_DEVICE is not set");
	ASSERT_GE(fd, 0);
	oversized_frame = malloc(caps.max_frame_bytes + 1);
	ASSERT_NE((uintptr_t)oversized_frame, (uintptr_t)0);

	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, NULL), -1);
	EXPECT_EQ(errno, EFAULT);

	message.version = 0;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.version = SERIAL_MSG_ABI_VERSION;

	message.flags = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.flags = 0;

	message.reserved[0] = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.reserved[0] = 0;

	message.initial_guard_ns = caps.max_guard_ns + 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.initial_guard_ns = 0;

	message.timeout_ns = caps.max_timeout_ns + 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.timeout_ns = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, ETIMEDOUT);
	EXPECT_EQ(message.completed_transfers, 0U);
	EXPECT_EQ(message.completed_bytes, 0ULL);
	message.timeout_ns = 0;

	message.transfers = 0;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.transfers = (uintptr_t)&transfer;

	message.transfer_count = 0;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	message.transfer_count = 1;

	message.transfers = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EFAULT);
	message.transfers = (uintptr_t)&transfer;

	message.transfer_count = caps.max_transfers + 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EMSGSIZE);
	message.transfer_count = 1;

	transfer.flags = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	transfer.flags = 0;

	transfer.reserved[0] = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	transfer.reserved[0] = 0;

	transfer.guard_ns = caps.max_guard_ns + 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	transfer.guard_ns = 0;

	transfer.min_interval_ns = caps.max_interval_ns + 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	transfer.min_interval_ns = 0;

	transfer.len = 0;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	transfer.tx_buf = (uintptr_t)oversized_frame;
	transfer.len = caps.max_frame_bytes + 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EMSGSIZE);
	transfer.tx_buf = (uintptr_t)&frame;
	transfer.len = sizeof(frame);

	transfer.tx_buf = 0;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EINVAL);
	transfer.tx_buf = 1;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EFAULT);
	transfer.tx_buf = (uintptr_t)&frame;

	oversized[0].len = caps.max_message_bytes;
	message.transfers = (uintptr_t)oversized;
	message.transfer_count = 2;
	ASSERT_EQ(ioctl(fd, TIOCSERWRITEMSG, &message), -1);
	EXPECT_EQ(errno, EMSGSIZE);

	free(oversized_frame);
	ASSERT_EQ(close(fd), 0);
}

TEST(serial_framed_tx_uring_validation)
{
	struct serial_ioc_message_caps caps = {};
	uint8_t frame = 0;
	struct serial_ioc_transfer transfer = {
		.tx_buf = (uintptr_t)&frame,
		.len = sizeof(frame),
	};
	struct serial_ioc_message message = {
		.version = SERIAL_MSG_ABI_VERSION,
		.transfers = (uintptr_t)&transfer,
		.transfer_count = 1,
	};
	struct serial_uring ring;
	struct io_uring_sqe *sqe;
	int results[1];
	int ret;
	int fd;

	fd = open_framed_serial(&caps);
	if (fd == -ENOENT)
		SKIP(return, "SERIAL_FRAMED_TX_DEVICE is not set");
	ASSERT_GE(fd, 0);
	ret = serial_uring_init(&ring);
	if (ret)
		SKIP(goto out_close, "io_uring setup failed: %s",
		     strerror(-ret));

	sqe = serial_uring_prep_message(&ring, fd, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->cmd_op++;
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], -EOPNOTSUPP);

	sqe = serial_uring_prep_message(&ring, fd, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->len = 1;
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], -EINVAL);

	sqe = serial_uring_prep_message(&ring, fd, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->uring_cmd_flags = IORING_URING_CMD_FIXED;
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], -EINVAL);

	sqe = serial_uring_prep_message(&ring, fd, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->buf_index = 1;
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], -EINVAL);

	sqe = serial_uring_prep_message(&ring, fd, (void *)1, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->flags = IOSQE_ASYNC;
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], -EFAULT);

	serial_uring_destroy(&ring);
out_close:
	ASSERT_EQ(close(fd), 0);
}

TEST(serial_framed_tx_uring_sqpoll_tostop)
{
	struct serial_ioc_message_caps caps = {};
	uint8_t frame = 0;
	struct serial_ioc_transfer transfer = {
		.tx_buf = (uintptr_t)&frame,
		.len = sizeof(frame),
	};
	struct serial_ioc_message message = {
		.version = SERIAL_MSG_ABI_VERSION,
		.transfers = (uintptr_t)&transfer,
		.transfer_count = 1,
	};
	struct serial_uring ring;
	struct io_uring_sqe *sqe;
	struct termios original;
	struct termios settings;
	int results[1];
	int ret;
	int fd;

	if (!getenv("SERIAL_FRAMED_TX_URING_TEST"))
		SKIP(return, "SERIAL_FRAMED_TX_URING_TEST is not set");

	fd = open_framed_serial(&caps);
	ASSERT_GE(fd, 0);
	ASSERT_EQ(tcgetattr(fd, &original), 0);
	settings = original;
	settings.c_lflag |= TOSTOP;
	ASSERT_EQ(tcsetattr(fd, TCSANOW, &settings), 0);

	ret = serial_uring_init_flags(&ring, IORING_SETUP_SQPOLL);
	if (ret)
		SKIP(goto out_restore, "SQPOLL setup failed: %s",
		     strerror(-ret));

	sqe = serial_uring_prep_message(&ring, fd, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	ASSERT_EQ(serial_uring_submit(&ring, 1, results), 0);
	EXPECT_EQ(results[0], 1);
	EXPECT_EQ(message.completed_transfers, 1U);
	EXPECT_EQ(message.completed_bytes, 1ULL);

	serial_uring_destroy(&ring);
out_restore:
	ASSERT_EQ(tcsetattr(fd, TCSANOW, &original), 0);
	ASSERT_EQ(close(fd), 0);
}

TEST(serial_framed_tx_write)
{
	static const uint8_t frame_a[] = "SERIAL-FRAME-A";
	static const uint8_t frame_b[] = "SERIAL-FRAME-B-LONGER";
	struct serial_ioc_message_caps caps = {};
	uint8_t prelude[2048];
	struct serial_ioc_transfer transfers[] = {
		{
			.tx_buf = (uintptr_t)frame_a,
			.len = sizeof(frame_a) - 1,
			.guard_ns = 1000000,
		}, {
			.tx_buf = (uintptr_t)frame_b,
			.len = sizeof(frame_b) - 1,
			.guard_ns = 1000000,
			.min_interval_ns = 2000000,
		}, {
			.guard_ns = 1000000,
		},
	};
	struct serial_ioc_message message = {
		.version = SERIAL_MSG_ABI_VERSION,
		.transfers = (uintptr_t)transfers,
		.transfer_count = ARRAY_SIZE(transfers),
		.initial_guard_ns = 1000000,
		.timeout_ns = 5000000000ULL,
	};
	uint8_t postlude[2048];
	uint8_t *large_frame;
	size_t large_frame_len;
	size_t framed_bytes;
	size_t i;
	int ret;
	int fd;

	if (!getenv("SERIAL_FRAMED_TX_WRITE_TEST"))
		SKIP(return, "SERIAL_FRAMED_TX_WRITE_TEST is not set");

	fd = open_framed_serial(&caps);
	ASSERT_GE(fd, 0);
	if (caps.max_frame_bytes < 176)
		SKIP(goto out_close, "driver frame limit is too small");

	/* Exercise DMA-sized frames without making the test architecture-sized. */
	large_frame_len = caps.max_frame_bytes < 4096 ?
			  caps.max_frame_bytes : 4096;
	large_frame = malloc(large_frame_len);
	ASSERT_NE((uintptr_t)large_frame, (uintptr_t)0);
	for (i = 0; i < large_frame_len; i++)
		large_frame[i] = i;
	transfers[2].tx_buf = (uintptr_t)large_frame;
	transfers[2].len = large_frame_len;
	framed_bytes = sizeof(frame_a) + sizeof(frame_b) - 2 +
		       large_frame_len;

	memset(prelude, 0xa5, sizeof(prelude));
	ASSERT_EQ(write(fd, prelude, sizeof(prelude)), (ssize_t)sizeof(prelude));
	ret = ioctl(fd, TIOCSERWRITEMSG, &message);
	if (ret < 0)
		TH_LOG("TIOCSERWRITEMSG failed: errno %d (%s)", errno,
		       strerror(errno));
	ASSERT_EQ(ret, (int)framed_bytes);
	EXPECT_EQ(message.completed_transfers, (uint32_t)ARRAY_SIZE(transfers));
	EXPECT_EQ(message.completed_bytes, (uint64_t)framed_bytes);
	memset(postlude, 0x5a, sizeof(postlude));
	ASSERT_EQ(write(fd, postlude, sizeof(postlude)),
		  (ssize_t)sizeof(postlude));
	ASSERT_EQ(tcdrain(fd), 0);
	free(large_frame);

out_close:
	ASSERT_EQ(close(fd), 0);
}

TEST(serial_framed_tx_uring_write)
{
	static const uint8_t frame_a[] = "SERIAL-URING-FRAME-A";
	static const uint8_t frame_b[] = "SERIAL-URING-FRAME-B";
	struct serial_ioc_message_caps caps = {};
	struct serial_ioc_transfer first_transfers[] = {
		{
			.tx_buf = (uintptr_t)frame_a,
			.len = sizeof(frame_a) - 1,
			.guard_ns = 1000000,
		}, {
			.tx_buf = (uintptr_t)frame_b,
			.len = sizeof(frame_b) - 1,
			.guard_ns = 1000000,
		},
	};
	struct serial_ioc_transfer second_transfer = {
		.guard_ns = 1000000,
	};
	struct serial_ioc_message messages[] = {
		{
			.version = SERIAL_MSG_ABI_VERSION,
			.transfers = (uintptr_t)first_transfers,
			.transfer_count = ARRAY_SIZE(first_transfers),
			.initial_guard_ns = 1000000,
			.timeout_ns = 5000000000ULL,
		}, {
			.version = SERIAL_MSG_ABI_VERSION,
			.transfers = (uintptr_t)&second_transfer,
			.transfer_count = 1,
			.initial_guard_ns = 1000000,
			.timeout_ns = 5000000000ULL,
		},
	};
	struct serial_uring ring;
	struct io_uring_sqe *sqe;
	uint8_t *large_frame;
	size_t large_frame_len;
	size_t first_bytes = sizeof(frame_a) + sizeof(frame_b) - 2;
	int results[ARRAY_SIZE(messages)];
	size_t i;
	int ret;
	int fd;

	if (!getenv("SERIAL_FRAMED_TX_URING_TEST"))
		SKIP(return, "SERIAL_FRAMED_TX_URING_TEST is not set");

	fd = open_framed_serial(&caps);
	ASSERT_GE(fd, 0);
	ret = serial_uring_init(&ring);
	if (ret)
		SKIP(goto out_close, "io_uring setup failed: %s",
		     strerror(-ret));

	large_frame_len = caps.max_frame_bytes < 4096 ?
			  caps.max_frame_bytes : 4096;
	large_frame = malloc(large_frame_len);
	ASSERT_NE((uintptr_t)large_frame, (uintptr_t)0);
	for (i = 0; i < large_frame_len; i++)
		large_frame[i] = i ^ 0x5a;
	second_transfer.tx_buf = (uintptr_t)large_frame;
	second_transfer.len = large_frame_len;

	sqe = serial_uring_prep_message(&ring, fd, &messages[0], 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	/* Exercise automatic blocking reissue from the submission context. */
	sqe->flags = IOSQE_IO_LINK;
	sqe = serial_uring_prep_message(&ring, fd, &messages[1], 1);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->flags = IOSQE_ASYNC;
	ASSERT_EQ(serial_uring_submit(&ring, ARRAY_SIZE(messages), results), 0);
	EXPECT_EQ(results[0], (int)first_bytes);
	EXPECT_EQ(results[1], (int)large_frame_len);
	EXPECT_EQ(messages[0].completed_transfers,
		  (uint32_t)ARRAY_SIZE(first_transfers));
	EXPECT_EQ(messages[0].completed_bytes, (uint64_t)first_bytes);
	EXPECT_EQ(messages[1].completed_transfers, 1U);
	EXPECT_EQ(messages[1].completed_bytes, (uint64_t)large_frame_len);
	ASSERT_EQ(tcdrain(fd), 0);

	free(large_frame);
	serial_uring_destroy(&ring);
out_close:
	ASSERT_EQ(close(fd), 0);
}

TEST(serial_framed_tx_uring_cancel)
{
	struct __kernel_timespec timeout = {
		.tv_nsec = 20000000,
	};
	struct serial_ioc_message_caps caps = {};
	uint8_t frame[] = "SERIAL-URING-CANCEL";
	struct serial_ioc_transfer transfer = {
		.tx_buf = (uintptr_t)frame,
		.len = sizeof(frame) - 1,
	};
	struct serial_ioc_message message = {
		.version = SERIAL_MSG_ABI_VERSION,
		.transfers = (uintptr_t)&transfer,
		.transfer_count = 1,
		.initial_guard_ns = 1000000000ULL,
		.timeout_ns = 5000000000ULL,
	};
	struct serial_uring ring;
	struct io_uring_sqe *sqe;
	int results[2];
	int ret;
	int fd;

	if (!getenv("SERIAL_FRAMED_TX_URING_CANCEL_TEST"))
		SKIP(return, "SERIAL_FRAMED_TX_URING_CANCEL_TEST is not set");

	fd = open_framed_serial(&caps);
	ASSERT_GE(fd, 0);
	ret = serial_uring_init(&ring);
	if (ret)
		SKIP(goto out_close, "io_uring setup failed: %s",
		     strerror(-ret));

	sqe = serial_uring_prep_message(&ring, fd, &message, 0);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->flags = IOSQE_ASYNC | IOSQE_IO_LINK;
	sqe = serial_uring_get_sqe(&ring);
	ASSERT_NE((uintptr_t)sqe, (uintptr_t)0);
	sqe->opcode = IORING_OP_LINK_TIMEOUT;
	sqe->fd = -1;
	sqe->addr = (uintptr_t)&timeout;
	sqe->len = 1;
	sqe->user_data = 2;

	ASSERT_EQ(serial_uring_submit(&ring, 2, results), 0);
	EXPECT_TRUE(results[0] == -ECANCELED || results[0] == -EINTR);
	EXPECT_EQ(results[1], -ETIME);
	EXPECT_EQ(message.completed_transfers, 0U);
	EXPECT_EQ(message.completed_bytes, 0ULL);

	serial_uring_destroy(&ring);
out_close:
	ASSERT_EQ(close(fd), 0);
}

static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xffff;
	size_t i;

	for (i = 0; i < len; i++) {
		unsigned int bit;

		crc ^= buf[i];
		for (bit = 0; bit < 8; bit++)
			crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
	}

	return crc;
}

TEST(serial_framed_tx_modbus_rtu)
{
	struct serial_ioc_message_caps caps = {};
	struct serial_rs485 original;
	struct serial_rs485 rs485 = {
		.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND,
	};
	uint8_t adu[256];
	struct serial_ioc_transfer transfer = {
		.tx_buf = (uintptr_t)adu,
		/* 5 ms exceeds t3.5 at every selectable test rate. */
		.guard_ns = 5000000,
	};
	struct serial_ioc_message message = {
		.version = SERIAL_MSG_ABI_VERSION,
		.transfers = (uintptr_t)&transfer,
		.transfer_count = 1,
		.initial_guard_ns = 5000000,
		.timeout_ns = 5000000000ULL,
	};
	uint16_t crc;
	size_t i;
	int saved_errno = 0;
	int restore_ret;
	int ret;
	int fd;

	if (!getenv("SERIAL_FRAMED_TX_MODBUS_TEST"))
		SKIP(return, "SERIAL_FRAMED_TX_MODBUS_TEST is not set");
	if (!getenv("SERIAL_FRAMED_TX_BAUD"))
		SKIP(return, "SERIAL_FRAMED_TX_BAUD is not set");

	fd = open_framed_serial(&caps);
	ASSERT_GE(fd, 0);
	if (!(caps.flags & SERIAL_MSG_CAP_RS485))
		SKIP(goto out_close, "driver does not support framed RS-485");
	if (caps.max_frame_bytes < 4)
		SKIP(goto out_close, "driver frame limit is too small");

	transfer.len = caps.max_frame_bytes < sizeof(adu) ?
		       caps.max_frame_bytes : sizeof(adu);
	/* Use a broadcast address so an attached Modbus server cannot reply. */
	adu[0] = 0;
	adu[1] = 0x7f;
	for (i = 2; i < transfer.len - 2; i++)
		adu[i] = i;
	crc = modbus_crc16(adu, transfer.len - 2);
	adu[transfer.len - 2] = crc;
	adu[transfer.len - 1] = crc >> 8;

	ASSERT_EQ(ioctl(fd, TIOCGRS485, &original), 0);
	ASSERT_EQ(ioctl(fd, TIOCSRS485, &rs485), 0);
	ret = ioctl(fd, TIOCSERWRITEMSG, &message);
	if (ret < 0)
		saved_errno = errno;
	restore_ret = ioctl(fd, TIOCSRS485, &original);

	EXPECT_EQ(restore_ret, 0);
	if (ret < 0)
		TH_LOG("RS-485 TIOCSERWRITEMSG failed: errno %d (%s)",
		       saved_errno, strerror(saved_errno));
	EXPECT_EQ(ret, (int)transfer.len);
	EXPECT_EQ(message.completed_transfers, 1U);
	EXPECT_EQ(message.completed_bytes, (uint64_t)transfer.len);

out_close:
	ASSERT_EQ(close(fd), 0);
}

TEST_HARNESS_MAIN
