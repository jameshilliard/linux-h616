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
#include <termios.h>
#include <unistd.h>

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
