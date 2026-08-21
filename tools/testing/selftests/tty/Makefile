# SPDX-License-Identifier: GPL-2.0
CFLAGS = -O2 -Wall $(KHDR_INCLUDES)
TEST_GEN_PROGS := serial_framed_tx_test tty_tstamp_update tty_tiocsti_test
LDLIBS += -lcap

include ../lib.mk

# Add libcap for TIOCSTI test
$(OUTPUT)/tty_tiocsti_test: LDLIBS += -lcap
$(OUTPUT)/serial_framed_tx_test: LDLIBS += -lutil
