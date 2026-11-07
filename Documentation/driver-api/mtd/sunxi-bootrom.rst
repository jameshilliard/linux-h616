.. SPDX-License-Identifier: GPL-2.0

=========================
Sunxi BootROM NAND views
=========================

The H616 BootROM uses a different page format from normal NAND storage.
The ``allwinner,boot-format`` boolean marks entries in the fixed partition
map that use this encoding. The sunxi driver exposes those entries as MTD
views with 1024-byte payload pages protected by BCH-80 and the BootROM
randomizer stream. The supported configuration is a single-target, 8-bit
NAND with 2048-byte physical pages.

Unmarked entries retain the native NAND geometry. If the map contains a
boot-format entry, the driver uses that map rather than command-line
partitions, whose syntax cannot describe the different page formats.
Disabled entries are not registered. Without boot-format entries, ordinary
partition discovery is unchanged.

Geometry and addressing
=======================

Device-tree offsets and lengths count physical main-area bytes. They must
be non-empty, eraseblock-aligned, within the native device and mutually
disjoint. MTD offsets instead count decoded payload bytes relative to the
start of the selected view. One logical page always consumes one physical
page; the mapping does not relocate pages or skip eraseblocks.

For example, with 2048+128-byte physical pages and 128-KiB eraseblocks:

.. list-table:: Native and BootROM geometry
   :header-rows: 1

   * - Quantity
     - Native MTD
     - BootROM view
   * - Page data
     - 2048 bytes
     - 1024 bytes
   * - OOB
     - 128 bytes
     - 1152 bytes
   * - Eraseblock data
     - 128 KiB
     - 64 KiB
   * - Capacity for a 256-KiB physical region
     - 256 KiB
     - 128 KiB

For a physical region beginning at ``R``, logical byte offset ``L`` selects
physical page ``R / 2048 + L / 1024`` and payload column ``L % 1024``.
An erase at logical eraseblock index ``N`` erases physical eraseblock
``R / native_erasesize + N``. Raw mode does not change this geometry.

Decoded and raw access
======================

Normal reads return decoded payload and report ECC status. Normal writes
accept page-aligned, whole 1024-byte payload pages and generate the protected
metadata, BCH parity and randomizer encoding. Normal-mode OOB requests are
not supported. The driver treats the payload as bytes; the image producer
is responsible for its boot-image header, length, checksum and padding.

Raw requests bypass ECC and randomization. The raw data buffer corresponds
to physical bytes 0 through 1023. The raw OOB buffer continues at physical
byte 1024 through the end of the physical main area and physical OOB.
Concatenating both buffers for one page therefore recovers its complete
physical record, including an existing pre-encoded image. The virtual OOB
is not a free-metadata area: it includes BootROM metadata and parity.

Use combined data/OOB requests to read or program complete raw records.
Writing decoded payload through raw mode does not generate a bootable
codeword. Conversely, writing an already encoded record through normal
mode encodes it again. Erase before programming; the view does not make
arbitrary in-place rewrites safe.

Even an all-0xff payload page needs its encoded metadata and parity if the
BootROM will read it. Userspace must not skip such a page merely because
its decoded payload looks erased. Image padding must remain consistent
with the image format's length and checksum.

Bad blocks and physical ranges
==============================

Ordinary bad-block-marker locations can contain BootROM payload or parity.
The view therefore does not interpret them as native-format markers:
``MEMGETBADBLOCK`` returns zero while the view is connected, and marking a
block bad through the view is not supported. This is not a guarantee that
the underlying flash is healthy. Program and erase failures remain errors;
the view performs no bad-block remapping or image-copy placement policy.

The driver rejects overlapping boot-format and native partitions in the
fixed map. A map containing only boot-format entries does not register a
whole-chip native device as a fallback. With mixed formats,
``CONFIG_MTD_PARTITIONED_MASTER`` can still expose the native master, which
aliases the boot ranges. Do not access those ranges through the native
master or create overlapping runtime partitions. Boot regions must remain
reserved from UBI, filesystems and other normal-format users.

Device removal
==============

Requests share the native NAND chip and controller serialization. Controller
removal drains and disconnects the views before releasing the hardware.
Existing handles retain their metadata but further I/O fails with
``ENODEV``. A subsequent controller binding creates new views; it does not
reattach old handles.
