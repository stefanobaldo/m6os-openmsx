# openMSX, with the FBLabs SD Mapper V2

This is a fork of [openMSX](https://github.com/openMSX/openMSX). Its `m6`
branch is openMSX 21.0 plus the FBLabs SD Mapper V2 cartridge
(`SDMapperV2`: two SD cards on an SPI port behind an ASCII16 mapper,
with the card model answering `CMD23` and offering a standard-capacity
card), written for the test harness of
[m6](https://github.com/stefanobaldo/m6os); and, as an option of that
device (`<m6block>`), the reference implementation of m6's block device
register set, through which a cartridge that is itself the machine's
memory mapper moves sectors into that RAM — m6's Turbo transfer mode —
writing into a `MemoryMapper` device of the same extension, and answers
the two FAT accelerator commands (a count of free clusters, the next
free cluster) on a FAT12, FAT16 or FAT32 table on its card.
Releases tagged `m6-21.0.N` carry a Linux x86_64 build and a macOS
Apple-silicon disk image; the binary reports its version as `21.0-m6.N`.

Everything else is openMSX as released: see `doc/` and the
[openMSX site](https://openmsx.org/).
