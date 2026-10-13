# openMSX, with the FBLabs SD Mapper V2

This is a fork of [openMSX](https://github.com/openMSX/openMSX). Its `m6`
branch is openMSX 21.0 plus the FBLabs SD Mapper V2 cartridge
(`SDMapperV2`: two SD cards on an SPI port behind an ASCII16 mapper,
with the card model answering `CMD23` and offering a standard-capacity
card), written for the test harness of
[m6](https://github.com/stefanobaldo/m6os).
Releases tagged `m6-21.0.N` carry a Linux x86_64 build and a macOS
Apple-silicon disk image; the binary reports its version as `21.0-m6.N`.

Everything else is openMSX as released: see `doc/` and the
[openMSX site](https://openmsx.org/).
