#include "SDMapperV2.hh"

#include "SdCard.hh"
#include "DeviceConfig.hh"
#include "MSXException.hh"
#include "MSXMemoryMapperBase.hh"
#include "MSXMotherBoard.hh"
#include "MSXCliComm.hh"
#include "DiskImageUtils.hh"
#include "endian.hh"
#include "serialize.hh"
#include "serialize_stl.hh"

#include "narrow.hh"
#include "outer.hh"

/*
FBLabs SD Mapper V2 (CPLD logic by Fabio Belavenuto, 2014-2020, CERN OHL
v1.1; the register map is documented in the cartridge's Nextor driver).

ASCII16 mapper over a 256K flash, of which the half selected by a switch on
the cartridge is in use (this device takes a 128K ROM, or a 256K one and
uses its lower half):
  6000h-67FFh (write)  bank of 4000h-7FFFh, bits 2-0
  7000h-77FFh (write)  bank of 8000h-BFFFh, bits 3-0; the flash shows in
                       8000h-BFFFh only when bit 3 is set (the updater's
                       window), and reads the bank in bits 2-0
Registers (every bank):
  7FF0h (write)  bit 0: chip select of SD card 1; bit 1: of SD card 2
  7FF0h (read)   with no card selected: bits 1-0 the two switches (here 0:
                 no RAM, driver half 0); with a card selected: bit 2 write
                 protected, bit 1 0 = card present, bit 0 card changed
                 since the last read (the read clears it). Here a slot
                 with no <sdcardN> is empty, and reads absent and changed
                 as the CPLD holds an empty slot; a declared card is
                 present and never changes, since openMSX cannot swap a
                 hard disk image while the machine runs
  7FF1h (r/w)    an 8-bit timer: the value written is decremented to 0 at
                 97.65625 kHz (one count every 10.24 us)
  7FF2h-7FFFh    nothing (the bus floats; FFh here)
SPI window (bank 7 only):
  7B00h-7EFFh    a write sends the byte and a read sends FFh; the byte
                 the card shifts out meanwhile is the read's value. The
                 CPLD holds the CPU with WAIT for the eight clocks; that
                 time is not modelled.

The m6 block device (optional, <m6block>; bank 7 only), 7F00h-7F3Fh — the
reference implementation of the register set m6 publishes for a
cartridge that moves sectors itself into the memory mapper it serves
(m6's docs/block-device.md has the semantics; this is its summary):
  7F00h-7F03h r  "M6BD"
  7F04h       r  version, 1
  7F05h       r  capabilities: bit 0 Turbo offered, bit 1 the FAT
                 accelerator's commands answer (always), bit 2 reserved,
                 bits 5-4 the subslot of this cartridge's slot where the
                 mapper is, bit 7 the device serves a mapper there; with
                 <turbo>false</turbo> only bit 1 is set
  7F06h       r  devices, 2 (the two card slots)
  7F07h       w  the device of the next command, 1 or 2
  7F08h-7F0Bh w  the first sector, little-endian
  7F0Ch       w  sectors, 1-32
  7F0Dh       w  the target segment of the mapper
  7F0Eh       w  the target's 256-byte slot in the segment, 0-62
  7F0Fh       w  the probe byte
  7F10h       w  the command: 1 read (card to RAM), 2 write (RAM to card),
                 10h count the free entries of a FAT table, 11h the first
                 free entry; the write starts it
  7F11h       r  80h while the command runs; 00h done; 01h-7Fh an error:
                 1 the probe byte is not at the target (nothing moved),
                 2 out of range, 3 the medium failed, 4 no medium, 5 the
                 card is write-protected, 6 a bad parameter, 7 an unknown
                 command
  7F12h-7F15h r  a 32-bit result: the count (10h) or the cluster, 0 for
                 none (11h), valid once the status is 00h
  7F16h-7F3Fh w  the FAT commands' parameters: +0 (4) the table's first
                 sector on the device, +4 (4) its sectors, +8 (1) the
                 type 12, 16 or 32, +9 (4) the data clusters N (the
                 entries are 2..N+1), +13 (4) the starting cluster X,
                 +17 (2) the span: table sectors to scan from the one
                 holding X's entry, 0 = to the table's end; 0 otherwise
The target is segment * 4000h + slot * 100h of the RAM of the MemoryMapper
device <mapper> names. Before anything moves the byte there is compared
with the probe byte. A command completes <turbo_us> microseconds a sector
after it started (default 50): until then the status reads 80h and, on a
read, the target still holds its old bytes; at completion a read's data is
in the RAM and a write's is on the card. Without <mapper> the block offers
Turbo with no RAM behind it and every command fails the probe — a device
that lies, for the kernel ROM's boot to refuse. <turbo_subslot> (default
3) is what the capabilities report. <turbo>false</turbo> (default true)
makes a device that offers the FAT accelerator and not Turbo: the
capabilities read bit 1 alone and a read or write command fails with 7.
The FAT commands read the span's sectors from the card and count, or
look for, the entries that are 0 — 12, 16 or 32 bits wide by the type, a
FAT12 entry straddling two sectors read whole, the top four bits of a
FAT32 entry ignored — from X through the last entry inside the span or
N+1, whichever comes first; they keep nothing between commands, and
complete <turbo_us> microseconds per sector read after they started.
Errors: 6 for a type not 12, 16 or 32, N of 0, or X outside 2..N+1; 2 for
a sector of the span past the device; 3 and 4 as for a read.
*/

namespace openmsx {

SDMapperV2::SDMapperV2(DeviceConfig& config)
	: MSXDevice(config)
	, romBlockDebug(*this)
	, blockDebug(*this)
	, rom(getName() + " ROM", "rom", config)
{
	if (rom.size() != 0x20000 && rom.size() != 0x40000) {
		throw MSXException("SD Mapper V2 ROM must be 128kB or 256kB.");
	}
	for (auto i : {0, 1}) {
		auto name = i == 0 ? "sdcard1" : "sdcard2";
		auto* child = config.findChild(name);
		cardDeclared[i] = child != nullptr;
		sdCard[i] = std::make_unique<SdCard>(DeviceConfig(config, child));
	}
	if (auto* block = config.findChild("m6block")) {
		DeviceConfig bc(config, block);
		blockPresent = true;
		mapperName = std::string(bc.getChildData("mapper", ""));
		int sub = bc.getChildDataAsInt("turbo_subslot", 3);
		if ((sub < 0) || (sub > 3)) {
			throw MSXException("m6block: turbo_subslot must be 0-3, got ", sub);
		}
		blockSubslot = narrow<uint8_t>(sub);
		int us = bc.getChildDataAsInt("turbo_us", 50);
		if (us < 0) throw MSXException("m6block: turbo_us must not be negative");
		turboUs = unsigned(us);
		turboOffered = bc.getChildDataAsBool("turbo", true);
	}
	reset(getCurrentTime());
}

SDMapperV2::~SDMapperV2() = default;

void SDMapperV2::reset(EmuTime time)
{
	bank1 = 0;
	bank2 = 0;
	cardSelect = 0;
	timerValue = 0;
	timerStart = time;
	bdReg.fill(0);
	bdStatus = 0;
	bdPending = 0;
	bdResult = 0;
	bdPendingResult = 0;
	bdBusy = false;
	bdDone = time;
	bdBuffer.clear();
	invalidateDeviceRCache(0x4000, 0x8000);
}

// The m6 block device.

static constexpr uint8_t BDC_TURBO  = 0x01;
static constexpr uint8_t BDC_FATACC = 0x02;
static constexpr uint8_t BDC_MAPPER = 0x80;
static constexpr uint8_t BDO_READ   = 1;
static constexpr uint8_t BDO_WRITE  = 2;
static constexpr uint8_t BDO_FATCOUNT = 0x10;
static constexpr uint8_t BDO_FATNEXT  = 0x11;
static constexpr uint8_t BDE_PROBE    = 1;
static constexpr uint8_t BDE_RANGE    = 2;
static constexpr uint8_t BDE_IO       = 3;
static constexpr uint8_t BDE_NOMEDIUM = 4;
static constexpr uint8_t BDE_RO       = 5;
static constexpr uint8_t BDE_PARAM    = 6;
static constexpr uint8_t BDE_CMD      = 7;
static constexpr uint8_t BD_BUSY      = 0x80;

MSXMemoryMapperBase* SDMapperV2::findMapper()
{
	if (!mapperLooked) {
		mapperLooked = true;
		if (!mapperName.empty()) {
			mapper = dynamic_cast<MSXMemoryMapperBase*>(
				getMotherBoard().findDevice(mapperName));
			if (!mapper) {
				getCliComm().printWarning(
					"SD Mapper V2 m6block: no memory mapper named \"",
					mapperName, "\": every Turbo command will fail its probe");
			}
		}
	}
	return mapper;
}

// A read of the status brings the running command to its end once its
// time has come: a read's data into the mapper's RAM, the result in.
void SDMapperV2::settleCommand(EmuTime time)
{
	if (!bdBusy || (time < bdDone)) return;
	bdBusy = false;
	if ((bdPending == 0) && !bdBuffer.empty()) {
		if (auto* m = findMapper()) {
			m->writeRamBlock(bdTarget, bdBuffer);
		}
	}
	bdBuffer.clear();
	bdResult = bdPendingResult;
	bdStatus = bdPending;
}

uint8_t SDMapperV2::capabilities() const
{
	uint8_t caps = BDC_FATACC;
	if (turboOffered) caps |= uint8_t(BDC_TURBO | BDC_MAPPER | (blockSubslot << 4));
	return caps;
}

// The FAT accelerator's two commands: the parameters decoded, the span's
// sectors read from the card, the entries counted or searched.
void SDMapperV2::fatCommand(uint8_t cmd, EmuTime time)
{
	auto fail = [&](uint8_t code) {
		bdBusy = false;
		bdBuffer.clear();
		bdStatus = code;
	};
	unsigned dev = bdReg[0x07];
	if ((dev < 1) || (dev > 2)) { fail(BDE_PARAM); return; }
	auto& card = *sdCard[dev - 1];
	if (!cardDeclared[dev - 1] || !card.hasMedium()) { fail(BDE_NOMEDIUM); return; }
	uint32_t fatSec = Endian::read_UA_L32(&bdReg[0x16]);
	uint32_t fatLen = Endian::read_UA_L32(&bdReg[0x1A]);
	uint8_t  type   = bdReg[0x1E];
	uint32_t nclus  = Endian::read_UA_L32(&bdReg[0x1F]);
	uint32_t start  = Endian::read_UA_L32(&bdReg[0x23]);
	uint32_t span   = Endian::read_UA_L16(&bdReg[0x27]);
	if ((type != 12) && (type != 16) && (type != 32)) { fail(BDE_PARAM); return; }
	if ((nclus == 0) || (start < 2) || (start > nclus + 1)) { fail(BDE_PARAM); return; }
	auto entryOffset = [&](uint64_t c) -> uint64_t {
		switch (type) {
		case 12: return c + (c >> 1);
		case 16: return c * 2;
		default: return c * 4;
		}
	};
	uint64_t firstSector = entryOffset(start) / 512;
	uint64_t endSector = fatLen; // exclusive
	if ((span != 0) && (firstSector + span < endSector)) endSector = firstSector + span;
	// The sectors read, two at a time at most: the one holding the
	// entry and, for a FAT12 entry across a boundary, the next.
	std::array<SectorBuffer, 2> buf;
	std::array<int64_t, 2> have = {-1, -1};
	unsigned sectorsRead = 0;
	auto readSector = [&](uint64_t idx, int slot) -> bool {
		if (have[slot] == int64_t(idx)) return true;
		uint64_t lba = uint64_t(fatSec) + idx;
		if (lba >= card.getNbSectors()) return false;
		card.readSector(size_t(lba), buf[slot]);
		have[slot] = int64_t(idx);
		++sectorsRead;
		return true;
	};
	uint32_t count = 0;
	uint32_t found = 0;
	try {
		for (uint64_t c = start; c <= uint64_t(nclus) + 1; ++c) {
			uint64_t off = entryOffset(c);
			uint64_t idx = off / 512;
			if (idx >= endSector) break;
			if (!readSector(idx, 0)) { fail(BDE_RANGE); return; }
			unsigned o = unsigned(off % 512);
			uint32_t entry;
			if (type == 12) {
				unsigned lo = buf[0].raw[o];
				unsigned hi;
				if (o + 1 < 512) {
					hi = buf[0].raw[o + 1];
				} else {
					if (idx + 1 >= fatLen) break; // the entry is past the table
					if (!readSector(idx + 1, 1)) { fail(BDE_RANGE); return; }
					hi = buf[1].raw[0];
				}
				unsigned w = lo | (hi << 8);
				entry = (c & 1) ? (w >> 4) : (w & 0xFFF);
			} else if (type == 16) {
				entry = buf[0].raw[o] | (buf[0].raw[o + 1] << 8);
			} else {
				entry = Endian::read_UA_L32(&buf[0].raw[o]) & 0x0FFFFFFF;
			}
			if (entry == 0) {
				if (cmd == BDO_FATNEXT) { found = narrow<uint32_t>(c); break; }
				++count;
			}
		}
	} catch (MSXException&) {
		fail(BDE_IO);
		return;
	}
	bdPending = 0;
	bdPendingResult = (cmd == BDO_FATNEXT) ? found : count;
	bdBuffer.clear();
	bdBusy = true;
	bdStatus = BD_BUSY;
	bdDone = time + EmuDuration::usec(turboUs * sectorsRead);
	++fatCommands;
}

void SDMapperV2::startCommand(EmuTime time)
{
	uint8_t cmd = bdReg[0x10];
	auto fail = [&](uint8_t code) {
		bdBusy = false;
		bdBuffer.clear();
		bdStatus = code;
	};
	if ((cmd == BDO_FATCOUNT) || (cmd == BDO_FATNEXT)) { fatCommand(cmd, time); return; }
	if ((cmd != BDO_READ) && (cmd != BDO_WRITE)) { fail(BDE_CMD); return; }
	if (!turboOffered) { fail(BDE_CMD); return; }
	unsigned dev = bdReg[0x07];
	unsigned count = bdReg[0x0C];
	unsigned slot = bdReg[0x0E];
	if ((dev < 1) || (dev > 2) || (count < 1) || (count > 32) ||
	    (slot + 2 * count > 64)) { fail(BDE_PARAM); return; }
	auto& card = *sdCard[dev - 1];
	if (!cardDeclared[dev - 1] || !card.hasMedium()) { fail(BDE_NOMEDIUM); return; }
	uint32_t lba = Endian::read_UA_L32(&bdReg[0x08]);
	if (uint64_t(lba) + count > card.getNbSectors()) { fail(BDE_RANGE); return; }
	if ((cmd == BDO_WRITE) && card.isWriteProtected()) { fail(BDE_RO); return; }
	auto* m = findMapper();
	size_t target = size_t(bdReg[0x0D]) * 0x4000 + size_t(slot) * 0x100;
	size_t len = size_t(count) * 512;
	if (!m || (target + len > m->getRamBytes())) { fail(BDE_PROBE); return; }
	if (m->peekRam(target) != bdReg[0x0F]) { fail(BDE_PROBE); return; }

	bdPending = 0;
	bdPendingResult = 0;
	bdBuffer.clear();
	bdTarget = narrow<uint32_t>(target);
	try {
		if (cmd == BDO_READ) {
			bdBuffer.resize(len);
			SectorBuffer buf;
			for (unsigned i = 0; i < count; ++i) {
				card.readSector(lba + i, buf);
				std::copy(buf.raw.begin(), buf.raw.end(), bdBuffer.begin() + i * 512);
			}
		} else {
			SectorBuffer buf;
			for (unsigned i = 0; i < count; ++i) {
				for (unsigned j = 0; j < 512; ++j) {
					buf.raw[j] = m->peekRam(target + i * 512 + j);
				}
				card.writeSector(lba + i, buf);
			}
		}
	} catch (MSXException&) {
		bdPending = BDE_IO;
		bdBuffer.clear();
	}
	bdBusy = true;
	bdStatus = BD_BUSY;
	bdDone = time + EmuDuration::usec(turboUs * count);
	++turboCommands;
}

uint8_t SDMapperV2::readBlock(uint16_t address, EmuTime time)
{
	unsigned r = address & 0x3F;
	if (r == 0x11) {
		settleCommand(time);
		return bdBusy ? BD_BUSY : bdStatus;
	}
	return peekBlock(address, time);
}

uint8_t SDMapperV2::peekBlock(uint16_t address, EmuTime time) const
{
	unsigned r = address & 0x3F;
	switch (r) {
	case 0x00: return 'M';
	case 0x01: return '6';
	case 0x02: return 'B';
	case 0x03: return 'D';
	case 0x04: return 1;
	case 0x05: return capabilities();
	case 0x06: return 2;
	case 0x11: return (bdBusy && (time < bdDone)) ? BD_BUSY : (bdBusy ? bdPending : bdStatus);
	case 0x12: return narrow_cast<uint8_t>(bdResult >>  0);
	case 0x13: return narrow_cast<uint8_t>(bdResult >>  8);
	case 0x14: return narrow_cast<uint8_t>(bdResult >> 16);
	case 0x15: return narrow_cast<uint8_t>(bdResult >> 24);
	default:   return bdReg[r];
	}
}

void SDMapperV2::writeBlock(uint16_t address, uint8_t value, EmuTime time)
{
	unsigned r = address & 0x3F;
	if ((r < 0x07) || ((0x11 <= r) && (r <= 0x15))) return; // read-only
	bdReg[r] = value;
	if (r == 0x10) startCommand(time);
}

uint8_t SDMapperV2::spiTransfer(uint8_t value)
{
	uint8_t result = 0xFF;
	for (auto i : {0, 1}) {
		bool selected = (cardSelect & (1 << i)) != 0;
		uint8_t r = sdCard[i]->transfer(value, !selected);
		if (selected) result &= r;
	}
	return result;
}

uint8_t SDMapperV2::readStatus() const
{
	switch (cardSelect) {
	case 0:
		return 0x00; // the switches: no RAM, driver half 0
	case 1:
	case 2: {
		int i = cardSelect - 1;
		if (!cardDeclared[i]) return 0x03; // absent, and the change latch held set
		return sdCard[i]->isWriteProtected() ? 0x04 : 0x00;
	}
	default:
		return 0xFF; // both selected: undefined on the CPLD
	}
}

uint8_t SDMapperV2::readTimer(EmuTime time) const
{
	// A 16-bit counter loaded with (value << 8) | FFh, decremented every
	// 40 ns (25 MHz) down to 0; the CPU reads its high byte.
	double elapsed = (time - timerStart).toDouble();
	auto ticks = uint64_t(elapsed / 40e-9);
	uint32_t count = (uint32_t(timerValue) << 8) | 0xFF;
	if (ticks >= count) return 0;
	return narrow_cast<uint8_t>((count - ticks) >> 8);
}

uint8_t SDMapperV2::readMem(uint16_t address, EmuTime time)
{
	if ((0x4000 <= address) && (address < 0x8000)) {
		if (spiVisible() && (0x7B00 <= address) && (address < 0x7F00)) {
			return spiTransfer(0xFF);
		}
		if (blockVisible(address)) return readBlock(address, time);
		if (address == 0x7FF0) return readStatus();
		if (address == 0x7FF1) return readTimer(time);
		if (address >= 0x7FF2) return 0xFF;
		return rom[0x4000 * bank1 + (address & 0x3FFF)];
	}
	if ((0x8000 <= address) && (address < 0xC000)) {
		if (bank2 & 0x08) {
			return rom[0x4000 * (bank2 & 0x07) + (address & 0x3FFF)];
		}
		return 0xFF;
	}
	return 0xFF;
}

uint8_t SDMapperV2::peekMem(uint16_t address, EmuTime time) const
{
	if ((0x4000 <= address) && (address < 0x8000)) {
		if (spiVisible() && (0x7B00 <= address) && (address < 0x7F00)) {
			return 0xFF; // a transfer cannot be peeked
		}
		if (blockVisible(address)) return peekBlock(address, time);
		if (address == 0x7FF0) return readStatus();
		if (address == 0x7FF1) return readTimer(time);
		if (address >= 0x7FF2) return 0xFF;
		return rom[0x4000 * bank1 + (address & 0x3FFF)];
	}
	if ((0x8000 <= address) && (address < 0xC000)) {
		if (bank2 & 0x08) {
			return rom[0x4000 * (bank2 & 0x07) + (address & 0x3FFF)];
		}
		return 0xFF;
	}
	return 0xFF;
}

const uint8_t* SDMapperV2::getReadCacheLine(uint16_t start) const
{
	if ((0x4000 <= start) && (start < 0x8000)) {
		if (spiVisible() && (0x7B00 <= start) && (start < 0x7F00)) {
			return nullptr;
		}
		if (start >= 0x7F00) return nullptr; // the registers' line
		return &rom[0x4000 * bank1 + (start & 0x3FFF)];
	}
	if ((0x8000 <= start) && (start < 0xC000)) {
		if (bank2 & 0x08) {
			return &rom[0x4000 * (bank2 & 0x07) + (start & 0x3FFF)];
		}
		return unmappedRead.data();
	}
	return unmappedRead.data();
}

void SDMapperV2::writeMem(uint16_t address, uint8_t value, EmuTime time)
{
	if ((address & 0xE800) == 0x6000) {
		// 6000h-67FFh and 7000h-77FFh: the two bank registers
		if (address & 0x1000) {
			bank2 = value & 0x0F;
			invalidateDeviceRCache(0x8000, 0x4000);
		} else {
			bank1 = value & 0x07;
			invalidateDeviceRCache(0x4000, 0x4000);
		}
		return;
	}
	if (spiVisible() && (0x7B00 <= address) && (address < 0x7F00)) {
		(void)spiTransfer(value);
		return;
	}
	if (blockVisible(address)) {
		writeBlock(address, value, time);
		return;
	}
	if (address == 0x7FF0) {
		cardSelect = value & 0x03;
		return;
	}
	if (address == 0x7FF1) {
		timerValue = value;
		timerStart = time;
		return;
	}
	// writes to the flash through 8000h-BFFFh (the updater) are ignored
}

// version 2: bdPendingResult (the FAT accelerator's commands)
SERIALIZE_CLASS_VERSION(SDMapperV2, 2);

template<typename Archive>
void SDMapperV2::serialize(Archive& ar, unsigned version)
{
	ar.template serializeBase<MSXDevice>(*this);
	ar.serialize("sdCard0",    *sdCard[0],
	             "sdCard1",    *sdCard[1],
	             "timerStart", timerStart,
	             "timerValue", timerValue,
	             "bank1",      bank1,
	             "bank2",      bank2,
	             "cardSelect", cardSelect,
	             "bdReg",      bdReg,
	             "bdStatus",   bdStatus,
	             "bdPending",  bdPending,
	             "bdResult",   bdResult,
	             "bdBusy",     bdBusy,
	             "bdDone",     bdDone,
	             "bdBuffer",   bdBuffer,
	             "bdTarget",   bdTarget);
	if (ar.versionAtLeast(version, 2)) {
		ar.serialize("bdPendingResult", bdPendingResult);
	}
	if constexpr (Archive::IS_LOADER) {
		invalidateDeviceRCache(0x4000, 0x8000);
	}
}
INSTANTIATE_SERIALIZE_METHODS(SDMapperV2);
REGISTER_MSXDEVICE(SDMapperV2, "SDMapperV2");


SDMapperV2::Blocks::Blocks(const SDMapperV2& device)
	: RomBlockDebuggableBase(device)
{
}

SDMapperV2::BlockDebuggable::BlockDebuggable(const SDMapperV2& device)
	: SimpleDebuggable(device.getMotherBoard(), device.getName() + " m6block",
	                   "The m6 block device's registers (00h-3Fh), then the "
	                   "commands started: FAT accelerator (40h-43h), Turbo (44h-47h).",
	                   0x48)
{
}

uint8_t SDMapperV2::BlockDebuggable::read(unsigned address, EmuTime time)
{
	const auto& dev = OUTER(SDMapperV2, blockDebug);
	if (address < 0x40) return dev.peekBlock(narrow<uint16_t>(0x7F00 + address), time);
	uint32_t v = (address < 0x44) ? dev.fatCommands : dev.turboCommands;
	return narrow_cast<uint8_t>(v >> (8 * (address & 3)));
}

unsigned SDMapperV2::Blocks::readExt(unsigned address)
{
	const auto& dev = OUTER(SDMapperV2, romBlockDebug);
	if ((0x4000 <= address) && (address < 0x8000)) return dev.bank1;
	if ((0x8000 <= address) && (address < 0xC000)) {
		return (dev.bank2 & 0x08) ? (dev.bank2 & 0x07) : unsigned(-1);
	}
	return unsigned(-1);
}

} // namespace openmsx
