#include "SDMapperV2.hh"

#include "SdCard.hh"
#include "DeviceConfig.hh"
#include "MSXException.hh"
#include "serialize.hh"

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
*/

namespace openmsx {

SDMapperV2::SDMapperV2(DeviceConfig& config)
	: MSXDevice(config)
	, romBlockDebug(*this)
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
	invalidateDeviceRCache(0x4000, 0x8000);
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

template<typename Archive>
void SDMapperV2::serialize(Archive& ar, unsigned /*version*/)
{
	ar.template serializeBase<MSXDevice>(*this);
	ar.serialize("sdCard0",    *sdCard[0],
	             "sdCard1",    *sdCard[1],
	             "timerStart", timerStart,
	             "timerValue", timerValue,
	             "bank1",      bank1,
	             "bank2",      bank2,
	             "cardSelect", cardSelect);
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
