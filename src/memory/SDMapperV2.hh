#ifndef SDMAPPERV2_HH
#define SDMAPPERV2_HH

#include "MSXDevice.hh"
#include "Rom.hh"
#include "RomBlockDebuggable.hh"
#include "EmuTime.hh"

#include <array>
#include <memory>

namespace openmsx {

class SdCard;

/** FBLabs SD Mapper V2: an ASCII16 mapper cartridge with up to two SD
  * cards on an SPI port. The mapper and slot expander the cartridge also
  * carries are not emulated; this device is the storage half, which is
  * what its Nextor driver (and m6) talk to.
  */
class SDMapperV2 final : public MSXDevice
{
public:
	explicit SDMapperV2(DeviceConfig& config);
	~SDMapperV2() override;

	void reset(EmuTime time) override;

	[[nodiscard]] uint8_t readMem(uint16_t address, EmuTime time) override;
	[[nodiscard]] uint8_t peekMem(uint16_t address, EmuTime time) const override;
	void writeMem(uint16_t address, uint8_t value, EmuTime time) override;
	[[nodiscard]] const uint8_t* getReadCacheLine(uint16_t start) const override;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	[[nodiscard]] bool spiVisible() const { return bank1 == 7; }
	[[nodiscard]] uint8_t spiTransfer(uint8_t value);
	[[nodiscard]] uint8_t readStatus() const;
	[[nodiscard]] uint8_t readTimer(EmuTime time) const;

private:
	struct Blocks final : RomBlockDebuggableBase {
		explicit Blocks(const SDMapperV2& device);
		[[nodiscard]] unsigned readExt(unsigned address) override;
	} romBlockDebug;

	Rom rom;
	std::array<std::unique_ptr<SdCard>, 2> sdCard;
	std::array<bool, 2> cardDeclared; // a slot with no <sdcardN> is empty
	EmuTime timerStart = EmuTime::zero();
	uint8_t timerValue = 0;
	uint8_t bank1 = 0;      // the bank in 4000h-7FFFh, 3 bits
	uint8_t bank2 = 0;      // the bank in 8000h-BFFFh, 4 bits; bit 3 enables reads there
	uint8_t cardSelect = 0; // 7FF0h bits 1-0: chip select of card 1 and card 2
};

} // namespace openmsx

#endif
