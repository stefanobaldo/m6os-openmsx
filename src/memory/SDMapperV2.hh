#ifndef SDMAPPERV2_HH
#define SDMAPPERV2_HH

#include "MSXDevice.hh"
#include "Rom.hh"
#include "RomBlockDebuggable.hh"
#include "EmuTime.hh"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace openmsx {

class SdCard;
class MSXMemoryMapperBase;

/** FBLabs SD Mapper V2: an ASCII16 mapper cartridge with up to two SD
  * cards on an SPI port. The mapper and slot expander the cartridge also
  * carries are not emulated by this class; this device is the storage
  * half, which is what its Nextor driver (and m6) talk to. With an
  * <m6block> child it also presents the m6 block device's register set
  * at 7F00h-7F3Fh of bank 7 — the reference implementation of that
  * specification (m6's docs/block-device.md) — whose Turbo commands move
  * sectors between a card and the RAM of a MemoryMapper device named in
  * the configuration, the way a cartridge that is itself the machine's
  * mapper does.
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
	[[nodiscard]] bool blockVisible(uint16_t address) const {
		return blockPresent && spiVisible() && (0x7F00 <= address) && (address < 0x7F40);
	}
	[[nodiscard]] uint8_t readBlock(uint16_t address, EmuTime time);
	[[nodiscard]] uint8_t peekBlock(uint16_t address, EmuTime time) const;
	void writeBlock(uint16_t address, uint8_t value, EmuTime time);
	void startCommand(EmuTime time);
	void settleCommand(EmuTime time);
	[[nodiscard]] MSXMemoryMapperBase* findMapper();

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

	// The m6 block device (<m6block>), see the header comment of the .cc
	bool blockPresent = false;
	uint8_t blockSubslot = 3;     // BD_CAPS bits 5-4: where the mapper is
	unsigned turboUs = 50;        // modelled time a sector takes
	std::string mapperName;       // the MemoryMapper device the commands write; "" = none (a device that offers Turbo with no RAM)
	MSXMemoryMapperBase* mapper = nullptr; // resolved at the first command
	bool mapperLooked = false;
	std::array<uint8_t, 0x40> bdReg = {}; // the registers as written
	uint8_t bdStatus = 0;         // BD_STAT once the command is done
	uint8_t bdPending = 0;        // the status a running command will end with
	uint32_t bdResult = 0;        // BD_RES
	bool bdBusy = false;
	EmuTime bdDone = EmuTime::zero();
	std::vector<uint8_t> bdBuffer; // a read's data, applied at completion
	uint32_t bdTarget = 0;        // its address in the mapper's RAM
};

} // namespace openmsx

#endif
