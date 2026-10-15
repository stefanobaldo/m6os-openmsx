#ifndef SDCARD_HH
#define SDCARD_HH

#include "circular_buffer.hh"
#include "DiskImageUtils.hh"

#include <array>
#include <cstdint>
#include <memory>

namespace openmsx {

class DeviceConfig;
class DiskManipulator;
class HD;

class SdCard
{
public:
	explicit SdCard(const DeviceConfig& config);
	~SdCard();

	uint8_t transfer(uint8_t value, bool cs);
	[[nodiscard]] bool isWriteProtected() const;

	/** Whole-sector access beside the SPI protocol, for a cartridge that
	  * moves sectors itself (the m6 block device's Turbo commands in
	  * SDMapperV2). A card with no image behind it has no medium; a read
	  * or write of a sector past the end, or one the image refuses,
	  * throws MSXException as HD does. The SPI state machine is left as
	  * it is: a driver uses one path or the other for a call.
	  */
	[[nodiscard]] bool hasMedium() const { return hd != nullptr; }
	[[nodiscard]] size_t getNbSectors() const;
	void readSector(size_t sector, SectorBuffer& buf);
	void writeSector(size_t sector, const SectorBuffer& buf);

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

// private:
	enum class Mode : uint8_t {
		COMMAND,
		READ,
		MULTI_READ,
		WRITE,
		MULTI_WRITE
	};

private:
	void executeCommand();
	[[nodiscard]] uint8_t readCurrentByteFromCurrentSector();

private:
	const std::unique_ptr<HD> hd; // can be nullptr
	DiskManipulator* diskManipulator = nullptr; // set when hd is: the card is reachable as hdX
	const bool sdhc; // false: a standard-capacity (SDv1) card, addressed by byte

	std::array<uint8_t, 6> cmdBuf;
	SectorBuffer sectorBuf;
	unsigned cmdIdx = 0;

	cb_queue<uint8_t> responseQueue;

	uint8_t transferDelayCounter = 0;
	Mode mode = Mode::COMMAND;
	unsigned currentSector = 0;
	int currentByteInSector = 0;
};

} // namespace openmsx

#endif
