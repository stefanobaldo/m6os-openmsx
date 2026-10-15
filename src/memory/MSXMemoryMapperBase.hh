#ifndef MSXMEMORYMAPPERBASE_HH
#define MSXMEMORYMAPPERBASE_HH

#include "MSXDevice.hh"
#include "MSXMapperIO.hh"
#include "CheckedRam.hh"
#include "SimpleDebuggable.hh"
#include "narrow.hh"
#include <array>
#include <span>

namespace openmsx {

class MSXMemoryMapperBase : public MSXDevice, public MSXMapperIOClient
{
public:
	explicit MSXMemoryMapperBase(const DeviceConfig& config);

	/**
	 * Returns the currently selected segment for the given page.
	 * @param page Z80 address page (0-3).
	 */
	[[nodiscard]] byte getSelectedSegment(byte page) const override { return registers[page]; }

	[[nodiscard]] unsigned getSizeInBlocks() const {
		return narrow<unsigned>(checkedRam.size() / 0x4000);
	}

	/** Access to the mapper's RAM by another device — a cartridge that
	  * writes a sector into the memory it serves (the m6 block device's
	  * Turbo mode in SDMapperV2). Addresses are offsets into the RAM
	  * (segment * 0x4000 + offset), not Z80 addresses; the caller keeps
	  * them inside getRamBytes(). writeRamBlock invalidates the CPU's
	  * cache lines of this device afterwards.
	  */
	[[nodiscard]] size_t getRamBytes() const { return checkedRam.size(); }
	[[nodiscard]] uint8_t peekRam(size_t addr) const { return checkedRam.peek(addr); }
	void writeRamBlock(size_t addr, std::span<const uint8_t> data);

	void reset(EmuTime time) override;
	void powerUp(EmuTime time) override;
	[[nodiscard]] byte readIO(uint16_t port, EmuTime time) override;
	[[nodiscard]] byte peekIO(uint16_t port, EmuTime time) const override;
	[[nodiscard]] byte readMem(uint16_t address, EmuTime time) override;
	void writeMem(uint16_t address, byte value, EmuTime time) override;
	[[nodiscard]] const byte* getReadCacheLine(uint16_t start) const override;
	[[nodiscard]] byte* getWriteCacheLine(uint16_t start) override;
	[[nodiscard]] byte peekMem(uint16_t address, EmuTime time) const override;
	[[nodiscard]] unsigned getBaseSizeAlignment() const override;

	// Subclasses _must_ override this method and
	//  - call MSXMemoryMapperBase::writeIOImpl()
	//  - handle CPU cacheline stuff (e.g. invalidate)
	void writeIO(uint16_t port, byte value, EmuTime time) override = 0;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

protected:
	/** Converts a Z80 address to a RAM address.
	  * @param address Index in Z80 address space.
	  * @return Index in RAM address space.
	  */
	[[nodiscard]] unsigned calcAddress(uint16_t address) const;
	[[nodiscard]] unsigned segmentOffset(byte page) const;

	void writeIOImpl(uint16_t port, byte value, EmuTime time);

	CheckedRam checkedRam;
	std::array<byte, 4> registers;

private:
	struct Debuggable final : SimpleDebuggable {
		Debuggable(MSXMotherBoard& motherBoard, const std::string& name);
		[[nodiscard]] byte read(unsigned address) override;
		void write(unsigned address, byte value) override;
		void readBlock(unsigned start, std::span<byte> output) override;
	} debuggable;
};
SERIALIZE_CLASS_VERSION(MSXMemoryMapperBase, 2);

REGISTER_BASE_NAME_HELPER(MSXMemoryMapperBase, "MemoryMapper"); // keep old name for bw-compat

} // namespace openmsx

#endif
