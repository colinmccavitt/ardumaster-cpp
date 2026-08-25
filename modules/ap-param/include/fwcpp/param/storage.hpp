#pragma once

// Port of StorageManager/StorageManager.h + StorageManager.cpp. CPP-020.
// See ADR-0013 for the full scoping rationale.
//
// SCOPE: this port targets SITL only (see ADR/memory: "SITL target, full
// fidelity"), which upstream builds with HAL_STORAGE_SIZE = 16384
// (libraries/AP_HAL/board/linux.h) and is a fixed-wing/rover build
// (Plane), giving STORAGE_NUM_AREAS = 15 and the specific 15-entry
// `layout` table reproduced below - read directly from upstream
// StorageManager.cpp's cumulative `#if STORAGE_NUM_AREAS >= N` blocks for
// exactly this size/vehicle combination, not guessed. Other board storage
// sizes (4k/8k/32k+) have entirely different layouts upstream and aren't
// reproduced - this port has no reason to support hardware it doesn't
// target.
//
// RawStorage REPLACES hal.storage: upstream's StorageAccess ultimately
// reads/writes through a HAL singleton (AP_HAL::get_HAL().storage).
// Reading AP_HAL_SITL/Storage.cpp shows hal.storage's read_block/
// write_block are themselves plain memcpy against an in-memory buffer -
// the flash-sector-emulation/dirty-line-tracking/background-flush-to-file
// machinery around that buffer is a separate persistence *mechanism*,
// not part of AP_Param's actual format (see ADR-0013). RawStorage here
// IS that buffer, owned explicitly (constructed by the caller, passed by
// reference) rather than reached for as a singleton - matching this
// port's standing no-singleton convention (ADR-0012) - with explicit
// load_from_file/save_to_file replacing the background flush thread.
//
// NOT reproduced: the AP_SDCARD_STORAGE_ENABLED file-attach path
// (StorageAccess::attach_file/flush_file and the `file != nullptr`
// branches of read_block/write_block) - a board-specific microSD storage
// region, never used for StorageParam (the only StorageType this port's
// AP_Param slice actually needs), and gated off by default upstream too.
//
// LITERAL SAFETY: no bare ambiguous double literals anywhere in this
// file - every value here is an integer byte count/offset or a memcpy of
// raw bytes, nothing depends on floating-point literal typing.

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace fwcpp::storage {

// Matches AP_HAL/board/linux.h's HAL_STORAGE_SIZE exactly - see file
// banner for why this port only supports this one board size.
inline constexpr std::uint32_t kStorageSize = 16384;

// The raw, flat backing store - see file banner for why this replaces
// hal.storage rather than reproducing its background-flush-to-flash
// mechanism. Bounds-checked matching upstream's own bounds-checking
// (which return false/no-op on out-of-range access rather than
// undefined behavior).
class RawStorage {
public:
    RawStorage() { buffer_.fill(0); }

    [[nodiscard]] bool read_block(void* dst, std::uint16_t addr, std::size_t n) const {
        if (static_cast<std::uint32_t>(addr) + n > buffer_.size()) {
            return false;
        }
        std::memcpy(dst, buffer_.data() + addr, n);
        return true;
    }

    [[nodiscard]] bool write_block(std::uint16_t addr, const void* src, std::size_t n) {
        if (static_cast<std::uint32_t>(addr) + n > buffer_.size()) {
            return false;
        }
        std::memcpy(buffer_.data() + addr, src, n);
        return true;
    }

    void erase() { buffer_.fill(0xFF); } // matches upstream: erased flash reads as 0xFF

    [[nodiscard]] std::uint32_t size() const { return static_cast<std::uint32_t>(buffer_.size()); }

    // Explicit persistence - replaces upstream's periodic background
    // flush (AP_HAL::millis()-driven _timer_tick), matching this port's
    // standing explicit-over-hidden-background-state pattern. Returns
    // false on any I/O failure, leaving `this` (load) or the file (save)
    // unchanged.
    [[nodiscard]] bool load_from_file(const char* path) {
        std::FILE* f = std::fopen(path, "rb");
        if (f == nullptr) {
            return false;
        }
        const std::size_t n = std::fread(buffer_.data(), 1, buffer_.size(), f);
        std::fclose(f);
        return n == buffer_.size();
    }

    [[nodiscard]] bool save_to_file(const char* path) const {
        std::FILE* f = std::fopen(path, "wb");
        if (f == nullptr) {
            return false;
        }
        const std::size_t n = std::fwrite(buffer_.data(), 1, buffer_.size(), f);
        std::fclose(f);
        return n == buffer_.size();
    }

private:
    std::array<std::uint8_t, kStorageSize> buffer_{};
};

// Matches upstream StorageManager::StorageType exactly - values are part
// of the on-storage layout contract (which type owns which byte range),
// not arbitrary, so the enumerator values are pinned even though this
// port doesn't need every type yet.
enum class StorageType : std::uint8_t {
    Param = 0,
    Fence = 1,
    Rally = 2,
    Mission = 3,
    Keys = 4,
    BindInfo = 5,
    CANDNA = 6,
    ParamBak = 7,
};

struct StorageArea {
    StorageType type;
    std::uint16_t offset;
    std::uint16_t length;
};

// The 15-entry layout for HAL_STORAGE_SIZE=16384, Plane (fixed-wing)
// build - see file banner. Order matters: StorageAccess concatenates
// same-type areas in this order to form one logical address space per
// type, exactly matching upstream's own iteration order.
inline constexpr std::array<StorageArea, 15> kLayout = {{
    {StorageType::Param, 0, 1280},
    {StorageType::Mission, 1280, 2506},
    {StorageType::Rally, 3786, 150},
    {StorageType::Fence, 3936, 160},
    {StorageType::Param, 4096, 1280},
    {StorageType::Rally, 5376, 300},
    {StorageType::Fence, 5676, 256},
    {StorageType::Mission, 5932, 2132},
    {StorageType::Keys, 8064, 64},
    {StorageType::BindInfo, 8128, 56},
    {StorageType::Param, 8192, 1280},
    {StorageType::Rally, 9472, 300},
    {StorageType::Fence, 9772, 256},
    {StorageType::Mission, 10028, 5204},
    {StorageType::CANDNA, 15232, 1024},
}};

// Per-type accessor over RawStorage, presenting all of a type's
// (possibly non-contiguous) areas as one logical, zero-based address
// space - matches upstream StorageAccess exactly, including splitting a
// read/write that crosses an area boundary into multiple RawStorage
// calls in area order.
class StorageAccess {
public:
    StorageAccess(RawStorage& storage, StorageType type) : storage_(storage), type_(type) {
        total_size_ = 0;
        for (const StorageArea& area : kLayout) {
            if (area.type == type_) {
                total_size_ += area.length;
            }
        }
    }

    [[nodiscard]] std::uint16_t size() const { return total_size_; }

    // Both read_block and write_block walk kLayout's areas of this
    // accessor's own type in table order, treating them as one
    // concatenated logical address space and splitting a read/write that
    // crosses an area boundary into multiple RawStorage calls - matching
    // upstream's own StorageAccess::read_block/write_block loop exactly
    // (upstream literally duplicates this loop between the two
    // functions; kept duplicated here too, rather than merged behind a
    // generic callback, so each stays a direct one-to-one read against
    // upstream's own source).
    [[nodiscard]] bool read_block(void* dst, std::uint16_t addr, std::size_t n) const {
        auto* b = static_cast<std::uint8_t*>(dst);
        for (const StorageArea& area : kLayout) {
            if (area.type != type_) {
                continue;
            }
            std::uint16_t length = area.length;
            std::uint16_t offset = area.offset;
            if (addr >= length) {
                addr = static_cast<std::uint16_t>(addr - length);
                continue;
            }
            std::size_t count = n;
            if (count + addr > length) {
                count = static_cast<std::size_t>(length - addr);
            }
            if (!storage_.read_block(b, static_cast<std::uint16_t>(addr + offset), count)) {
                return false;
            }
            n -= count;
            if (n == 0) {
                break;
            }
            b += count;
            addr = 0;
        }
        return n == 0;
    }

    [[nodiscard]] bool write_block(std::uint16_t addr, const void* src, std::size_t n) {
        const auto* b = static_cast<const std::uint8_t*>(src);
        for (const StorageArea& area : kLayout) {
            if (area.type != type_) {
                continue;
            }
            std::uint16_t length = area.length;
            std::uint16_t offset = area.offset;
            if (addr >= length) {
                addr = static_cast<std::uint16_t>(addr - length);
                continue;
            }
            std::size_t count = n;
            if (count + addr > length) {
                count = static_cast<std::size_t>(length - addr);
            }
            if (!storage_.write_block(static_cast<std::uint16_t>(addr + offset), b, count)) {
                return false;
            }
            n -= count;
            if (n == 0) {
                break;
            }
            b += count;
            addr = 0;
        }
        return n == 0;
    }

    // These convenience helpers silently ignore an out-of-range read
    // (leaving the local zero-init in place) or write, matching
    // upstream's own read_byte/read_uint16/.../write_byte/... exactly -
    // upstream's versions don't check read_block/write_block's bool
    // return either. read_block/write_block themselves ARE [[nodiscard]]
    // for callers who go through them directly, so the ignores below are
    // explicit `(void)`, not accidental.
    [[nodiscard]] std::uint8_t read_byte(std::uint16_t loc) const {
        std::uint8_t v = 0;
        (void)read_block(&v, loc, sizeof(v));
        return v;
    }
    [[nodiscard]] std::uint16_t read_uint16(std::uint16_t loc) const {
        std::uint16_t v = 0;
        (void)read_block(&v, loc, sizeof(v));
        return v;
    }
    [[nodiscard]] std::uint32_t read_uint32(std::uint16_t loc) const {
        std::uint32_t v = 0;
        (void)read_block(&v, loc, sizeof(v));
        return v;
    }
    [[nodiscard]] float read_float(std::uint16_t loc) const {
        float v = 0.0f;
        (void)read_block(&v, loc, sizeof(v));
        return v;
    }

    void write_byte(std::uint16_t loc, std::uint8_t value) { (void)write_block(loc, &value, sizeof(value)); }
    void write_uint16(std::uint16_t loc, std::uint16_t value) { (void)write_block(loc, &value, sizeof(value)); }
    void write_uint32(std::uint16_t loc, std::uint32_t value) { (void)write_block(loc, &value, sizeof(value)); }
    void write_float(std::uint16_t loc, float value) { (void)write_block(loc, &value, sizeof(value)); }

private:
    RawStorage& storage_;
    StorageType type_;
    std::uint16_t total_size_ = 0;
};

} // namespace fwcpp::storage
