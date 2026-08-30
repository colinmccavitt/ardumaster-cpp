#pragma once

// Port of AP_Logger_Block BufferToPage / PageToBuffer device seam.
// CPP-090, slice 7.
//
// Upstream: AP_Logger_Block.h virtual BufferToPage / PageToBuffer;
// AP_Logger_Flash_JEDEC.cpp uses df_PageSize=256 and 1-indexed pages
// (page 0 / past df_NumPages+1 rejected; PageToBuffer fills 0xFF on
// invalid). FinishWrite (Block.cpp ~89-98) flushes then advances
// df_PageAdr with wrap to 1 when past df_NumPages.
//
// This is a RAM page map, not NAND/SPI. Separate from MemoryBackend
// (append-only byte buffer with no page erase). Working buffer is
// page-sized; BufferToPage / PageToBuffer copy between buffer and the
// page array. ErasePage zero-fills (test-friendly; JEDEC uses 0xFF).
// No SectorErase / chip erase / DMA / chip-select.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace fwcpp::logger {

// JEDEC NOR page size (AP_Logger_Flash_JEDEC.cpp Init).
inline constexpr std::size_t kDfPageSize = 256;

// Small capacity for unit tests (not a real chip page count).
inline constexpr std::size_t kDfTestPageCount = 16;

template <std::size_t PageSize = kDfPageSize, std::size_t PageCount = kDfTestPageCount>
class DataFlashPageMap {
public:
    static_assert(PageSize > 0);
    static_assert(PageCount > 0);

    DataFlashPageMap() = default;

    [[nodiscard]] static constexpr std::size_t page_size() { return PageSize; }
    [[nodiscard]] static constexpr std::size_t page_count() { return PageCount; }

    [[nodiscard]] std::span<std::uint8_t> buffer() {
        return std::span<std::uint8_t>(buffer_.data(), PageSize);
    }
    [[nodiscard]] std::span<const std::uint8_t> buffer() const {
        return std::span<const std::uint8_t>(buffer_.data(), PageSize);
    }

    // Fill the working buffer (tests / writers). Truncates to PageSize.
    void fill_buffer(std::span<const std::uint8_t> bytes) {
        const std::size_t n = std::min(bytes.size(), PageSize);
        std::copy_n(bytes.begin(), n, buffer_.begin());
        if (n < PageSize) {
            std::fill(buffer_.begin() + static_cast<std::ptrdiff_t>(n), buffer_.end(),
                      std::uint8_t{0});
        }
    }

    // AP_Logger_Flash_JEDEC::BufferToPage — write working buffer to page.
    // Pages are 1-indexed. Returns false for page 0 or > PageCount.
    [[nodiscard]] bool BufferToPage(std::uint32_t page_num) {
        if (!valid_page(page_num)) {
            return false;
        }
        auto& page = pages_[page_num - 1];
        std::copy(buffer_.begin(), buffer_.end(), page.begin());
        return true;
    }

    // AP_Logger_Flash_JEDEC::PageToBuffer — load page into working buffer.
    // Invalid page zero-fills the buffer and returns false (RAM model;
    // JEDEC fills 0xFF).
    [[nodiscard]] bool PageToBuffer(std::uint32_t page_num) {
        if (!valid_page(page_num)) {
            std::fill(buffer_.begin(), buffer_.end(), std::uint8_t{0});
            return false;
        }
        const auto& page = pages_[page_num - 1];
        std::copy(page.begin(), page.end(), buffer_.begin());
        return true;
    }

    // Zero-fill one page (no SPI SectorErase). Invalid page is a no-op.
    void ErasePage(std::uint32_t page_num) {
        if (!valid_page(page_num)) {
            return;
        }
        auto& page = pages_[page_num - 1];
        std::fill(page.begin(), page.end(), std::uint8_t{0});
    }

    // FinishWrite wrap: advance after BufferToPage; wrap to 1 past last.
    [[nodiscard]] static constexpr std::uint32_t wrap_page(std::uint32_t page_adr) {
        if (page_adr >= PageCount) {
            return 1;
        }
        return page_adr + 1;
    }

    // Read back a stored page without touching the working buffer.
    [[nodiscard]] std::span<const std::uint8_t> page_bytes(std::uint32_t page_num) const {
        if (!valid_page(page_num)) {
            return {};
        }
        const auto& page = pages_[page_num - 1];
        return std::span<const std::uint8_t>(page.data(), PageSize);
    }

private:
    [[nodiscard]] static constexpr bool valid_page(std::uint32_t page_num) {
        return page_num >= 1 && page_num <= PageCount;
    }

    std::array<std::uint8_t, PageSize> buffer_{};
    std::array<std::array<std::uint8_t, PageSize>, PageCount> pages_{};
};

// Thin FinishWrite-style helper: BufferToPage(page_adr) then wrap.
// Returns the next page address (1 after wrap). Does not SectorErase.
// Returns 0 if BufferToPage rejected page_adr.
template <std::size_t PageSize, std::size_t PageCount>
[[nodiscard]] std::uint32_t finish_write_page(DataFlashPageMap<PageSize, PageCount>& map,
                                              std::uint32_t page_adr) {
    if (!map.BufferToPage(page_adr)) {
        return 0;
    }
    return DataFlashPageMap<PageSize, PageCount>::wrap_page(page_adr);
}

}  // namespace fwcpp::logger
