#pragma once

// Compile-only Watchdog stub (CPP-089 slice 1).
//
// Upstream surface (AP_HAL_ChibiOS/hwdef/common/watchdog.h):
//   stm32_watchdog_init / pat / was_watchdog_reset
//   stm32_watchdog_save / load  — uint32 word copy of PersistentData
//
// This is not a port of watchdog.c IWDG MMIO (IWDGD.KR unlock / start /
// reload, RCC reset-status bits). SITL HAL_SITL_Class.cpp watchdog_save
// writes persistent.dat via open/write; this stub never opens files,
// never syscalls reboot, never pokes IWDG/WWDG.
//
// save/load copy a caller-owned uint32 span into a fixed 19-word buffer
// (ESP32 Util.h: sizeof(persistent_data) <= 19*4; ChibiOS RTC backup
// starts at idx 1, leaving 19 words). Conceptual match only.

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

namespace fwcpp::hal {

inline constexpr std::size_t kWatchdogPersistentWords = 19;

class Watchdog {
public:
    void init() {
        enabled_ = true;
        inited_ = true;
    }

    // Upstream pat writes IWDG only when watchdog_enabled.
    void pat() {
        if (enabled_) {
            ++pat_count_;
        }
    }

    [[nodiscard]] bool is_initialized() const { return inited_; }
    [[nodiscard]] bool is_enabled() const { return enabled_; }
    [[nodiscard]] std::uint32_t pat_count() const { return pat_count_; }

    // Injected/stored bool. Upstream ChibiOS reads RCC IWDG reset bits;
    // SITL reads getenv("SITL_WATCHDOG_RESET"). Neither is reproduced.
    void set_was_reset(bool reset) { was_reset_ = reset; }
    [[nodiscard]] bool was_reset() const { return was_reset_; }

    void save(std::span<const std::uint32_t> words) {
        const std::size_t n = std::min(words.size(), backup_.size());
        for (std::size_t i = 0; i < n; ++i) {
            backup_[i] = words[i];
        }
        saved_n_ = n;
    }

    void load(std::span<std::uint32_t> words) const {
        const std::size_t n = std::min(words.size(), saved_n_);
        for (std::size_t i = 0; i < n; ++i) {
            words[i] = backup_[i];
        }
    }

    [[nodiscard]] std::size_t saved_word_count() const { return saved_n_; }

private:
    bool inited_ = false;
    bool enabled_ = false;
    bool was_reset_ = false;
    std::uint32_t pat_count_ = 0;
    std::array<std::uint32_t, kWatchdogPersistentWords> backup_{};
    std::size_t saved_n_ = 0;
};

}  // namespace fwcpp::hal
