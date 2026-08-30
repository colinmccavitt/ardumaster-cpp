#pragma once

// PARAM_REQUEST_LIST (msgid 21) / PARAM_VALUE (22) / PARAM_SET (23) for
// CPP-087 slice 3. Upstream: GCS_Param.cpp handle_param_request_list /
// queued_param_send / handle_param_set / send_parameter_value, and
// AP_Param::save_sync(send_to_gcs) which emits PARAM_VALUE after a set.
// Injected ParamStore stands in for AP_Param (ADR-0012: no singleton).
// No PARAM_REQUEST_READ, PARAM_ERROR, MISSION, or vehicle handlers.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <fwcpp/gcs/framing.hpp>

namespace fwcpp::gcs {

inline constexpr std::size_t kParamRequestListLen = 2;
inline constexpr std::size_t kParamValueLen = 25;
inline constexpr std::size_t kParamSetLen = 23;
inline constexpr std::size_t kParamIdLen = 16;
inline constexpr std::size_t kParamNameCapacity = kParamIdLen + 1;
inline constexpr std::size_t kMaxParams = 8;

// common.xml MAV_PARAM_TYPE_REAL32. Store type is caller-supplied;
// mav_param_type() maps other AP_Param types to this when not INT8/16/32.
inline constexpr std::uint8_t kMavParamTypeReal32 = 9;

// send_parameter_value uses param_index = -1 after PARAM_SET.
inline constexpr std::uint16_t kParamIndexSetAck = static_cast<std::uint16_t>(-1);

struct ParamRequestList {
    std::uint8_t target_system{};
    std::uint8_t target_component{};
};

// Size-sorted v2 wire (like HEARTBEAT / COMMAND_LONG): param_value float,
// target_system, target_component, param_id[16], param_type.
struct ParamSet {
    float param_value{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
    std::uint8_t param_id[kParamIdLen]{};
    std::uint8_t param_type{};
};

// Size-sorted v2 wire: param_value float, param_count, param_index,
// param_id[16], param_type.
struct ParamValue {
    float param_value{};
    std::uint16_t param_count{};
    std::uint16_t param_index{};
    std::uint8_t param_id[kParamIdLen]{};
    std::uint8_t param_type{};
};

struct ParamEntry {
    char name[kParamNameCapacity]{};
    float value{};
    std::uint8_t type{kMavParamTypeReal32};
};

// Fixed table injected by the caller. Not AP_Param.
struct ParamStore {
    ParamEntry entries[kMaxParams]{};
    std::uint16_t count{0};
};

enum class ParamSetStatus : std::uint8_t {
    kApplied = 0,
    kUnknown = 1,
    kRejected = 2,
};

inline void copy_name_to_param_id(std::uint8_t* id, const char* name) {
    std::size_t i = 0;
    if (name != nullptr) {
        for (; i < kParamIdLen && name[i] != '\0'; ++i) {
            id[i] = static_cast<std::uint8_t>(name[i]);
        }
    }
    for (; i < kParamIdLen; ++i) {
        id[i] = 0;
    }
}

inline void copy_param_id_to_name(char* name, const std::uint8_t* id) {
    std::size_t i = 0;
    for (; i < kParamIdLen && id[i] != 0; ++i) {
        name[i] = static_cast<char>(id[i]);
    }
    name[i] = '\0';
}

[[nodiscard]] inline bool param_name_eq(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return a == b;
    }
    return std::strcmp(a, b) == 0;
}

[[nodiscard]] inline std::size_t pack_param_request_list(const ParamRequestList& req,
                                                         std::span<std::uint8_t> buf) {
    if (buf.size() < kParamRequestListLen) {
        return 0;
    }
    buf[0] = req.target_system;
    buf[1] = req.target_component;
    return kParamRequestListLen;
}

[[nodiscard]] inline bool unpack_param_request_list(std::span<const std::uint8_t> buf,
                                                    ParamRequestList& out) {
    if (buf.size() < kParamRequestListLen) {
        return false;
    }
    out.target_system = buf[0];
    out.target_component = buf[1];
    return true;
}

[[nodiscard]] inline std::size_t pack_param_set(const ParamSet& set, std::span<std::uint8_t> buf) {
    if (buf.size() < kParamSetLen) {
        return 0;
    }
    write_f32_le(buf.data() + 0, set.param_value);
    buf[4] = set.target_system;
    buf[5] = set.target_component;
    for (std::size_t i = 0; i < kParamIdLen; ++i) {
        buf[6 + i] = set.param_id[i];
    }
    buf[22] = set.param_type;
    return kParamSetLen;
}

[[nodiscard]] inline bool unpack_param_set(std::span<const std::uint8_t> buf, ParamSet& out) {
    if (buf.size() < kParamSetLen) {
        return false;
    }
    out.param_value = read_f32_le(buf.data() + 0);
    out.target_system = buf[4];
    out.target_component = buf[5];
    for (std::size_t i = 0; i < kParamIdLen; ++i) {
        out.param_id[i] = buf[6 + i];
    }
    out.param_type = buf[22];
    return true;
}

[[nodiscard]] inline std::size_t pack_param_value(const ParamValue& value, std::span<std::uint8_t> buf) {
    if (buf.size() < kParamValueLen) {
        return 0;
    }
    write_f32_le(buf.data() + 0, value.param_value);
    write_u16_le(buf.data() + 4, value.param_count);
    write_u16_le(buf.data() + 6, value.param_index);
    for (std::size_t i = 0; i < kParamIdLen; ++i) {
        buf[8 + i] = value.param_id[i];
    }
    buf[24] = value.param_type;
    return kParamValueLen;
}

[[nodiscard]] inline bool unpack_param_value(std::span<const std::uint8_t> buf, ParamValue& out) {
    if (buf.size() < kParamValueLen) {
        return false;
    }
    out.param_value = read_f32_le(buf.data() + 0);
    out.param_count = read_u16_le(buf.data() + 4);
    out.param_index = read_u16_le(buf.data() + 6);
    for (std::size_t i = 0; i < kParamIdLen; ++i) {
        out.param_id[i] = buf[8 + i];
    }
    out.param_type = buf[24];
    return true;
}

[[nodiscard]] inline bool param_request_list_from_frame(const Frame& frame, ParamRequestList& out) {
    if (frame.msgid != kMsgIdParamRequestList) {
        return false;
    }
    return unpack_param_request_list(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool param_set_from_frame(const Frame& frame, ParamSet& out) {
    if (frame.msgid != kMsgIdParamSet) {
        return false;
    }
    return unpack_param_set(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool param_value_from_frame(const Frame& frame, ParamValue& out) {
    if (frame.msgid != kMsgIdParamValue) {
        return false;
    }
    return unpack_param_value(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool param_store_insert(ParamStore& store, const char* name, float value,
                                             std::uint8_t type) {
    if (name == nullptr || name[0] == '\0' || store.count >= kMaxParams) {
        return false;
    }
    ParamEntry& e = store.entries[store.count];
    std::uint8_t id[kParamIdLen]{};
    copy_name_to_param_id(id, name);
    copy_param_id_to_name(e.name, id);
    e.value = value;
    e.type = type;
    ++store.count;
    return true;
}

[[nodiscard]] inline const ParamEntry* param_store_find(const ParamStore& store, const char* name) {
    if (name == nullptr || name[0] == '\0') {
        return nullptr;
    }
    for (std::uint16_t i = 0; i < store.count; ++i) {
        if (param_name_eq(store.entries[i].name, name)) {
            return &store.entries[i];
        }
    }
    return nullptr;
}

[[nodiscard]] inline ParamEntry* param_store_find(ParamStore& store, const char* name) {
    return const_cast<ParamEntry*>(param_store_find(static_cast<const ParamStore&>(store), name));
}

[[nodiscard]] inline ParamValue make_param_value(const ParamEntry& entry, std::uint16_t count,
                                                 std::uint16_t index) {
    ParamValue v{};
    copy_name_to_param_id(v.param_id, entry.name);
    v.param_value = entry.value;
    v.param_count = count;
    v.param_index = index;
    v.param_type = entry.type;
    return v;
}

// Emit one PARAM_VALUE per injected entry into caller-owned storage
// (preferred over streaming leftover). Returns how many were written.
[[nodiscard]] inline std::size_t emit_param_list(const ParamStore& store, std::span<ParamValue> out) {
    const std::size_t n = store.count;
    const std::size_t emit = n < out.size() ? n : out.size();
    for (std::size_t i = 0; i < emit; ++i) {
        out[i] = make_param_value(store.entries[i], static_cast<std::uint16_t>(n),
                                  static_cast<std::uint16_t>(i));
    }
    return emit;
}

// handle_param_set: find by name, reject NaN/Inf, write value, ack with
// PARAM_VALUE (index -1). Unknown name is ignored (upstream send_param_error
// MAV_PARAM_ERROR_DOES_NOT_EXIST; this slice has no PARAM_ERROR msgid).
[[nodiscard]] inline ParamSetStatus apply_param_set(ParamStore& store, const ParamSet& set,
                                                    ParamValue& ack) {
    if (!std::isfinite(set.param_value)) {
        return ParamSetStatus::kRejected;
    }
    char key[kParamNameCapacity]{};
    copy_param_id_to_name(key, set.param_id);
    ParamEntry* vp = param_store_find(store, key);
    if (vp == nullptr) {
        return ParamSetStatus::kUnknown;
    }
    vp->value = set.param_value;
    ack = make_param_value(*vp, store.count, kParamIndexSetAck);
    return ParamSetStatus::kApplied;
}

}  // namespace fwcpp::gcs
