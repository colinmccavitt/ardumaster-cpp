#pragma once

// Port of libraries/SITL/SIM_GPS_Trimble.h/.cpp. DCOL parser (command 0x64
// APPFILE) enables GSOF output channels; publish() emits GENOUT 0x40 with
// POSITION_TIME / LLH / VELOCITY / PDOP / SIGMA records at configured rates.

#include <cassert>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

inline std::uint16_t sim_htobe16(std::uint16_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return static_cast<std::uint16_t>((v << 8) | (v >> 8));
#else
    return v;
#endif
}
inline std::uint32_t sim_htobe32(std::uint32_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return __builtin_bswap32(v);
#else
    return v;
#endif
}
inline std::uint64_t sim_htobe64(std::uint64_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return __builtin_bswap64(v);
#else
    return v;
#endif
}

#if defined(__GNUC__)
#define FWCPP_SIM_PACKED __attribute__((packed))
#else
#define FWCPP_SIM_PACKED
#endif

class DCOL_Parser {
public:
    bool dcol_parse(char data_in);

    static constexpr std::uint8_t STX = 0x02;
    static constexpr std::uint8_t ETX = 0x03;

    enum class Status : std::uint8_t { OK = 0x00 };
    enum class Command_Response : std::uint8_t { ACK = 0x06, NACK = 0x15 };
    enum class Output_Rate : std::uint8_t { OFF = 0, FREQ_10_HZ = 1, FREQ_50_HZ = 15, FREQ_100_HZ = 16 };
    enum class Packet_Type : std::uint8_t { COMMAND_APPFILE = 0x64 };
    enum class Appfile_Record_Type : std::uint8_t { SERIAL_PORT_BAUD_RATE_FORMAT = 0x02, OUTPUT_MESSAGE = 0x07 };
    enum class Output_Msg_Msg_Type : std::uint8_t { GSOF = 10 };
    enum class Gsof_Msg_Record_Type : std::uint8_t {
        POSITION_TIME = 1,
        LLH = 2,
        VELOCITY_DATA = 8,
        PDOP_INFO = 9,
        POSITION_SIGMA_INFO = 12,
    };

protected:
    static constexpr std::uint8_t MAX_PAYLOAD_SIZE = 255;
    static constexpr std::uint8_t MAX_CHANNEL_NUM = 70;
    Output_Rate channel_rates[MAX_CHANNEL_NUM] = {Output_Rate::OFF};
    std::uint32_t last_publish_ms[MAX_CHANNEL_NUM] {};
    static std::uint32_t RateToPeriodMs(Output_Rate rate);

private:
    enum class Parse_State {
        WAITING_ON_STX,
        WAITING_ON_STATUS,
        WAITING_ON_PACKET_TYPE,
        WAITING_ON_LENGTH,
        WAITING_ON_PACKET_DATA,
        WAITING_ON_CSUM,
        WAITING_ON_ETX,
    };
    bool valid_csum();
    bool parse_payload();
    bool parse_cmd_appfile();
    Status status {Status::OK};
    Parse_State parse_state {Parse_State::WAITING_ON_STX};
    Packet_Type packet_type {Packet_Type::COMMAND_APPFILE};
    std::uint8_t expected_payload_length {0};
    std::uint8_t cur_payload_idx {0};
    std::uint8_t expected_csum {0};
    std::uint8_t appfile_trans_num {0};
    std::uint8_t payload[MAX_PAYLOAD_SIZE] {};
    void reset();
};

class GPS_Trimble : public GPS_Backend, public DCOL_Parser {
public:
    using GPS_Backend::GPS_Backend;
    GPS_Trimble(const GPS_Trimble&) = delete;
    GPS_Trimble& operator=(const GPS_Trimble&) = delete;
    void publish(const GPS_Data* d) override;
    void update_read() override;
    void enable_gsof_channel(Gsof_Msg_Record_Type t, Output_Rate r) {
        channel_rates[static_cast<std::uint8_t>(t)] = r;
    }

private:
    void send_gsof(const std::uint8_t* buf, std::uint16_t size);
    std::uint64_t gsof_pack_double(const double& src);
    std::uint32_t gsof_pack_float(const float& src);
};

inline std::uint32_t DCOL_Parser::RateToPeriodMs(const Output_Rate rate) {
    switch (rate) {
    case Output_Rate::OFF:
        return 0;
    case Output_Rate::FREQ_10_HZ:
        return 100;
    case Output_Rate::FREQ_50_HZ:
        return 20;
    case Output_Rate::FREQ_100_HZ:
        return 10;
    }
    return 0;
}

inline void DCOL_Parser::reset() {
    cur_payload_idx = 0;
    expected_payload_length = 0;
    parse_state = Parse_State::WAITING_ON_STX;
}

inline bool DCOL_Parser::valid_csum() {
    std::uint8_t sum = static_cast<std::uint8_t>(status);
    sum = static_cast<std::uint8_t>(sum + static_cast<std::uint8_t>(packet_type));
    sum = static_cast<std::uint8_t>(sum + expected_payload_length);
    sum = static_cast<std::uint8_t>(sum + crc_sum_of_bytes(payload, expected_payload_length));
    return sum == expected_csum;
}

inline bool DCOL_Parser::parse_cmd_appfile() {
    constexpr std::uint8_t file_control_info_block_sz = 4;
    constexpr std::uint8_t appfile_header_sz = 3;
    constexpr std::uint8_t min_cmd_appfile_sz = file_control_info_block_sz + appfile_header_sz;
    if (expected_payload_length < min_cmd_appfile_sz) {
        return false;
    }
    appfile_trans_num = payload[0];
    constexpr std::uint8_t file_ctrl_idx = appfile_header_sz;
    if (payload[file_ctrl_idx] != 0x03 || payload[file_ctrl_idx + 1] != 0x00 || payload[file_ctrl_idx + 2] != 0x01 ||
        payload[file_ctrl_idx + 3] != 0x00) {
        return false;
    }
    constexpr std::uint8_t app_file_records_idx = appfile_header_sz + file_control_info_block_sz;
    const std::uint8_t record_type = payload[app_file_records_idx];
    if (record_type == static_cast<std::uint8_t>(Appfile_Record_Type::SERIAL_PORT_BAUD_RATE_FORMAT)) {
        return true;
    }
    if (record_type == static_cast<std::uint8_t>(Appfile_Record_Type::OUTPUT_MESSAGE)) {
        if (payload[app_file_records_idx + 2] == static_cast<std::uint8_t>(Output_Msg_Msg_Type::GSOF)) {
            const auto gsof_submessage_type = payload[app_file_records_idx + 6];
            const auto rate = payload[app_file_records_idx + 4];
            if (rate == static_cast<std::uint8_t>(Output_Rate::OFF) ||
                rate == static_cast<std::uint8_t>(Output_Rate::FREQ_10_HZ) ||
                rate == static_cast<std::uint8_t>(Output_Rate::FREQ_50_HZ) ||
                rate == static_cast<std::uint8_t>(Output_Rate::FREQ_100_HZ)) {
                channel_rates[gsof_submessage_type] = static_cast<Output_Rate>(rate);
            }
        }
        return true;
    }
    return false;
}

inline bool DCOL_Parser::parse_payload() {
    if (packet_type == Packet_Type::COMMAND_APPFILE) {
        return parse_cmd_appfile();
    }
    return false;
}

inline bool DCOL_Parser::dcol_parse(const char data_in) {
    bool ret = false;
    switch (parse_state) {
    case Parse_State::WAITING_ON_STX:
        if (static_cast<std::uint8_t>(data_in) == STX) {
            reset();
            parse_state = Parse_State::WAITING_ON_STATUS;
        }
        break;
    case Parse_State::WAITING_ON_STATUS:
        if (static_cast<std::uint8_t>(data_in) != static_cast<std::uint8_t>(Status::OK)) {
            reset();
        } else {
            status = static_cast<Status>(data_in);
            parse_state = Parse_State::WAITING_ON_PACKET_TYPE;
        }
        break;
    case Parse_State::WAITING_ON_PACKET_TYPE:
        packet_type = static_cast<Packet_Type>(data_in);
        parse_state = Parse_State::WAITING_ON_LENGTH;
        break;
    case Parse_State::WAITING_ON_LENGTH:
        expected_payload_length = static_cast<std::uint8_t>(data_in);
        parse_state = Parse_State::WAITING_ON_PACKET_DATA;
        break;
    case Parse_State::WAITING_ON_PACKET_DATA:
        payload[cur_payload_idx] = static_cast<std::uint8_t>(data_in);
        if (++cur_payload_idx == expected_payload_length) {
            parse_state = Parse_State::WAITING_ON_CSUM;
        }
        break;
    case Parse_State::WAITING_ON_CSUM:
        expected_csum = static_cast<std::uint8_t>(data_in);
        parse_state = Parse_State::WAITING_ON_ETX;
        break;
    case Parse_State::WAITING_ON_ETX:
        if (static_cast<std::uint8_t>(data_in) != ETX) {
            reset();
            break;
        }
        if (valid_csum()) {
            ret = parse_payload();
        }
        reset();
        break;
    }
    return ret;
}

inline std::uint64_t GPS_Trimble::gsof_pack_double(const double& src) {
    std::uint64_t dst;
    std::memcpy(&dst, &src, sizeof(dst));
    return sim_htobe64(dst);
}
inline std::uint32_t GPS_Trimble::gsof_pack_float(const float& src) {
    std::uint32_t dst;
    std::memcpy(&dst, &src, sizeof(dst));
    return sim_htobe32(dst);
}

inline void GPS_Trimble::send_gsof(const std::uint8_t* buf, const std::uint16_t size) {
    const std::uint8_t STATUS = 0xa8;
    const std::uint8_t PACKET_TYPE = 0x40;
    static std::uint8_t TRANSMISSION_NUMBER = 0;
    constexpr std::uint8_t PAGE_INDEX = 0;
    constexpr std::uint8_t MAX_PAGE_INDEX = 0;
    const std::uint8_t gsof_header[3] = {TRANSMISSION_NUMBER, PAGE_INDEX, MAX_PAGE_INDEX};
    ++TRANSMISSION_NUMBER;
    const std::uint8_t length = static_cast<std::uint8_t>(size + sizeof(gsof_header));
    const std::uint8_t dcol_header[4] {STX, STATUS, PACKET_TYPE, length};
    std::uint8_t csum = static_cast<std::uint8_t>(STATUS + PACKET_TYPE + length);
    for (std::size_t i = 0; i < sizeof(gsof_header); i++) {
        csum = static_cast<std::uint8_t>(csum + gsof_header[i]);
    }
    for (std::uint16_t i = 0; i < size; i++) {
        csum = static_cast<std::uint8_t>(csum + buf[i]);
    }
    const std::uint8_t dcol_trailer[2] = {csum, ETX};
    write_to_autopilot(reinterpret_cast<const char*>(dcol_header), sizeof(dcol_header));
    write_to_autopilot(reinterpret_cast<const char*>(gsof_header), sizeof(gsof_header));
    write_to_autopilot(reinterpret_cast<const char*>(buf), size);
    write_to_autopilot(reinterpret_cast<const char*>(dcol_trailer), sizeof(dcol_trailer));
}

inline void GPS_Trimble::publish(const GPS_Data* d) {
    const std::uint32_t now = front.now_ms();
    std::uint8_t buf[MAX_PAYLOAD_SIZE] {};
    std::uint8_t payload_sz = 0;
    std::uint8_t offset = 0;
    auto due = [&](Gsof_Msg_Record_Type t) {
        const auto last_time = last_publish_ms[static_cast<std::uint8_t>(t)];
        const auto desired_rate = channel_rates[static_cast<std::uint8_t>(t)];
        return desired_rate != Output_Rate::OFF && (now - last_time > RateToPeriodMs(desired_rate));
    };

    if (due(Gsof_Msg_Record_Type::POSITION_TIME)) {
        constexpr std::uint8_t GSOF_POS_TIME_LEN = 0x0A;
        const std::uint8_t bootcount = 17;
        enum class POS_FLAGS_1 : std::uint8_t {
            NEW_POSITION = 1U << 0,
            CLOCK_FIX_CALULATED = 1U << 1,
            HORIZ_FROM_THIS_POS = 1U << 2,
            HEIGHT_FROM_THIS_POS = 1U << 3,
            RESERVED_4 = 1U << 4,
            LEAST_SQ_POSITION = 1U << 5,
            RESERVED_6 = 1U << 6,
            POSITION_L1_PSEUDORANGES = 1U << 7
        };
        const std::uint8_t pos_flags_1 = static_cast<std::uint8_t>(POS_FLAGS_1::NEW_POSITION) |
                                         static_cast<std::uint8_t>(POS_FLAGS_1::CLOCK_FIX_CALULATED) |
                                         static_cast<std::uint8_t>(POS_FLAGS_1::HORIZ_FROM_THIS_POS) |
                                         static_cast<std::uint8_t>(POS_FLAGS_1::HEIGHT_FROM_THIS_POS) |
                                         static_cast<std::uint8_t>(POS_FLAGS_1::RESERVED_4) |
                                         static_cast<std::uint8_t>(POS_FLAGS_1::LEAST_SQ_POSITION) |
                                         static_cast<std::uint8_t>(POS_FLAGS_1::POSITION_L1_PSEUDORANGES);
        enum class POS_FLAGS_2 : std::uint8_t {
            DIFFERENTIAL_POS = 1U << 0,
            DIFFERENTIAL_POS_PHASE_RTK = 1U << 1,
            POSITION_METHOD_FIXED_PHASE = 1U << 2,
            OMNISTAR_ACTIVE = 1U << 3,
            DETERMINED_WITH_STATIC_CONSTRAINT = 1U << 4,
        };
        std::uint8_t pos_flags_2 = 0;
        if (d->have_lock) {
            pos_flags_2 = static_cast<std::uint8_t>(POS_FLAGS_2::DIFFERENTIAL_POS) |
                          static_cast<std::uint8_t>(POS_FLAGS_2::DIFFERENTIAL_POS_PHASE_RTK) |
                          static_cast<std::uint8_t>(POS_FLAGS_2::POSITION_METHOD_FIXED_PHASE) |
                          static_cast<std::uint8_t>(POS_FLAGS_2::OMNISTAR_ACTIVE) |
                          static_cast<std::uint8_t>(POS_FLAGS_2::DETERMINED_WITH_STATIC_CONSTRAINT);
        }
        const auto gps_tow = gps_time();
        struct FWCPP_SIM_PACKED gsof_pos_time {
            std::uint8_t OUTPUT_RECORD_TYPE;
            std::uint8_t RECORD_LEN;
            std::uint32_t time_week_ms;
            std::uint16_t time_week;
            std::uint8_t num_sats;
            std::uint8_t pos_flags_1;
            std::uint8_t pos_flags_2;
            std::uint8_t initialized_num;
        } pos_time {static_cast<std::uint8_t>(Gsof_Msg_Record_Type::POSITION_TIME),
                    GSOF_POS_TIME_LEN,
                    sim_htobe32(gps_tow.ms),
                    sim_htobe16(gps_tow.week),
                    d->have_lock ? d->num_sats : std::uint8_t(3),
                    pos_flags_1,
                    pos_flags_2,
                    bootcount};
        payload_sz = static_cast<std::uint8_t>(payload_sz + sizeof(pos_time));
        std::memcpy(&buf[offset], &pos_time, sizeof(pos_time));
        offset = static_cast<std::uint8_t>(offset + sizeof(pos_time));
        last_publish_ms[static_cast<std::uint8_t>(Gsof_Msg_Record_Type::POSITION_TIME)] = now;
    }

    if (due(Gsof_Msg_Record_Type::LLH)) {
        constexpr std::uint8_t GSOF_POS_LEN = 0x18;
        constexpr double DEG_TO_RAD_DOUBLE = 0.017453292519943295769;
        struct FWCPP_SIM_PACKED gsof_pos {
            std::uint8_t OUTPUT_RECORD_TYPE;
            std::uint8_t RECORD_LEN;
            std::uint64_t lat;
            std::uint64_t lng;
            std::uint64_t alt;
        } pos {static_cast<std::uint8_t>(Gsof_Msg_Record_Type::LLH),
               GSOF_POS_LEN,
               gsof_pack_double(d->latitude * DEG_TO_RAD_DOUBLE),
               gsof_pack_double(d->longitude * DEG_TO_RAD_DOUBLE),
               gsof_pack_double(static_cast<double>(d->altitude))};
        payload_sz = static_cast<std::uint8_t>(payload_sz + sizeof(pos));
        std::memcpy(&buf[offset], &pos, sizeof(pos));
        offset = static_cast<std::uint8_t>(offset + sizeof(pos));
        last_publish_ms[static_cast<std::uint8_t>(Gsof_Msg_Record_Type::LLH)] = now;
    }

    if (due(Gsof_Msg_Record_Type::VELOCITY_DATA)) {
        constexpr std::uint8_t GSOF_VEL_LEN = 0x0D;
        enum class VEL_FIELDS : std::uint8_t {
            VALID = 1U << 0,
            CONSECUTIVE_MEASUREMENTS = 1U << 1,
            HEADING_VALID = 1U << 2,
        };
        std::uint8_t vel_flags = 0;
        if (d->have_lock) {
            vel_flags = static_cast<std::uint8_t>(VEL_FIELDS::VALID) |
                        static_cast<std::uint8_t>(VEL_FIELDS::CONSECUTIVE_MEASUREMENTS) |
                        static_cast<std::uint8_t>(VEL_FIELDS::HEADING_VALID);
        }
        struct FWCPP_SIM_PACKED gsof_vel {
            std::uint8_t OUTPUT_RECORD_TYPE;
            std::uint8_t RECORD_LEN;
            std::uint8_t flags;
            std::uint32_t horiz_m_p_s;
            std::uint32_t heading_rad;
            std::uint32_t vertical_m_p_s;
        } vel {static_cast<std::uint8_t>(Gsof_Msg_Record_Type::VELOCITY_DATA),
               GSOF_VEL_LEN,
               vel_flags,
               gsof_pack_float(d->speed_2d()),
               gsof_pack_float(d->ground_track_rad()),
               gsof_pack_float(static_cast<float>(d->speedD))};
        payload_sz = static_cast<std::uint8_t>(payload_sz + sizeof(vel));
        std::memcpy(&buf[offset], &vel, sizeof(vel));
        offset = static_cast<std::uint8_t>(offset + sizeof(vel));
        last_publish_ms[static_cast<std::uint8_t>(Gsof_Msg_Record_Type::VELOCITY_DATA)] = now;
    }

    if (due(Gsof_Msg_Record_Type::PDOP_INFO)) {
        constexpr std::uint8_t GSOF_DOP_LEN = 0x10;
        struct FWCPP_SIM_PACKED gsof_dop {
            std::uint8_t OUTPUT_RECORD_TYPE {static_cast<std::uint8_t>(Gsof_Msg_Record_Type::PDOP_INFO)};
            std::uint8_t RECORD_LEN {GSOF_DOP_LEN};
            std::uint32_t pdop {sim_htobe32(1)};
            std::uint32_t hdop {sim_htobe32(1)};
            std::uint32_t vdop {sim_htobe32(1)};
            std::uint32_t tdop {sim_htobe32(1)};
        } dop {};
        payload_sz = static_cast<std::uint8_t>(payload_sz + sizeof(dop));
        std::memcpy(&buf[offset], &dop, sizeof(dop));
        offset = static_cast<std::uint8_t>(offset + sizeof(dop));
        last_publish_ms[static_cast<std::uint8_t>(Gsof_Msg_Record_Type::PDOP_INFO)] = now;
    }

    if (due(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO)) {
        constexpr std::uint8_t GSOF_POS_SIGMA_LEN = 0x26;
        struct FWCPP_SIM_PACKED gsof_pos_sigma {
            std::uint8_t OUTPUT_RECORD_TYPE {static_cast<std::uint8_t>(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO)};
            std::uint8_t RECORD_LEN {GSOF_POS_SIGMA_LEN};
            std::uint32_t pos_rms {sim_htobe32(0)};
            std::uint32_t sigma_e {sim_htobe32(0)};
            std::uint32_t sigma_n {sim_htobe32(0)};
            std::uint32_t cov_en {sim_htobe32(0)};
            std::uint32_t sigma_up {sim_htobe32(0)};
            std::uint32_t semi_major_axis {sim_htobe32(0)};
            std::uint32_t semi_minor_axis {sim_htobe32(0)};
            std::uint32_t orientation {sim_htobe32(0)};
            std::uint32_t unit_variance {sim_htobe32(0)};
            std::uint16_t n_epocs {sim_htobe16(1)};
        } pos_sigma {};
        payload_sz = static_cast<std::uint8_t>(payload_sz + sizeof(pos_sigma));
        std::memcpy(&buf[offset], &pos_sigma, sizeof(pos_sigma));
        offset = static_cast<std::uint8_t>(offset + sizeof(pos_sigma));
        last_publish_ms[static_cast<std::uint8_t>(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO)] = now;
    }

    (void)offset;
    if (payload_sz > 0) {
        send_gsof(buf, payload_sz);
    }
}

inline void GPS_Trimble::update_read() {
    char c[MAX_PAYLOAD_SIZE];
    const auto n_read = read_from_autopilot(c, MAX_PAYLOAD_SIZE);
    if (n_read > 0) {
        for (ssize_t i = 0; i < n_read; i++) {
            if (dcol_parse(c[i])) {
                constexpr std::uint8_t response[1] = {static_cast<std::uint8_t>(Command_Response::ACK)};
                write_to_autopilot(reinterpret_cast<const char*>(response), sizeof(response));
            }
        }
    }
}

#undef FWCPP_SIM_PACKED

}  // namespace fwcpp::sim
