#pragma once

#include "crazyflie/link/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace cfo {

// CRTP port and channel constants used in this code. Names from the cflib
// reference (crazyflie-lib-python). Deliberately not a complete enumeration —
// only what the code uses lives here.
namespace crtp {

inline constexpr std::uint8_t kPortConsole          = 0;
inline constexpr std::uint8_t kPortParam            = 2;
inline constexpr std::uint8_t kPortLog              = 5;
inline constexpr std::uint8_t kPortGenericSetpoint  = 7;
inline constexpr std::uint8_t kPortHighLevelCmd     = 8;
inline constexpr std::uint8_t kPortSupervisor       = 9;
inline constexpr std::uint8_t kPortPlatform         = 13;
inline constexpr std::uint8_t kPortLinkControl      = 15;

inline constexpr std::uint8_t kChannelConsole         = 0;   // on port 0
inline constexpr std::uint8_t kChannelParamToc        = 0;   // on port 2
inline constexpr std::uint8_t kChannelParamRead       = 1;   // on port 2
inline constexpr std::uint8_t kChannelParamWrite      = 2;   // on port 2
inline constexpr std::uint8_t kChannelLogToc          = 0;   // on port 5
inline constexpr std::uint8_t kChannelLogSettings     = 1;   // on port 5
inline constexpr std::uint8_t kChannelLogData         = 2;   // on port 5
inline constexpr std::uint8_t kChannelGenericSetpoint   = 0;   // on port 7
inline constexpr std::uint8_t kChannelHighLevelCmd      = 0;   // on port 8
inline constexpr std::uint8_t kChannelSupervisorInfo    = 0;   // on port 9 — queries
inline constexpr std::uint8_t kChannelSupervisorCommand = 1;   // on port 9 — commands
inline constexpr std::uint8_t kChannelPlatformCommand   = 0;   // on port 13
inline constexpr std::uint8_t kChannelLinkControl       = 3;   // on port 15

// Generic-commander setpoint type codes (cflib commander_generic.py /
// firmware crtp_commander_generic.c). Only the ones we use are listed.
inline constexpr std::uint8_t kSetpointStop       = 0;
inline constexpr std::uint8_t kSetpointHover      = 5;
inline constexpr std::uint8_t kSetpointNotifyStop = 0xFF;

// Arming command IDs.
//   Modern  (CRTP v12+): SUPERVISOR/ch1, byte CMD_ARM_SYSTEM         = 0x01
//   Legacy  (pre-v12)  : PLATFORM/ch0,   byte PLATFORM_REQUEST_ARMING = 0x01
// Same command byte value, different (port, channel) — a coincidence in the
// cflib constants but we name them separately for clarity.
inline constexpr std::uint8_t kCmdSupervisorArm        = 0x01;
inline constexpr std::uint8_t kCmdPlatformRequestArm   = 0x01;

// Supervisor info-channel: ask for the current state bitfield. The firmware
// echoes the command back with bit 7 set (response flag), followed by the
// little-endian bitfield bytes (cflib supervisor.py / firmware supervisor.c).
inline constexpr std::uint8_t kCmdSupervisorGetStateBitfield = 0x0C;
inline constexpr std::uint8_t kCmdResponseFlag               = 0x80;

// Clears the supervisor's "locked" / "crashed" flags so that subsequent arm
// requests are accepted. Single-byte payload on SUPERVISOR_CH_COMMAND.
// Mirrors cflib's send_crash_recovery_request().
inline constexpr std::uint8_t kCmdSupervisorRecover = 0x02;

// PARAM TOC uses the same V2 command codes as LOG TOC (kCmdTocItemV2,
// kCmdTocInfoV2). The element payload differs: a metadata byte (low
// nibble = type, bit 4 = extended, bit 6 = read-only) followed by the
// null-terminated group + name strings.
inline constexpr std::uint8_t kParamTypeFlagExtended = 0x10;
inline constexpr std::uint8_t kParamTypeFlagReadonly = 0x40;
inline constexpr std::uint8_t kParamTypeMaskCore     = 0x0F;

// PARAM type codes (cflib param.py types map). Different numbering from
// LOG. We only handle uint8_t in the WRITE builder for slice (d) since
// the only param we set is `commander.enHighLevel` (uint8_t).
inline constexpr std::uint8_t kParamTypeUint8  = 0x08;

// High-level commander opcodes (cflib high_level_commander.py).
inline constexpr std::uint8_t kCmdHlcStop      = 0x03;
inline constexpr std::uint8_t kCmdHlcTakeoff2  = 0x07;
inline constexpr std::uint8_t kCmdHlcLand2     = 0x08;
inline constexpr std::uint8_t kCmdHlcGoTo2     = 0x0C;

// LOG/TOC commands (cflib log.py / toc.py)
inline constexpr std::uint8_t kCmdTocItemV2          = 0x02;
inline constexpr std::uint8_t kCmdTocInfoV2          = 0x03;
inline constexpr std::uint8_t kCmdLogStartLogging    = 0x03;
inline constexpr std::uint8_t kCmdLogStopLogging     = 0x04;
inline constexpr std::uint8_t kCmdLogResetLogging    = 0x05;
inline constexpr std::uint8_t kCmdLogCreateBlockV2   = 0x06;

// LOG variable type codes (cflib LogTocElement.types)
inline constexpr std::uint8_t kLogTypeUint8    = 1;
inline constexpr std::uint8_t kLogTypeUint16   = 2;
inline constexpr std::uint8_t kLogTypeUint32   = 3;
inline constexpr std::uint8_t kLogTypeInt8     = 4;
inline constexpr std::uint8_t kLogTypeInt16    = 5;
inline constexpr std::uint8_t kLogTypeInt32    = 6;
inline constexpr std::uint8_t kLogTypeFloat    = 7;
inline constexpr std::uint8_t kLogTypeFP16     = 8;

// Encode a (stored, fetched) type pair as the byte expected by
// CMD_CREATE_BLOCK_V2: high nibble = stored, low nibble = fetched.
constexpr std::uint8_t log_type_byte(std::uint8_t stored, std::uint8_t fetched) {
    return static_cast<std::uint8_t>((stored << 4) | (fetched & 0x0F));
}

} // namespace crtp

// Coarse classification of an inbound packet. Drives the RX-thread switch
// that hands packets to specific decoders. Adding a kind requires adding
// the corresponding decoder.
enum class PacketKind : std::uint8_t {
    Unknown,        // not yet decoded
    Console,        // port 0, ch 0 — firmware console output (ASCII)
    LogData,        // port 5, ch 2 — log block sample (decoded by decode_log_block_sample)
    LogTocReply,    // port 5, ch 0 — TOC info or item (used during setup only)
    LogSettingsAck, // port 5, ch 1 — settings command response
    LinkControl,    // port 15, ch 3 — radio link maintenance, payload not always empty
};

// Decoded firmware console message. The firmware emits text fragments
// (often without trailing newline); a single packet is one fragment. Line
// reassembly is a separate concern not addressed by this slice.
struct ConsoleMessage {
    std::string text;
    std::chrono::system_clock::time_point t;
};

enum class DecodeError : std::uint8_t {
    WrongPort,
    Truncated,
    UnexpectedCmd,
    WrongBlockId,
    WrongSize,
};

// LOG TOC info response (cmd 0x03 on port 5/ch 0).
struct LogTocInfo {
    std::uint16_t count;     // number of TOC entries
    std::uint32_t crc;       // CRC32 of the TOC (used for caching by cflib)
};

// One LOG TOC item — the firmware's record for a single log variable.
struct LogTocItem {
    std::uint16_t index;     // variable id, used in CREATE_BLOCK_V2
    std::uint8_t  type_code; // crtp::kLogType*, low nibble of the wire byte
    std::string   group;     // e.g. "stateEstimate"
    std::string   name;      // e.g. "x"
};

// One PARAM TOC item. Same shape as LogTocItem but the low-nibble type
// space is different (LOG vs PARAM use different numeric codes).
struct ParamTocItem {
    std::uint16_t index;
    std::uint8_t  type_code;   // low nibble of the metadata byte
    bool          read_only;   // bit 6 of the metadata byte
    std::string   group;
    std::string   name;
};

// Reuses LogTocInfo's shape (count + crc) — the wire format is identical.
using ParamTocInfo = LogTocInfo;

// Variable spec for CMD_CREATE_BLOCK_V2 — stored/fetched type byte plus the
// resolved variable id from the TOC.
struct LogVarSpec {
    std::uint8_t  type_byte;
    std::uint16_t var_id;
};

// One decoded sample of our log block (slice 2's specific layout).
// Field order matches the on-wire byte order so the decoder is a straight
// memcpy / FP16 conversion.
struct LogBlockSample {
    std::uint32_t timestamp_ms;  // firmware monotonic clock, 24-bit on wire
    float x, y, z;               // stateEstimate.{x,y,z}        (float32)
    float vbat;                  // pm.vbat                      (FP16 → float32)
    float roll, pitch, yaw;      // stabilizer.{roll,pitch,yaw}  (float32)
};

// Settings ack response (CREATE_BLOCK_V2, START_LOGGING, ...).
struct LogSettingsAck {
    std::uint8_t cmd;
    std::uint8_t block_id;
    std::uint8_t error_code;     // 0 = ok; cflib treats EEXIST as ok for create
};

// Decoded supervisor state — the 11 bits the firmware sends in response to
// CMD_GET_STATE_BITFIELD on port 9 / channel 0. Names match cflib's
// supervisor.py properties.
struct SupervisorState {
    bool can_be_armed{};
    bool is_armed{};
    bool is_auto_armed{};
    bool can_fly{};
    bool is_flying{};
    bool is_tumbled{};
    bool is_locked{};
    bool is_crashed{};
    bool hl_control_active{};
    bool hl_traj_finished{};
    bool hl_control_disabled{};
};

} // namespace cfo
