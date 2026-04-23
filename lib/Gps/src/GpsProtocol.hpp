#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <algorithm>

namespace Gps {

enum DeviceVersion {
  GPS_UNKNOWN,
  GPS_M8,
  GPS_M9,
  GPS_F9,
};

constexpr uint8_t UBX_SYNC0 = 0xB5;
constexpr uint8_t UBX_SYNC1 = 0x62;
constexpr uint16_t UBX_SYNC = UBX_SYNC1 << 8 | UBX_SYNC0;

enum MsgClass: uint8_t
{
  UBX_ACK  = 0x05,
  UBX_CFG  = 0x06,
  UBX_MON  = 0x0A,
  UBX_NAV  = 0x01,
  UBX_HNR  = 0xB5,
  NMEA_MSG = 0xF0,
};

enum MsgId: uint16_t
{
  UBX_ACK_NAK      = 0x00 << 8 | UBX_ACK,  // Message not acknowledged
  UBX_ACK_ACK      = 0x01 << 8 | UBX_ACK,  // Message acknowledged

  UBX_MON_VER      = 0x04 << 8 | UBX_MON,  // Receiver and software version

  UBX_CFG_CFG      = 0x09 << 8 | UBX_CFG,  // Clear, save and load configurations
  UBX_CFG_DAT      = 0x06 << 8 | UBX_CFG,  // Set/Set datum
  UBX_CFG_PRT      = 0x00 << 8 | UBX_CFG,  // Port configuration for UART ports
  UBX_CFG_MSG      = 0x01 << 8 | UBX_CFG,  // Set message rate(s)
  UBX_CFG_INF      = 0x02 << 8 | UBX_CFG,  // Information message configuration
  UBX_CFG_RST      = 0x04 << 8 | UBX_CFG,  // Reset receiver / Clear backup data
  UBX_CFG_RATE     = 0x08 << 8 | UBX_CFG,  // Navigation/measurement rate settings
  UBX_CFG_SBAS     = 0x16 << 8 | UBX_CFG,  // SBAS configuration
  UBX_CFG_NAV5     = 0x24 << 8 | UBX_CFG,  // Navigation engine settings
  UBX_CFG_GNSS     = 0x3E << 8 | UBX_CFG,  // GNSS system configuration (deprecated >PROTVER 23.01)
  UBX_CFG_VALSET   = 0x8A << 8 | UBX_CFG,  // Configuration input (Generation 9+, replaces legacy CFG-* messages)
  UBX_CFG_VALGET   = 0x8B << 8 | UBX_CFG,  // Configuration output (Generation 9+, replaces legacy CFG-* messages)

  UBX_NAV_HPOSECEF = 0x13 << 8 | UBX_NAV,  // High precision position solution in ECEF (28 Bytes)
  UBX_NAV_HPOSLLH  = 0x14 << 8 | UBX_NAV,  // High precision geodetic position solution (36 Bytes)
  UBX_NAV_POSECEF  = 0x01 << 8 | UBX_NAV,  // Position solution in ECEF (20 Bytes)
  UBX_NAV_POSLLH   = 0x02 << 8 | UBX_NAV,  // Geodetic position solution (28 Bytes)
  UBX_NAV_PVT      = 0x07 << 8 | UBX_NAV,  // Navigation position velocity time solution (92 Bytes)
  UBX_NAV_SOL      = 0x06 << 8 | UBX_NAV,  // Navigation solution information (52 Bytes)
  UBX_NAV_STATUS   = 0x03 << 8 | UBX_NAV,  // Receiver navigation status (16 Bytes)
  UBX_NAV_SVINFO   = 0x30 << 8 | UBX_NAV,  // Space vehicle information (8 + 12xNumCh Bytes)
  UBX_NAV_DOP      = 0x04 << 8 | UBX_NAV,  // Dilution of precision (18 Bytes)
  UBX_NAV_SAT      = 0x35 << 8 | UBX_NAV,  // Satellite information (8 + 12xNumSats Bytes)
  UBX_NAV_TIMEUTC  = 0x21 << 8 | UBX_NAV,  // UTC time solution (20 Bytes)
  UBX_NAV_VELNED   = 0x12 << 8 | UBX_NAV,  // Velocity solution in NED frame (36 Bytes)

  UBX_HNR_PVT      = 0x00 << 8 | UBX_HNR,  // High rate output of PVT solution (72 Bytes)

  NMEA_MSG_GGA     = 0x00 << 8 | NMEA_MSG, // Global positioning system fix data
  NMEA_MSG_GLL     = 0x01 << 8 | NMEA_MSG, // Latitude and longitude, with time of position fix and status
  NMEA_MSG_GSA     = 0x02 << 8 | NMEA_MSG, // GNSS DOP and active satellites
  NMEA_MSG_GSV     = 0x03 << 8 | NMEA_MSG, // GNSS satellites in view
  NMEA_MSG_RMC     = 0x04 << 8 | NMEA_MSG, // Recommended minimum data
  NMEA_MSG_VTG     = 0x05 << 8 | NMEA_MSG, // Course over ground and ground speed
};

inline uint16_t ubxChecksum(uint16_t init, uint8_t v)
{
  uint8_t a = ((init & 0xff) + v) & 0xff;
  uint8_t b = ((init >> 8) + a) & 0xff;
  return b << 8 | a;
}

inline uint16_t ubxChecksum(uint16_t init, const uint8_t* it, size_t len)
{
  while(len--)
  {
    init = ubxChecksum(init, *it++);
  }
  return init;
}

enum UbxState: uint8_t
{
  UBX_STATE_IDLE,
  UBX_STATE_SYNC,
  UBX_STATE_CLASS,
  UBX_STATE_ID,
  UBX_STATE_LEN0,
  UBX_STATE_LEN1,
  UBX_STATE_PAYLOAD,
  UBX_STATE_CRC0,
  UBX_STATE_CRC1,
  UBX_STATE_READY,
};

class UbxMessage
{
public:
  UbxMessage() = default;
  UbxMessage(const UbxMessage& m) = default;
  UbxMessage& operator=(const UbxMessage& m) = default;

  UbxMessage(uint8_t mId): msgId(mId), length(0), written(0) {}

  uint16_t msgId = 0;
  uint16_t length = 0;
  uint8_t payload[512] = {0};
  uint16_t crc = 0;

  UbxState status{UBX_STATE_IDLE};
  uint16_t written = 0;

  uint16_t checksum() const
  {
    uint16_t c = 0;
    c = ubxChecksum(c, msgId & 0xff);
    c = ubxChecksum(c, msgId >> 8);
    c = ubxChecksum(c, length & 0xff);
    c = ubxChecksum(c, length >> 8);
    for(size_t i = 0; i < length; i++) {
      c = ubxChecksum(c, payload[i]);
    }
    return c;
  }

  template<typename Target>
  const Target* getAs() const
  {
    return reinterpret_cast<const Target*>(payload);
  }

  bool isReady() const { return status == UBX_STATE_READY; }
  bool isIdle() const { return status == UBX_STATE_IDLE; }

  bool isResponse(MsgId id) const { return msgId == id; }

  bool isAck() const { return msgId == UBX_ACK_ACK; }
  bool isAck(MsgId id) const { return isAck() && id == (payload[1] << 8 | payload[0]); }

  bool isNak() const { return msgId == UBX_ACK_NAK; }
  bool isNak(MsgId id) const { return isNak() && id == (payload[1] << 8 | payload[0]); }
};

class UbxRequest
{
public:
  UbxRequest(uint16_t msgId): msgId(msgId), length(0) {}

  UbxRequest() = delete;
  UbxRequest(const UbxRequest& m) = delete;
  UbxRequest& operator=(const UbxRequest& m) = delete;
  UbxRequest(UbxRequest&& m) = delete;
  UbxRequest& operator=(UbxRequest&& m) = delete;

  size_t write(const uint8_t * buff, size_t len)
  {
    if(length + len > sizeof(payload) - 2) return 0;
    std::copy(buff, buff + len, payload + length);
    length += len;
    return len;
  }

  template<typename T>
  size_t write(const T& data)
  {
    return write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
  }

  uint16_t msgId;
  uint16_t length = 0;
  uint8_t payload[72] = {0};
};

// -----------------------------------------------------------------

enum NmeaState: uint8_t
{
  NMEA_STATE_IDLE,
  NMEA_STATE_PAYLOAD,
  NMEA_STATE_END,
  NMEA_STATE_READY,
};

class NmeaMessage
{
public:
  char payload[512] = {0};

  size_t length = 0;
  NmeaState status{NMEA_STATE_IDLE};

  bool isReady() const { return status == NMEA_STATE_READY; }
  bool isIdle() const { return status == NMEA_STATE_IDLE; }
};

// --------------------------------------------------------------

class UbxMonVer
{
public:
  static constexpr MsgId ID = UBX_MON_VER;
} __attribute__((packed));

/**
 * Used to get message rate
 * @deprecated UBX-CFG-MSG is deprecated in favor of UBX-CFG-VALSET with CFG-MSGOUT-* keys
 * kept for older firmware versions
 */
class UbxCfgMsg2
{
public:
  static constexpr MsgId ID = UBX_CFG_MSG;
  uint16_t msgId;
} __attribute__((packed));

/**
 * Used to enable UBX and disable NMEA  messages
 * @deprecated UBX-CFG-MSG is deprecated in favor of UBX-CFG-VALSET with CFG-MSGOUT-* keys
 * kept for older firmware versions
 */
class UbxCfgMsg3
{
public:
  static constexpr MsgId ID = UBX_CFG_MSG;
  uint16_t msgId;
  uint8_t rate; // message output rate (0=disables, 1=every navigation solution, 2=every 2nd solution etc.)
} __attribute__((packed));

constexpr uint32_t CFG_MSGOUT_NMEA_GGA_UART1 = 0x209100bb; // NMEA GGA message output rate
constexpr uint32_t CFG_MSGOUT_NMEA_GLL_UART1 = 0x209100ca; // NMEA GLL message output rate
constexpr uint32_t CFG_MSGOUT_NMEA_GSA_UART1 = 0x209100c0; // NMEA GSA message output rate
constexpr uint32_t CFG_MSGOUT_NMEA_GSV_UART1 = 0x209100c5; // NMEA GSV message output rate
constexpr uint32_t CFG_MSGOUT_NMEA_RMC_UART1 = 0x209100e8; // NMEA RMC message output rate
constexpr uint32_t CFG_MSGOUT_NMEA_VTG_UART1 = 0x209100b1; // NMEA VTG message output rate
constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007; // UBX-NAV-PVT message output rate
constexpr uint32_t CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016; // UBX-NAV-SAT message output rate

/**
 * @deprecated UBX-CFG-MSG is deprecated in favor of UBX-CFG-VALSET with CFG-MSGOUT-* keys
 * kept for older firmware versions
 */
class UbxCfgMsg8
{
public:
  static constexpr MsgId ID = UBX_CFG_MSG;
  uint16_t msgId;
  uint8_t channels[6];
} __attribute__((packed));

/**
 * Used to interface interface baud rate
 * @deprecated UBX-CFG-PRT is deprecated in favor of UBX-CFG-VALSET with CFG-UART1-* keys
 * kept for older firmware versions
 */
class UbxCfgPrt1
{
public:
  static constexpr MsgId ID = UBX_CFG_PRT;
  uint8_t portId;
} __attribute__((packed));

/**
 * Used to set interface baud rate
 * @deprecated UBX-CFG-PRT is deprecated in favor of UBX-CFG-VALSET with 
 * CFG-UART1-* keys
 * kept for older firmware versions
 */
class UbxCfgPrt20
{
public:
  static constexpr MsgId ID = UBX_CFG_PRT;
  uint8_t portId;
  uint8_t resered1;
  uint16_t txReady;
  uint32_t mode;
  uint32_t baudRate;
  uint16_t inProtoMask;
  uint16_t outProtoMask;
  uint16_t flags;
  uint16_t resered2;
} __attribute__((packed));

constexpr uint32_t CFG_UART1_BAUDRATE = 0x40520001; // (?->target) UART1 baud rate (e.g. 9600, 115200)
constexpr uint32_t CFG_UART1_STOPBITS = 0x20520002; // (1) UART1 stop bits (0=half, 1=1, 2=1.5, 3=2)
constexpr uint32_t CFG_UART1_DATABITS = 0x20520003; // (8) UART1 data bits (0=8bits, 1=7bits)
constexpr uint32_t CFG_UART1_PARITY = 0x20520004; // (0) UART1 parity (0=none, 1=odd, 2=even)
constexpr uint32_t CFG_UART1_ENABLED = 0x10520005; // (1) UART1 enabled (0=disabled, 1=enabled)
constexpr uint32_t CFG_UART1_INPROT_UBX = 0x10730001; // (1) UART1 input protocol ubx
constexpr uint32_t CFG_UART1_INPROT_NMEA = 0x10730002; // (1) UART1 input protocol nmea
constexpr uint32_t CFG_UART1_OUTPROT_UBX = 0x10740001; // (1) UART1 output protocol ubx
constexpr uint32_t CFG_UART1_OUTPROT_NMEA = 0x10740002; // (1) UART1 output protocol nmea

/**
 * Used to set measurement, navigation rate and time reference
 * @deprecated UBX-CFG-RATE is deprecated in favor of UBX-CFG-VALSET with
 * CFG_RATE-MEAS, CFG-RATE-NAV and CFG-RATE-TIMEREF keys
 * kept for older firmware versions
 */
class UbxCfgRate6
{
public:
  static constexpr MsgId ID = UBX_CFG_RATE;
  uint16_t measRate; // in ms
  uint16_t navRate;  // ratio to measRate
  uint16_t timeRef;  // 0-utc, 1-gps, 2-glonass (18+), 3-BeiDou (18+), 4-Galileo (18+), 5-NavIC (29+)
} __attribute__((packed));

constexpr uint32_t CFG_RATE_MEAS = 0x30210001; // (1000->mRate) Measurement rate in ms
constexpr uint32_t CFG_RATE_NAV = 0x30210002;  // (1) Navigation rate (number of measurement cycles)
constexpr uint32_t CFG_RATE_TIMEREF = 0x20210003; // (1->0) Time reference (0-utc, 1-gps, 2-glonass, 3-BeiDou (18+), 4-Galileo (18+), 5-NavIC (29+))

/**
 * Used to set SBAS parameters
 * @deprecated UBX-CFG-SBAS is deprecated in favor of UBX-CFG-VALSET with CFG-SBAS-* keys
 * enablement should be done via CFG-SIGNAL-SBAS_ENA key
 * kept for older firmware versions
 */
class UbxCfgSbas8
{
public:
  static constexpr MsgId ID = UBX_CFG_SBAS;
  uint8_t mode; // SBAS mode flags
  uint8_t usage;  // SBAS usage flags
  uint8_t maxSbas;  // Maximum number of SBAS prioritized tracking channels (valid range: 0 - 3) to use
  uint8_t scanmode2; // Continuation of scanmode bitmask
  uint32_t scanmode1; // Which SBAS PRN numbers to search for (bitmask). If all bits are set to zero, auto-scan (i.e. allvalid PRNs) are searched. Every bit corresponds to a PRN number
} __attribute__((packed));

constexpr uint32_t CFG_SBAS_TESTMODE = 0x10360002; // SBAS test mode
constexpr uint32_t CFG_SBAS_RANGING = 0x10360003; // SBAS ranging source
constexpr uint32_t CFG_SBAS_DIFFCORR = 0x10360004; // SBAS differential corrections
constexpr uint32_t CFG_SBAS_INTEGRITY = 0x10360005; // SBAS integrity information
constexpr uint32_t CFG_SBAS_PRNSCANMASK = 0x50360006; // (0x72bc8 -> 0) Which SBAS PRN numbers to search for (bitmask). If all bits are set to zero, auto-scan (i.e. allvalid PRNs) are searched. Every bit corresponds to a PRN number

struct UbxCfgGnssBlock
{
  uint8_t gnssId;
  uint8_t resTrkCh;
  uint8_t maxTrkCh;
  uint8_t reserved1;
  uint8_t flagsEnable;  // bit 0: enable GNSS system
  uint8_t flagsReserved;
  uint8_t sigCfgMask;   // signal config: 0x01=L1, 0x20=L1+L5 (dual-band)
  uint8_t flagsHigh;
} __attribute__((packed));

/**
 * Used to set enabled GNSS systems and signals
 * @deprecated UBX-CFG-GNSS is deprecated in favor of UBX-CFG-VALSET with CFG-SIGNAL-* keys
 * kept for older firmware versions
 */
class UbxCfgGnss7
{
public:
  static constexpr MsgId ID = UBX_CFG_GNSS;
  uint8_t msgVer;
  uint8_t numTrkChHw;
  uint8_t numTrkChUse;
  uint8_t numConfigBlocks;
  UbxCfgGnssBlock blocks[7];
} __attribute__((packed));

class UbxCfgGnssHeader
{
public:
  uint8_t msgVer;
  uint8_t numTrkChHw;
  uint8_t numTrkChUse;
  uint8_t numConfigBlocks;
} __attribute__((packed));

// CFG-SIGNAL key IDs for UBX-CFG-VALSET (PROTVER > 27.00, u-blox Interface Description UBX-23001092)
constexpr uint32_t CFG_SIGNAL_GPS_ENA  = 0x10310001; // GPS enable
constexpr uint32_t CFG_SIGNAL_GPS_L5   = 0x10310004; // GPS L5 signal enable (M10 dual-band)
constexpr uint32_t CFG_SIGNAL_SBAS_ENA = 0x10310020; // SBAS enable
constexpr uint32_t CFG_SIGNAL_GAL_ENA  = 0x10310021; // Galileo enable
constexpr uint32_t CFG_SIGNAL_BDS_ENA  = 0x10310022; // BeiDou enable
constexpr uint32_t CFG_SIGNAL_QZSS_ENA = 0x10310024; // QZSS enable
constexpr uint32_t CFG_SIGNAL_GLO_ENA  = 0x10310025; // GLONASS enable
constexpr uint32_t CFG_SIGNAL_BDS_B2A  = 0x10310028; // BeiDou B2a signal enable (M10 dual-band)

struct UbxCfgValsetKV
{
  uint32_t key;
  uint8_t  value;
} __attribute__((packed));

template<size_t Size>
class UbxCfgValset
{
public:
  static constexpr MsgId ID = UBX_CFG_VALSET;
  uint8_t version;      // 0
  uint8_t layers;       // 0x01 = RAM only
  uint8_t reserved[2];
  UbxCfgValsetKV kv[Size];
} __attribute__((packed));

template<uint32_t K, typename T>
struct UbxCfgValsetItem
{
  explicit constexpr UbxCfgValsetItem(T v): key(K), value(v)
  {
    constexpr size_t size = (K >> 28) & 0x07u;
    if constexpr (size == 1 || size == 2) {
      static_assert((sizeof(T)) == 1, "Invalid value size for key, expected byte");
    } else if constexpr (size == 3) {
      static_assert((sizeof(T)) == 2, "Invalid value size for key, expected word");
    } else if constexpr (size == 4) {
      static_assert((sizeof(T)) == 4, "Invalid value size for key, expected dword");
    } else if constexpr (size == 5) {
      static_assert((sizeof(T)) == 8, "Invalid value size for key, expected qword");
    }
  }

  uint32_t key;
  T  value;
} __attribute__((packed));

class UbxCfgValsetHeader
{
public:
  uint8_t version = 0;      // 0
  uint8_t layers = 0x01;    // 0x01 = RAM only
  uint16_t position = 0;    //
} __attribute__((packed));

/**
 * Used to set enabled NAV5 parameters
 * @deprecated UBX-CFG-NAV5 is deprecated in favor of UBX-CFG-VALSET with CFG-NAVSPG-* keys
 * kept for older firmware versions
 */
class UbxCfgNav5
{
public:
  static constexpr MsgId ID = UBX_CFG_NAV5;
  union {                        // Parameters bitmask. Only the masked parameters will be applied.
    uint16_t value;
    struct {
      uint8_t dyn: 1;            // Apply dynamic model settings
      uint8_t minEl: 1;          // Apply minimum elevation settings
      uint8_t posFixMode: 1;     // Apply fix mode settings
      uint8_t drLim: 1;          // Reserved
      uint8_t posMask: 1;        // Apply position mask settings
      uint8_t timeMask: 1;       // Apply time mask settings
      uint8_t staticHoldMask: 1; // Apply static hold settings
      uint8_t dgpsMask: 1;       // Apply DGPS settings
      uint8_t cnoThreshold: 1;   // Apply CNO threshold settings (cnoThresh, cnoThreshNumSVs)
      uint8_t reserved: 1;
      uint8_t utc: 1;            // Apply UTC settings
    };
  } mask;
  uint8_t dynModel;           // Dynamic model, 0:portable, 2:stationary, 3:pedestrian, 4:automotive, 5:sea, 6:airbone<1g, 7:airbone<2g, 8: airbone<4g, 9:wrist watch, 10:motorbike, 11: lawnmower, 12: electric scooter
  uint8_t fixMode;            // Position fixing mode, 1: 2D only, 2: 3D only, 3: auto 2D/3D
  int32_t fixedAlt;           // Fixed altitude (mean sea level) for 2D fix mode, [m * 0.01]
  uint32_t fixedAltVar;       // Fixed altitude variance for 2D mode, [m^2 * 0.0001]
  int8_t minElev;             // Minimum elevation for a GNSS satellite to be used in NAV, [deg]
  uint8_t drLimit;            // reserved [s]
  uint16_t pDOP;              // Position DOP mask to use * 0.1
  uint16_t tDOP;              // Time DOP mask to use * 0.1
  uint16_t pAcc;              // Position accuracy mask [m]
  uint16_t tAcc;              // Time accuracy mask [m]
  uint8_t staticHoldThresh;   // Static hold threshold [cm/s]
  uint8_t dgnssTimeout;       // DGNSS timeout [s]
  uint8_t cnoThreshNumSVs;    // Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted
  uint8_t cnoThresh;          // C/N0 threshold for deciding whether to attempt a fix
  uint8_t reserved0[2];
  uint16_t staticHoldMaxDist; // Static hold distance threshold
  uint8_t utcStandard;        // UTC standard to be used, 0:auto, 3:USNaval Obs, 5: EURLabs, 6:Soviet, 7: NTSC (China), 8: NTPL (India)
  uint8_t reserved1[5];
} __attribute__((packed));

constexpr uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021; // (0->8) Dynamic platform model, 0:portable, 2:stationary, 3:pedestrian, 4:automotive, 5:sea, 6:airbone<1g, 7:airbone<2g, 8: airbone<4g, 9:wrist watch, 10:motorbike, 11: lawnmower, 12: electric scooter
constexpr uint32_t CFG_NAVSPG_FIXMODE = 0x20110011; // (3) Position fixing mode, 1: 2D only, 2: 3D only, 3: auto 2D/3D
constexpr uint32_t CFG_NAVSPG_CONSTR_DGNSSTO = 0x201100c4; // (60) DGNSS timeout [s] 60
constexpr uint32_t CFG_NAVSPG_CONSTR_ALT = 0x401100c1; // (0) Fixed altitude (mean sea level) for 2D fix mode
constexpr uint32_t CFG_NAVSPG_CONSTR_ALTVAR = 0x401100c2; // (10000) Fixed altitude variance for 2D mode
constexpr uint32_t CFG_NAVSPG_INFIL_CNOTHRS = 0x201100ab; // (0) C/N0 threshold for deciding whether to attempt a fix
constexpr uint32_t CFG_NAVSPG_INFIL_NCNOTHRS = 0x201100aa; // (0) Number of satellites required to have C/N0 above CFG-NAVSPG-INFIL_CNOTHRS for a fix to be attempted
constexpr uint32_t CFG_NAVSPG_INFIL_MINELEV = 0x201100a4; // (5) Minimum elevation for a GNSS satellite to be used in navigation
constexpr uint32_t CFG_NAVSPG_OUTFIL_PDOP = 0x301100b1; // (250) Output filter position DOP mask (threshold)
constexpr uint32_t CFG_NAVSPG_OUTFIL_TDOP = 0x301100b2; // (250) Output filter time DOP mask (threshold)
constexpr uint32_t CFG_NAVSPG_OUTFIL_PACC = 0x301100b3; // (100) Output filter position accuracy mask (threshold)
constexpr uint32_t CFG_NAVSPG_OUTFIL_TACC = 0x301100b4; // (350 -> 300?) Output filter time accuracy mask (threshold)
constexpr uint32_t CFG_NAVSPG_OUTFIL_FACC = 0x301100b5; // (150) Output filter frequency accuracy mask (threshold)
constexpr uint32_t CFG_NAVSPG_UTCSTANDARD = 0x2011001c; // (0) UTC standard 0:auto, 3:USNaval Obs, 5: EURLabs, 6:Soviet, 7: NTSC (China), 8: NTPL (India)
constexpr uint32_t CFG_MOT_GNSSSPEED_THRS = 0x20250038; // (0) GNSS speed threshold below which platform is considered as stationary (a.k.a. static hold threshold)
constexpr uint32_t CFG_MOT_GNSSDIST_THRS = 0x3025003b; //  (0) Distance above which GNSS-based stationary motion is exit (a.k.a. static hold distance threshold)

/**
 * Used to receive high rate PVT solution
 */
class UbxNavPvt92
{
public:
  static constexpr MsgId ID = UBX_NAV_PVT;
  uint32_t iTow;      // ms
  uint16_t year;      // UTC year
  uint8_t month;      // 1..12
  uint8_t day;        // 1..31
  uint8_t hour;       // 0..23
  uint8_t min;        // 0..59
  uint8_t sec;        // 0..60
  union {
    uint8_t value;
    struct {
      uint8_t validDate: 1; // 1 = valid UTC Date
      uint8_t validTime: 1; // 1 = valid UTC time of day
      uint8_t fullyResolved: 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved.
      uint8_t validMag: 1; // 1 = valid magnetic declination
    };
  } valid;
  uint32_t tAcc;      // time accuracy (ns)
  int32_t nano;       // Fraction of seconds (ns x -1e9..1e9)
  uint8_t fixType;    // 0: no-fix, 1: dead-recon, 2: 2D, 3: 3D, 4: GNSS+DR, 5: time fix only
  union {
    uint8_t value;
    struct {
      uint8_t gnssFixOk: 1; // 1 = valid fix (i.e within DOP & accuracy masks)
      uint8_t diffSoln: 1; // 1 = differential corrections were applied
      uint8_t psmState: 3; // Power save mode state
      uint8_t headVehValid: 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t carrSoln: 2; // Carrier phase range solution status: 0: no carrier, 1: carrier with floating ambiguities, 2: with fixed ambiguities
    };
  } flags;            // fix status flags
  union {
    uint8_t value;
    struct {
      uint8_t reserved : 5;
      uint8_t confirmedAvai: 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
      uint8_t confirmedDate: 1; // 1 = UTC Date validity could be confirmed
      uint8_t confirmedTime: 1; // 1 = UTC Time of Day could be confirmed
    };
  } flags2;           // additional flags
  uint8_t numSV;      // Number of satelites in Nav Sol
  int32_t lon;        // Longnitude, deg x 1e-7
  int32_t lat;        // Latitude, deg x 1e-7
  int32_t height;     // Height above allipsoid (mm)
  int32_t hSML;       // Height above mean sea level (mm)
  uint32_t hAcc;      // horizontal accuracy (mm)
  uint32_t vAcc;      // vertical accuracy (mm)
  int32_t velN;       // notrh velocity (mm/s)
  int32_t velE;       // east velocity (mm/s)
  int32_t velD;       // down velocity (mm/s)
  int32_t gSpeed;     // ground speed (mm/s, 2D)
  int32_t headMot;    // heading of motion (deg x 1e-5)
  uint32_t sAcc;      // speed accuracy (mm/s)
  uint32_t headAcc;   // heading accuracy (deg x 1e-5)
  uint16_t pDOP;      // position Dillution DOP (x 0.01)
  union {
    uint16_t value;
    struct {
      uint8_t invalidLlh: 1;        // 1 = Invalid lon, lat, height and hMSL
      uint8_t lastCorrectionAge: 4; // Age of the most recently received differential correction: 0: ?, 1: 0-1, 2: 1-2, 3: 2-5, 4: 5-10, 5: 10-15...
    };
  } flags3;           // additional flags
  uint32_t reserved1; // reserved
  int32_t headVeh;    // heading of vehicle (deg x 1e-5), only if headVehValid flag is set
  int16_t magDec;     // Magnetic declination (deg x 1e-2), only in ADR4.10+
  uint16_t magAcc;     // Magnetic declination accuracy (deg x 1e-2), only in ADR4.10+
} __attribute__((packed));

/**
 * Used to receive satellite information
 */
class UbxNavSat
{
public:
  static constexpr MsgId ID = UBX_NAV_SAT;
  uint32_t iTow;      // time of week
  uint8_t version;    // message version (0x01)
  uint8_t numSvs;     // number of satelites
  uint16_t reserved1;
  struct {
    uint8_t gnssId; // GNSS ID
    uint8_t svId;   // Satelite ID
    uint8_t cno;    // dBHz
    int8_t elev;    // deg +/-90
    int16_t azim;   // deg 0-360
    int16_t prRes;  // Pseudorange residual
    union {
      uint32_t value;
      struct {
        uint8_t qualityInd: 3; // quality indicatopr: 0-no signal, 1-searching, 2-aquired, 3-unstable, 4-code locked, 5,6,7-code and carrier locked
        uint8_t svUsed: 1; // used for navigation
        uint8_t health: 2; // signal health 0-unknown, 1-healthy, 2-unhealty
        uint8_t difCorr: 1; // differential correction available for this SV
        uint8_t smoothed: 1; // carrier smotthed pseudorange used
        uint8_t orbitSource: 3; // orbit source: 0-no inform, 1-ephemeris, 2-almanac, 3-assistnow offline, 4-assistnow autonomous, 5,6,7-other
        uint8_t ephAvail: 1; // ephemeris available
        uint8_t elmAvail: 1; // almanac available
        uint8_t enoAvail: 1; // assistnow offline available
        uint8_t eopAvail: 1; // assistnow autonomous available
        uint8_t reserved: 1;
        uint8_t sbasCorrUsed: 1; // SBAS corrections used
        uint8_t rtcmCorrUsed: 1; // RTCM corrections used
        uint8_t slasCorrUsed: 1; // SLAS corrections used
        uint8_t spartnCorrUsed: 1; // SPARTN corrections used
        uint8_t prCorrUsed: 1; // Pseudorange corrections used
        uint8_t crCorrUsed: 1; // Carrier range corrections used
        uint8_t doCorrUsed: 1; // Range rate (Doppler) corrections used
        uint8_t clasCorrUsed: 1; // CLAS corrections used
      };
    } flags;
  } sats[];
} __attribute__((packed));

// ----------------------------------------------------------------------------------------

template<typename MsgType, typename Enable = void>
class UbxFrame
{
  static_assert(sizeof(MsgType) != 0, "MsgType cannot be empty!");
public:
  UbxFrame(const MsgType& m): msg(m)
  {
    const uint8_t* it = reinterpret_cast<const uint8_t*>(this) + 2; // start after sync headers
    crc = ubxChecksum(0, it, sizeof(MsgType) + 4); // plus msgClass + msgId + length
  }

  const uint16_t hdr = UBX_SYNC;
  const MsgId id = MsgType::ID;
  const uint16_t length = sizeof(MsgType);
  const MsgType msg;
  uint16_t crc;
} __attribute__((packed));

// specialization for zero-size messages
template<typename MsgType>
class UbxFrame<MsgType, typename std::enable_if<std::is_empty<MsgType>::value>::type>
{
public:
  UbxFrame(const MsgType& m)
  {
    const uint8_t* it = reinterpret_cast<const uint8_t*>(this) + 2; // start after sync headers
    crc = ubxChecksum(0, it, 4); // only msgClass + msgId + length
  }

  const uint16_t hdr = UBX_SYNC;
  const MsgId id = MsgType::ID;
  const uint16_t length = 0;
  uint16_t crc;
} __attribute__((packed));

// specialization for UbxRequest, which has variable length payload
template<>
class UbxFrame<UbxRequest>
{
public:
  UbxFrame(const UbxRequest& m)
  {
    auto ptr = std::copy_n(reinterpret_cast<const uint8_t*>(&m.msgId), 2, data); // msgId
    ptr = std::copy_n(reinterpret_cast<const uint8_t*>(&m.length), 2, ptr); // length
    ptr = std::copy_n(m.payload, m.length, ptr); // payload

    const uint8_t* it = reinterpret_cast<const uint8_t*>(this) + 2; // start after sync headers
    uint16_t crc = ubxChecksum(0, it, m.length + 4); // plus msgClass + msgId + length + payload
    data[m.length + 4] = crc & 0xff; // crc low byte
    data[m.length + 5] = crc >> 8; // crc high byte
  }

  uint16_t size() const
  {
    return ((uint16_t)data[2] | ((uint16_t)data[3] << 8)) + 8; // header(2) + msgId(2) + length(2) + payload + crc(2)
  }

  const uint16_t hdr = UBX_SYNC;
  uint8_t data[80] = {0};
};

}
