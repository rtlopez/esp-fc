#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

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

class UbxCfgMsg2
{
public:
  static constexpr MsgId ID = UBX_CFG_MSG;
  uint16_t msgId;
} __attribute__((packed));

class UbxCfgMsg3
{
public:
  static constexpr MsgId ID = UBX_CFG_MSG;
  uint16_t msgId;
  uint8_t rate;
} __attribute__((packed));

class UbxCfgMsg8
{
public:
  static constexpr MsgId ID = UBX_CFG_MSG;
  uint16_t msgId;
  uint8_t channels[6];
} __attribute__((packed));

class UbxCfgPrt1
{
public:
  static constexpr MsgId ID = UBX_CFG_PRT;
  uint8_t portId;
} __attribute__((packed));

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

class UbxCfgRate6
{
public:
  static constexpr MsgId ID = UBX_CFG_RATE;
  uint16_t measRate; // in ms
  uint16_t navRate;  // ratio to measRate
  uint16_t timeRef;  // 0-utc, 1-gps, 2-glonass (18+), 3-BeiDou (18+), 4-Galileo (18+), 5-NavIC (29+)
} __attribute__((packed));

class UbxCfgSbas8
{
public:
  static constexpr MsgId ID = UBX_CFG_SBAS;
  uint8_t mode; // SBAS mode flags
  uint8_t usage;  // SBAS usage flags
  uint8_t maxSbas;  // Maximum number of SBAS prioritized tracking channels (valid range: 0 - 3) to use
  uint8_t scanmode2; // Continuation of scanmode bitmask
  uint32_t scanmode1; // Which SBAS PRN numbers to search for (bitmask).If all bits are set to zero, auto-scan (i.e. allvalid PRNs) are searched. Every bit corresponds to a PRN number
} __attribute__((packed));

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

}
