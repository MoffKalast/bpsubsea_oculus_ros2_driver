/*
 * Copyright 2017 Blueprint Subsea.
 * Copyright 2026 bpsubsea_oculus_ros2_driver contributors.
 *
 * This file contains wire-compatible definitions derived from Oculus Viewer,
 * which is distributed under GPL-3.0-or-later.
 */

#pragma once

#include <cstdint>
#include <string>

namespace bpsubsea_oculus_ros2_driver::protocol
{

constexpr uint16_t kOculusCheckId = 0x4f53;
constexpr uint16_t kDefaultDataPort = 52100;
constexpr uint16_t kDefaultStatusPort = 52102;

enum OculusDeviceType : uint16_t
{
  deviceTypeUndefined = 0,
  deviceTypeImagingSonar = 1,
};

enum OculusMessageType : uint16_t
{
  messageSimpleFire = 0x15,
  messagePingResult = 0x22,
  messageSimplePingResult = 0x23,
  messageUserConfig = 0x55,
  messageDummy = 0xff,
};

enum PingRateType : uint8_t
{
  pingRateNormal = 0x00,
  pingRateHigh = 0x01,
  pingRateHighest = 0x02,
  pingRateLow = 0x03,
  pingRateLowest = 0x04,
  pingRateStandby = 0x05,
};

enum DataSizeType : uint8_t
{
  dataSize8Bit = 0,
  dataSize16Bit = 1,
  dataSize24Bit = 2,
  dataSize32Bit = 3,
};

enum OculusPartNumberType : uint16_t
{
  partNumberUndefined = 0,
  partNumberM370s = 1041,
  partNumberMT370s = 2418,
  partNumberMD370s = 1433,
  partNumberMD370s_Burton = 2294,
  partNumberMD370s_Impulse = 1217,
  partNumberC550d = 4713,
  partNumberM750d = 1032,
  partNumberMT750d = 2419,
  partNumberMD750d = 1434,
  partNumberMD750d_Burton = 1921,
  partNumberMD750d_Impulse = 1244,
  partNumberM1200d = 1042,
  partNumberMT1200d = 2420,
  partNumberMD1200d = 1435,
  partNumberMD1200d_Burton = 2086,
  partNumberMD1200d_Impulse = 1219,
  partNumberM3000d = 2203,
  partNumberMT3000d = 2599,
  partNumberMD3000d_Burton = 2659,
  partNumberMD3000d_Impulse = 2658,
  partNumberEnd = 0xffff,
};

#pragma pack(push, 1)

struct OculusMessageHeader
{
  uint16_t oculusId;
  uint16_t srcDeviceId;
  uint16_t dstDeviceId;
  uint16_t msgId;
  uint16_t msgVersion;
  uint32_t payloadSize;
  uint16_t partNumber;
};

struct OculusSimpleFireMessage
{
  OculusMessageHeader head;
  uint8_t masterMode;
  PingRateType pingRate;
  uint8_t networkSpeed;
  uint8_t gammaCorrection;
  uint8_t flags;
  double range;
  double gainPercent;
  double speedOfSound;
  double salinity;
};

struct OculusSimpleFireMessage2
{
  OculusMessageHeader head;
  uint8_t masterMode;
  PingRateType pingRate;
  uint8_t networkSpeed;
  uint8_t gammaCorrection;
  uint8_t flags;
  double range;
  double gainPercent;
  double speedOfSound;
  double salinity;
  uint32_t extFlags;
  uint32_t reserved0[2];
  uint32_t beaconLocatorFrequency;
  uint32_t reserved1[5];
};

struct OculusSimplePingResult
{
  OculusSimpleFireMessage fireMessage;
  uint32_t pingId;
  uint32_t status;
  double frequency;
  double temperature;
  double pressure;
  double speedOfSoundUsed;
  uint32_t pingStartTime;
  DataSizeType dataSize;
  double rangeResolution;
  uint16_t nRanges;
  uint16_t nBeams;
  uint32_t imageOffset;
  uint32_t imageSize;
  uint32_t messageSize;
};

struct OculusSimplePingResult2
{
  OculusSimpleFireMessage2 fireMessage;
  uint32_t pingId;
  uint32_t status;
  double frequency;
  double temperature;
  double pressure;
  double heading;
  double pitch;
  double roll;
  double speedOfSoundUsed;
  double pingStartTime;
  DataSizeType dataSize;
  double rangeResolution;
  uint16_t nRanges;
  uint16_t nBeams;
  uint32_t spare0;
  uint32_t spare1;
  uint32_t spare2;
  uint32_t spare3;
  uint32_t imageOffset;
  uint32_t imageSize;
  uint32_t messageSize;
};

struct OculusVersionInfo
{
  uint32_t firmwareVersion0;
  uint32_t firmwareDate0;
  uint32_t firmwareVersion1;
  uint32_t firmwareDate1;
  uint32_t firmwareVersion2;
  uint32_t firmwareDate2;
};

struct OculusStatusMsg
{
  OculusMessageHeader hdr;
  uint32_t deviceId;
  OculusDeviceType deviceType;
  OculusPartNumberType partNumber;
  uint32_t status;
  OculusVersionInfo versionInfo;
  uint32_t ipAddr;
  uint32_t ipMask;
  uint32_t connectedIpAddr;
  uint8_t macAddr0;
  uint8_t macAddr1;
  uint8_t macAddr2;
  uint8_t macAddr3;
  uint8_t macAddr4;
  uint8_t macAddr5;
  double temperature0;
  double temperature1;
  double temperature2;
  double temperature3;
  double temperature4;
  double temperature5;
  double temperature6;
  double temperature7;
  double pressure;
};

#pragma pack(pop)

inline std::string part_number_to_model(uint16_t part_number)
{
  switch (part_number) {
    case partNumberM370s:
    case partNumberMT370s:
    case partNumberMD370s:
    case partNumberMD370s_Burton:
    case partNumberMD370s_Impulse:
      return "Oculus M370s";
    case partNumberC550d:
      return "Oculus C550d";
    case partNumberM750d:
    case partNumberMT750d:
    case partNumberMD750d:
    case partNumberMD750d_Burton:
    case partNumberMD750d_Impulse:
      return "Oculus M750d";
    case partNumberM1200d:
    case partNumberMT1200d:
    case partNumberMD1200d:
    case partNumberMD1200d_Burton:
    case partNumberMD1200d_Impulse:
      return "Oculus M1200d";
    case partNumberM3000d:
    case partNumberMT3000d:
    case partNumberMD3000d_Burton:
    case partNumberMD3000d_Impulse:
      return "Oculus M3000d";
    default:
      return "Oculus";
  }
}

inline uint8_t data_size_bytes(uint8_t data_size)
{
  switch (data_size) {
    case dataSize8Bit:
      return 1;
    case dataSize16Bit:
      return 2;
    case dataSize24Bit:
      return 3;
    case dataSize32Bit:
      return 4;
    default:
      return 1;
  }
}

inline const char * image_encoding(uint8_t data_size)
{
  switch (data_size) {
    case dataSize8Bit:
      return "mono8";
    case dataSize16Bit:
      return "mono16";
    case dataSize24Bit:
      return "8UC3";
    case dataSize32Bit:
      return "32SC1";
    default:
      return "mono8";
  }
}

}  // namespace bpsubsea_oculus_ros2_driver::protocol
