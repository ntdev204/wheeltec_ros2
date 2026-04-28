


#ifndef __LIPKG_H
#define __LIPKG_H

#include <chrono>

#include "pointdata.h"
#include "transform.h"
#include "cmd_interface_linux.h"
#include "slbf.h"

namespace ldlidar {

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
 public:
  const int kPointFrequence = 2300;

  LiPkg();
  ~LiPkg();

  std::string GetSdkVersionNumber(void);
  void SetProductType(LDType type_number);
  

  void SetLaserScanDir(bool dir);

  double GetSpeed(void);

  uint16_t GetSpeedOrigin(void);

  uint16_t GetTimestamp(void);

  bool IsFrameReady(void);
  void ResetFrameReady(void);
  void SetFrameReady(void);

  long GetErrorTimes(void);
  void CommReadCallback(const char *byte, size_t len);
  Points2D GetLaserScanData(void);
 
 private:
  LDType ld_product_type_;
  std::string sdk_pack_verison_;
  bool laser_scan_dir_;
  bool is_frame_ready_;
  uint16_t timestamp_;
  double speed_;
  long error_times_;

  LiDARFrameTypeDef pkg;
  Points2D frame_tmp_;
  Points2D laser_scan_data_;
  std::mutex  mutex_lock1_;
  std::mutex  mutex_lock2_;
  

  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t *data, long len);

  bool AssemblePacket();
  void SetLaserScanData(Points2D& src);
};

}

#endif

