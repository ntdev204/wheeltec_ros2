


#ifndef __LIPKG_H
#define __LIPKG_H

#include <chrono>

#include "pointdata.h"
#include "tofbf.h"
#include "cmd_interface_linux.h"

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
  const int kPointFrequence = 4500;

  LiPkg();
  ~LiPkg();
  

  std::string GetSdkPackVersionNum(void) const;
  

  double GetSpeed(void); 
  

  uint16_t GetSpeedOrigin(void);
  

  uint16_t GetTimestamp(void);  
  

  bool IsFrameReady(void);  
  

  void ResetFrameReady(void);
  

  long GetErrorTimes(void);  
  

  void CommReadCallback(const char *byte, size_t len);
  

  Points2D GetLaserScanData(void);
  
 private:
  std::string sdk_pack_version_;
  uint16_t timestamp_;
  double speed_;
  long error_times_;
  bool is_frame_ready_;

  LiDARFrameTypeDef pkg_;
  Points2D frame_tmp_;
  Points2D laser_scan_data_;
  std::mutex  mutex_lock1_;
  std::mutex  mutex_lock2_;

  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t* data, long len);  
  bool AssemblePacket();  
  void SetLaserScanData(Points2D& src);
  void SetFrameReady(void);
};

}

#endif


