




#pragma once
#include "sl_lidar_driver.h"

#ifndef __cplusplus
#error "The RPlidar SDK requires a C++ compiler to be built"
#endif


namespace rp { namespace standalone{ namespace rplidar {
    using namespace sl;
    typedef LidarScanMode RplidarScanMode;

enum {
   DRIVER_TYPE_SERIALPORT = 0x0,
   DRIVER_TYPE_TCP = 0x1,
   DRIVER_TYPE_UDP = 0x2,
};

class RPlidarDriver {
public:
    enum {
        DEFAULT_TIMEOUT = 2000,
    };

    enum {
        MAX_SCAN_NODES = 8192,
    };

    enum {
        LEGACY_SAMPLE_DURATION = 476,
    };

public:




    static RPlidarDriver * CreateDriver(_u32 drivertype = CHANNEL_TYPE_SERIALPORT);
    

    RPlidarDriver(sl_u32 channelType);



    static void DisposeDriver(RPlidarDriver * drv);












    u_result connect(const char *path, _u32 portOrBaud, _u32 flag = 0);
    

    void disconnect();


    bool isConnected(); 





    u_result reset(_u32 timeout = DEFAULT_TIMEOUT);

    u_result clearNetSerialRxCache() {
        return RESULT_OK;
    }


    u_result getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs = DEFAULT_TIMEOUT);
   

    u_result getTypicalScanMode(_u16& outMode, _u32 timeoutInMs = DEFAULT_TIMEOUT);







    u_result startScan(bool force, bool useTypicalScan, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL);







    u_result startScanExpress(bool force, _u16 scanMode, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL, _u32 timeout = DEFAULT_TIMEOUT);







    u_result getHealth(rplidar_response_device_health_t & health, _u32 timeout = DEFAULT_TIMEOUT);





    u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = DEFAULT_TIMEOUT);




    u_result setMotorPWM(_u16 pwm);


    u_result startMotor();


    u_result stopMotor();






    u_result checkMotorCtrlSupport(bool & support, _u32 timeout = DEFAULT_TIMEOUT);





	u_result  setLidarIpConf(const rplidar_ip_conf_t& conf, _u32 timeout = DEFAULT_TIMEOUT);





    u_result  getLidarIpConf(rplidar_ip_conf_t& conf, _u32 timeout = DEFAULT_TIMEOUT);




	u_result getDeviceMacAddr(_u8* macAddrArray, _u32 timeoutInMs = DEFAULT_TIMEOUT);




    u_result stop(_u32 timeout = DEFAULT_TIMEOUT);


















    u_result grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);








    u_result ascendScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t count);








    u_result getScanDataWithInterval(rplidar_response_measurement_node_t * nodebuffer, size_t & count);








    u_result getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count);


    virtual ~RPlidarDriver();
protected:
    RPlidarDriver();

private:
    sl_u32 _channelType;
    IChannel* _channel;
    ILidarDriver* _lidarDrv;
    
};




}}}
