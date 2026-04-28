

 


#pragma once

#ifndef __cplusplus
#error "The Slamtec LIDAR SDK requires a C++ compiler to be built"
#endif

#include <vector>
#include <map>
#include <string>

#ifndef DEPRECATED
    #ifdef __GNUC__
        #define DEPRECATED(func) func __attribute__ ((deprecated))
    #elif defined(_MSC_VER)
        #define DEPRECATED(func) __declspec(deprecated) func
    #else
        #pragma message("WARNING: You need to implement DEPRECATED for this compiler")
        #define DEPRECATED(func) func
    #endif
#endif


#include "sl_lidar_cmd.h"

#include <string>

namespace sl {

#ifdef DEPRECATED
#define DEPRECATED_WARN(fn, replacement) do { \
        static bool __shown__ = false; \
        if (!__shown__) { \
            printDeprecationWarn(fn, replacement); \
            __shown__ = true; \
        } \
    } while (0)
#endif

    

    struct LidarScanMode
    {

        sl_u16  id;


        float   us_per_sample;


        float   max_distance;


        sl_u8   ans_type;


        char    scan_mode[64];
    };

    template <typename T>
    struct Result
    {
        sl_result err;
        T value;
        Result(const T& value)
            : err(SL_RESULT_OK)
            , value(value)
        {
        }

        Result(sl_result err)
            : err(err)
            , value()
        {
        }

        operator sl_result() const
        {
            return err;
        }

        operator bool() const
        {
            return SL_IS_OK(err);
        }

        T& operator* ()
        {
            return value;
        }

        T* operator-> ()
        {
            return &value;
        }
    };

    enum LIDARTechnologyType {
        LIDAR_TECHNOLOGY_UNKNOWN = 0,
        LIDAR_TECHNOLOGY_TRIANGULATION = 1,
        LIDAR_TECHNOLOGY_DTOF = 2,
        LIDAR_TECHNOLOGY_ETOF = 3,
        LIDAR_TECHNOLOGY_FMCW = 4,
    };

    enum LIDARMajorType {
        LIDAR_MAJOR_TYPE_UNKNOWN = 0,
        LIDAR_MAJOR_TYPE_A_SERIES = 1,
        LIDAR_MAJOR_TYPE_S_SERIES = 2,
        LIDAR_MAJOR_TYPE_T_SERIES = 3,
        LIDAR_MAJOR_TYPE_M_SERIES = 4,
        LIDAR_MAJOR_TYPE_C_SERIES = 6,
    };

    enum LIDARInterfaceType {
        LIDAR_INTERFACE_UART = 0,
        LIDAR_INTERFACE_ETHERNET = 1,
        LIDAR_INTERFACE_USB = 2,
        LIDAR_INTERFACE_CANBUS = 5,


        LIDAR_INTERFACE_UNKNOWN = 0xFFFF,
    };

    struct SlamtecLidarTimingDesc {

        sl_u32  sample_duration_uS;
        sl_u32  native_baudrate;
        
        sl_u32  linkage_delay_uS;

        LIDARInterfaceType native_interface_type;

        bool    native_timestamp_support;
    };

    

    class IChannel
    {
    public:
        virtual ~IChannel() {}

    public:
        

        virtual bool open() = 0;

        

        virtual void close() = 0;

        

        virtual void flush() = 0;

        

        virtual bool waitForData(size_t size, sl_u32 timeoutInMs = -1, size_t* actualReady = nullptr) = 0;


        

        virtual sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs = 1000) = 0;


        

        virtual int write(const void* data, size_t size) = 0;

        

        virtual int read(void* buffer, size_t size) = 0;

        

        virtual void clearReadCache() = 0;

        virtual int getChannelType() = 0;

    private:

    };

    

    class ISerialPortChannel : public IChannel
    {
    public:
        virtual ~ISerialPortChannel() {}

    public:
        virtual void setDTR(bool dtr) = 0;
    };

    

    Result<IChannel*> createSerialPortChannel(const std::string& device, int baudrate);

    

    Result<IChannel*> createTcpChannel(const std::string& ip, int port);

    

    Result<IChannel*> createUdpChannel(const std::string& ip, int port);

    enum MotorCtrlSupport
    {
        MotorCtrlSupportNone = 0,
        MotorCtrlSupportPwm = 1,
        MotorCtrlSupportRpm = 2,
    };

    enum ChannelType{
        CHANNEL_TYPE_SERIALPORT = 0x0,
        CHANNEL_TYPE_TCP = 0x1,
        CHANNEL_TYPE_UDP = 0x2,
    };

        

    struct LidarMotorInfo
    {
        MotorCtrlSupport motorCtrlSupport;


        sl_u16 desired_speed;


        sl_u16 max_speed;


        sl_u16 min_speed;
    };

    class ILidarDriver
    {
    public:
        virtual ~ILidarDriver() {}

    public:
        

        virtual sl_result connect(IChannel* channel) = 0;

        

        virtual void disconnect() = 0;
        
        

        virtual bool isConnected() = 0;

    public:
        enum
        {
            DEFAULT_TIMEOUT = 2000
        };

    public:




        virtual sl_result reset(sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;


        virtual sl_result getAllSupportedScanModes(std::vector<LidarScanMode>& outModes, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;


        virtual sl_result getTypicalScanMode(sl_u16& outMode, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;







        virtual sl_result startScan(bool force, bool useTypicalScan, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr) = 0;







        virtual sl_result startScanExpress(bool force, sl_u16 scanMode, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;







        virtual sl_result getHealth(sl_lidar_response_device_health_t& health, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;





        virtual sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;






        virtual sl_result checkMotorCtrlSupport(MotorCtrlSupport& motorCtrlSupport, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;








        virtual sl_result getFrequency(const LidarScanMode& scanMode, const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, float& frequency) = 0;





		virtual sl_result setLidarIpConf(const sl_lidar_ip_conf_t& conf, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;
       




        virtual sl_result getLidarIpConf( sl_lidar_ip_conf_t& conf, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;






		virtual sl_result getDeviceMacAddr(sl_u8* macAddrArray, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;




        virtual sl_result stop(sl_u32 timeout = DEFAULT_TIMEOUT) = 0;


















        virtual sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;































        virtual sl_result grabScanDataHqWithTimeStamp(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u64 & timestamp_uS, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;









        virtual sl_result ascendScanData(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t count) = 0;








        virtual sl_result getScanDataWithIntervalHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count) = 0;






        virtual sl_result setMotorSpeed(sl_u16 speed = DEFAULT_MOTOR_SPEED) = 0;
        



        virtual sl_result getMotorInfo(LidarMotorInfo &motorInfo, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;
    








        virtual sl_result negotiateSerialBaudRate(sl_u32 requiredBaudRate, sl_u32* baudRateDetected = NULL) = 0;








        virtual LIDARTechnologyType getLIDARTechnologyType(const sl_lidar_response_device_info_t* devInfo = nullptr) = 0;
        
        





        virtual LIDARMajorType getLIDARMajorType(const sl_lidar_response_device_info_t* devInfo = nullptr) = 0;











        virtual sl_result getModelNameDescriptionString(std::string& out_description, bool fetchAliasName = true, const sl_lidar_response_device_info_t* devInfo = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

};

    

    Result<ILidarDriver*> createLidarDriver();
}
