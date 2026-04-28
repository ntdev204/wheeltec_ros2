













#ifndef VISIBILITY_H_
#define VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif




#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define RPLIDAR_ROS_EXPORT __attribute__ ((dllexport))
    #define RPLIDAR_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define RPLIDAR_ROS_EXPORT __declspec(dllexport)
    #define RPLIDAR_ROS_IMPORT __declspec(dllimport)
  #endif

  #ifdef RPLIDAR_ROS_DLL
    #define RPLIDAR_ROS_PUBLIC RPLIDAR_ROS_EXPORT
  #else
    #define RPLIDAR_ROS_PUBLIC RPLIDAR_ROS_IMPORT
  #endif

  #define RPLIDAR_ROS_PUBLIC_TYPE RPLIDAR_ROS_PUBLIC

  #define RPLIDAR_ROS_LOCAL

#else

  #define RPLIDAR_ROS_EXPORT __attribute__ ((visibility("default")))
  #define RPLIDAR_ROS_IMPORT

  #if __GNUC__ >= 4
    #define RPLIDAR_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define RPLIDAR_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RPLIDAR_ROS_PUBLIC
    #define RPLIDAR_ROS_LOCAL
  #endif

  #define RPLIDAR_ROS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif
