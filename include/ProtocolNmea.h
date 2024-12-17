#ifndef PROTOCOL_NMEA_HPP_
#define PROTOCOL_NMEA_HPP_

#include <ros/ros.h>

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <sstream> 

#include <thread>
#include <boost/date_time.hpp>
#include <boost/format.hpp>

#define GNSS_STATUS_SINGLE 1
#define GNSS_STATUS_RTK_FIX 4
#define GNSS_STATUS_RTK_FLOAT 5

// COMMAND
#define NMEA_CONTROL_COMMAND "command"
// 重置
#define NMEA_CONTROL_FRESET "freset,response: OK"
// 重启
#define NMEA_CONTROL_REBOOT "REBOOT"
// 设置成功
#define NMEA_CONTROL_SUCCESS "response: OK"
// 设置失败
#define NMEA_CONTROL_FAILURE "FAILURE"

#ifdef LOG_TAG
    #define LOG_GET_LOGGER() std::string(LOG_TAG)
#else
    #define LOG_GET_LOGGER() std::string("")
#endif

#define CREATE_LOG_MSG(MSG) std::stringstream abc123;abc123 << "[" << std::this_thread::get_id() << "][" << __FUNCTION__ << "@" << __LINE__ << "]" << MSG
#define LOG_INFO(MSG) {CREATE_LOG_MSG(MSG);ROS_INFO_STREAM("[" + LOG_GET_LOGGER() + "] " + abc123.str());}
#define LOG_ERROR(MSG) {CREATE_LOG_MSG(MSG);ROS_ERROR_STREAM("[" + LOG_GET_LOGGER() + "] " + abc123.str());}
#define LOG_WARN(MSG) {CREATE_LOG_MSG(MSG);ROS_WARN_STREAM("[" + LOG_GET_LOGGER() + "] " + abc123.str());}
#define LOG_DEBUG(MSG) {CREATE_LOG_MSG(MSG);ROS_DEBUG_STREAM("[" + LOG_GET_LOGGER() + "] " + abc123.str());}
class InfoNmea
{
public:
    std::string id;
    int status = -1;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    boost::posix_time::ptime time;
    double speed = 0.0;
    double yaw = 0.0;
    int satellite_count = 0;
    double hdop = 0.0;

    double alt_origin = 0.0;
    double alt_correct = 0.0;

    std::vector<std::string> params;
    boost::posix_time::ptime time_system;
};
typedef std::shared_ptr<InfoNmea> InfoNmeaPtr;

typedef std::shared_ptr<std::vector<unsigned char>> FramePtr;

class ProtocolNmea
{
public:
    virtual bool decode(InfoNmeaPtr &p_response, FramePtr &p_frame);
    virtual FramePtr split(FramePtr &p_frame);

protected:
    virtual bool char_to_hex(unsigned char &data, const char &h, const char &l);
    virtual std::string remove_calc(const std::string &data);
};
typedef std::shared_ptr<ProtocolNmea> ProtocolNmeaPtr;

#endif