#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include "ProtocolNmea.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <tf/transform_datatypes.h> 

#define BUFFER_MAX 1024

typedef boost::shared_ptr<boost::asio::io_service> ContextPtr;
typedef boost::shared_ptr<boost::asio::serial_port> SerialPortPtr;

class SensorGnss {
public:
    explicit SensorGnss(ros::NodeHandle& nh);
    virtual ~SensorGnss();

protected:
    void handle_context(ContextPtr p_context);
    void handle_gnss(boost::asio::yield_context yield, SerialPortPtr p_serial, ContextPtr p_context);
    bool wgs84_to_ecef(double &x, double &y, double &z, const double &longitude, const double &latitude, const double &altitude);
    bool init_origin(const double &longitude, const double &latitude, const double &altitude, std::vector<double> &params);
    bool wgs84_to_enu(double &x, double &y, double &z, const double &lon, const double &lat, const double &alt);
    ros::Time get_time_from_gnss(const boost::posix_time::ptime &time);
    bool update_gnss_data();
    bool handle_write(boost::asio::yield_context yield, std::shared_ptr<std::vector<uint8_t>> p_frame, SerialPortPtr p_serial);
    // rcl_interfaces::msg::SetParametersResult handle_callback_set_parameters(const std::vector<rclcpp::Parameter> &parameters);
    std::string ptime_to_str(const boost::posix_time::ptime& data);

    ros::Publisher mp_pub_nav;
    ros::Publisher mp_pub_gnss;
    ros::Publisher mp_pub_pose;
    ContextPtr mp_context;
    boost::thread m_thread;
    std::string m_frame_id = "gnss";
    InfoNmeaPtr mp_gpgga;
    InfoNmeaPtr mp_gphdt;
    InfoNmeaPtr mp_gprmc;
    std::shared_ptr<std::vector<double>> mp_map_origin;
    std::string m_gnss_pose_zero;
    int m_gnss_pose_rtk_status = 0;
    double m_gnss_rotation = 0.0;
    InfoNmeaPtr mp_response_action;
    bool m_enabled = true;
    double m_hdop = -1.0;
    boost::posix_time::ptime m_time_hdop;
    ros::NodeHandle m_nh;
public:
    //待定*****
    // void reconfigureCallback(your_package::YourConfig& config, uint32_t level);
    // dynamic_reconfigure::Server<your_package::YourConfig> m_reconfigureServer;
    // dynamic_reconfigure::Server<your_package::YourConfig>::CallbackType m_reconfigureCallback;
};

SensorGnss::SensorGnss(ros::NodeHandle& nh) : m_nh(nh)
{
    std::string gnss_wgs84 = nh.param<std::string>("gnss", "gnss");
    mp_pub_nav = nh.advertise<sensor_msgs::NavSatFix>(gnss_wgs84, 10);
    std::string gnss_origin = nh.param<std::string>("gnss_origin", "gnss_origin");
    mp_pub_gnss = nh.advertise<std_msgs::Float64MultiArray>(gnss_origin, 10);
    std::string gnss_pose = nh.param<std::string>("gnss_pose", "gnss_pose");
    mp_pub_pose = nh.advertise<geometry_msgs::PoseStamped>(gnss_pose, 10);
    m_frame_id = nh.param<std::string>("frame_id", "gnss");
    m_gnss_pose_zero = nh.param<std::string>("gnss_pose_zero", "");
    m_gnss_pose_rtk_status = nh.param<int>("gnss_pose_rtk_status", 0);
    m_hdop = nh.param<double>("hdop", -1.0);

    // m_reconfigureCallback = boost::bind(&SensorGnss::reconfigureCallback, this, _1, _2);
    // m_reconfigureServer.setCallback(m_reconfigureCallback);

    std::string gnss_com = nh.param<std::string>("com", "/dev/ttyTHS0");
    if (gnss_com.empty() || !boost::filesystem::exists(gnss_com)) {
        ROS_ERROR("GNSS串口路径错误: %s", gnss_com.c_str());
        return;
    }
    int gnss_baud = nh.param<int>("baud", 115200);
    m_enabled = nh.param<bool>("enabled", true);

    ROS_INFO("GNSS串口: %s; 波特率: %d; GNSS帧ID: %s; WGS84坐标系主题: %s; GNSS原始数据主题: %s; 东北天坐标系主题: %s; 东北天坐标系原点: %s; 东北天坐标系RTK定位状态: %d; 节点状态: %s; hdop: %f",
             gnss_com.c_str(), gnss_baud, m_frame_id.c_str(), gnss_wgs84.c_str(), gnss_origin.c_str(), gnss_pose.c_str(),
             (m_gnss_pose_zero.empty() ? "<使用第一个点>" : m_gnss_pose_zero.c_str()), m_gnss_pose_rtk_status,
             (m_enabled ? "启用" : "禁用"), m_hdop);

    if (!m_gnss_pose_zero.empty()) {
        std::vector<std::string> ps;
        boost::split(ps, m_gnss_pose_zero, boost::is_any_of(","));
        if (ps.size() != 3) {
            ROS_ERROR("东北天坐标系原点经纬高非法: %s", m_gnss_pose_zero.c_str());
        } else {
            double longitude = atof(ps[0].c_str());
            double latitude = atof(ps[1].c_str());
            double altitude = atof(ps[2].c_str());
            if (-180.0 > longitude || 180.0 < longitude || -90.0 > latitude || 90.0 < latitude || -9000.0 > altitude || 9000.0 < altitude) {
                ROS_ERROR("东北天坐标系原点经纬高非法: %s", m_gnss_pose_zero.c_str());
            } else {
                auto p_map_origin = std::make_shared<std::vector<double>>();
                init_origin(longitude, latitude, altitude, *p_map_origin);
                mp_map_origin = p_map_origin;
                ROS_INFO("建立地图的经纬度与坐标的对应关系: %.2f,%.2f,%.2f -> %.10f,%.10f,%.2f",
                         (*p_map_origin)[0], (*p_map_origin)[1], (*p_map_origin)[2], longitude, latitude, altitude);
            }
        }
    }

    mp_context = boost::make_shared<ContextPtr::element_type>();
    m_thread = boost::thread(boost::bind(&SensorGnss::handle_context, this, mp_context));
    auto p_serial_gnss = boost::make_shared<SerialPortPtr::element_type>(*mp_context);
    boost::system::error_code ec;
    p_serial_gnss->open(gnss_com, ec);
    if (ec) {
        ROS_ERROR("打开GNSS串口失败: %s; 串口: %s", ec.message().c_str(), gnss_com.c_str());
        return;
    }
    p_serial_gnss->set_option(boost::asio::serial_port_base::baud_rate(gnss_baud));
    p_serial_gnss->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    p_serial_gnss->set_option(boost::asio::serial_port_base::character_size(8));
    p_serial_gnss->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    p_serial_gnss->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    boost::asio::spawn(*mp_context, boost::bind(&SensorGnss::handle_gnss, this, boost::placeholders::_1, p_serial_gnss, mp_context));
}

SensorGnss::~SensorGnss() {
    auto p_context = mp_context;
    if (p_context) {
        p_context->stop();
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        m_thread.interrupt();
    }
}

void SensorGnss::handle_context(ContextPtr p_context) {
    while (true) {
        ROS_INFO("开始GNSS工作线程");
        try {
            boost::asio::io_service::work work(*p_context);
            p_context->run();
        } catch (const boost::thread_interrupted&) {
            break;
        } catch (const std::exception& e) {
            ROS_ERROR("发生错误: %s", e.what());
        }
        boost::this_thread::sleep(boost::posix_time::seconds(10));
    }
    ROS_INFO("结束GNSS工作线程");
}

void SensorGnss::handle_gnss(boost::asio::yield_context yield, SerialPortPtr p_serial, ContextPtr) 
{
    char buffer[BUFFER_MAX];
    boost::system::error_code ec;
    ProtocolNmea protocol;
    auto p_frame_total = std::make_shared<std::vector<uint8_t>>();
    InfoNmeaPtr p_response;
    auto p_frame = std::make_shared<std::vector<uint8_t>>();
    std::shared_ptr<sensor_msgs::NavSatFix> p_pose_first;
    boost::posix_time::seconds time_log_duration(1);
    auto time_log_last = boost::posix_time::second_clock::universal_time();
    std::vector<double> params;
    int status_last = -1;
    boost::posix_time::ptime gpgga_time_last;

    while (ros::ok()) {
        auto count = p_serial->async_read_some(boost::asio::buffer(buffer, BUFFER_MAX), yield[ec]);
        if (ec || 0 >= count) {
            ROS_ERROR("读取数据失败: %s", ec.message().c_str());
            return;
        }
        if (!m_enabled) {
            mp_gpgga.reset();
            mp_gphdt.reset();
            continue;
        }
        p_frame_total->insert(p_frame_total->end(), buffer, buffer + count);
        if (1000 < p_frame_total->size()) {
            ROS_ERROR("缓冲区太长; 尝试删除部分数据");
            p_frame_total->erase(p_frame_total->begin(), p_frame_total->begin() + 300);
        }
        while ((p_frame = protocol.split(p_frame_total))) {
            p_response = std::make_shared<InfoNmea>();
            if (protocol.decode(p_response, p_frame)) {
                auto id = p_response->id;
                std::transform(id.begin(), id.end(), id.begin(), ::toupper);
                auto time_now = ros::Time::now();
                auto time_current = boost::posix_time::microsec_clock::universal_time();
                p_response->time_system = time_current;

                // ... [其他逻辑保持相似]
                

                if ("GNGGA" == id || "GPGGA" == id) 
                {
                    // GPGGA只有时间，没有日期，这里的日期取的是系统日期，需要根据GPRMC的日期修正
                    auto p_gprmc = mp_gprmc;
                    if (p_gprmc)
                    {
                        auto t = boost::posix_time::ptime(p_gprmc->time.date(), p_response->time.time_of_day());
                        if(boost::posix_time::time_duration(0, 0, 0) == t.time_of_day()){
                            // 当时间跨天的时候，需要校验GPRMC是不是比GPGGA提前到了，如果GPRMC比GPGGA晚到的话，会导致时间回退。
                            // 例如从2024-06-27 23:59:59跳回2024-06-27 00:00:00，要修正为2024-06-28 00:00:00
                            if(!gpgga_time_last.is_not_a_date_time() && gpgga_time_last > t){
                                t += boost::gregorian::days(1);
                            }
                        }
                        
                        p_response->time = t;
                        gpgga_time_last = p_response->time;
                    }

                    // sensor_msgs::msg::NavSatFix msg;
                    sensor_msgs::NavSatFix msg;
                    msg.header.stamp = get_time_from_gnss(p_response->time_system);
                    msg.header.frame_id = m_frame_id;
                    msg.status.status = p_response->status;
                    msg.latitude = p_response->latitude;
                    msg.longitude = p_response->longitude;
                    msg.altitude = p_response->altitude;
                    if (0 < mp_pub_nav.getNumSubscribers())
                    {
                        mp_pub_nav.publish(msg);
                    }

                    // 检测是不是可以发布位置信息
                    mp_gpgga = p_response;
                    update_gnss_data();

                    // 定时打印GNSS
                    if (status_last != p_response->status || time_log_last + time_log_duration <= time_current)
                    {
                        std::string str_status;
                        if (GNSS_STATUS_SINGLE == p_response->status)
                        {
                            str_status = (boost::format("单点定位[%d]") % p_response->status).str();
                        }
                        else if (GNSS_STATUS_RTK_FLOAT == p_response->status)
                        {
                            str_status = (boost::format("RTK浮点解[%d]") % p_response->status).str();
                        }
                        else if (GNSS_STATUS_RTK_FIX == p_response->status)
                        {
                            str_status = (boost::format("RTK固定解[%d]") % p_response->status).str();
                        }
                        else
                        {
                            str_status = (boost::format("未知[%d]") % p_response->status).str();
                        }
                        LOG_INFO("GNSS 状态:" << status_last << " -> " << str_status << "; 坐标:" << (boost::format("%.12f,%.12f,%.4f") % p_response->longitude % p_response->latitude % p_response->altitude).str());
                        status_last = p_response->status;
                        time_log_last = time_current;
                        if (boost::posix_time::seconds(1) == time_log_duration && GNSS_STATUS_RTK_FIX == p_response->status)
                        {
                            time_log_duration = boost::posix_time::seconds(60);
                        }
                    }
                } else if ("GNHDT" == id || "GPHDT" == id) {
                    mp_gphdt = p_response;
                    update_gnss_data();
                } else if ("GNRMC" == id || "GPRMC" == id) {
                    mp_gprmc = p_response;
                } else {
                    mp_response_action = p_response;
                }
            }
        }
    }
}

bool SensorGnss::init_origin(const double &longitude, const double &latitude, const double &altitude, std::vector<double> &params) 
{
    double lamb = latitude * M_PI / 180.0;
    double phi = longitude * M_PI / 180.0;
    double sin_lat = sin(lamb);
    double cos_lat = cos(lamb);
    double sin_lon = sin(phi);
    double cos_lon = cos(phi);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (params.empty())
    {
        params.push_back(x);
        params.push_back(y);
        params.push_back(z);
        params.push_back(sin_lon);
        params.push_back(cos_lon);
        params.push_back(sin_lat);
        params.push_back(cos_lat);
        double ecef_x0, ecef_y0, ecef_z0;
        wgs84_to_ecef(ecef_x0, ecef_y0, ecef_z0, longitude, latitude, altitude);
        params.push_back(ecef_x0);
        params.push_back(ecef_y0);
        params.push_back(ecef_z0);
    }
    return true;
}

// bool SensorGnss::wgs84_to_ecef(double &x, double &y, double &z, const double &longitude, const double &latitude, const double &altitude) {
//     // ... [保持不变]
// }

// bool SensorGnss::wgs84_to_enu(double &x, double &y, double &z, const double &lon, const double &lat, const double &alt) {
//     // ... [保持不变]
// }
bool SensorGnss::wgs84_to_ecef(double &x, double &y, double &z, const double &longitude, const double &latitude, const double &altitude)
{
    double a = 6378137;
    double b = 6356752.3142;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);
    double lamb = latitude * M_PI / 180.0;
    double phi = longitude * M_PI / 180.0;
    double s = sin(lamb);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lat = sin(lamb);
    double cos_lat = cos(lamb);
    double sin_lon = sin(phi);
    double cos_lon = cos(phi);

    x = (altitude + N) * cos_lat * cos_lon;
    y = (altitude + N) * cos_lat * sin_lon;
    z = (altitude + (1 - e_sq) * N) * sin_lat;
    return true;
}

bool SensorGnss::wgs84_to_enu(double &x, double &y, double &z, const double &lon, const double &lat, const double &alt)
{
    auto p_map_origin = mp_map_origin;
    if (!p_map_origin)
    {
        return false;
    }
    double ecef_x, ecef_y, ecef_z;
    wgs84_to_ecef(ecef_x, ecef_y, ecef_z, lon, lat, alt);
    
    double sin_lon = p_map_origin->at(3);
    double cos_lon = p_map_origin->at(4);
    double sin_lat = p_map_origin->at(5);
    double cos_lat = p_map_origin->at(6);
    double ecef_x0 = p_map_origin->at(7);
    double ecef_y0 = p_map_origin->at(8);
    double ecef_z0 = p_map_origin->at(9);
    double delta_x = ecef_x - ecef_x0;
    double delta_y = ecef_y - ecef_y0;
    double delta_z = ecef_z - ecef_z0;
    x = -sin_lon * delta_x + cos_lon * delta_y;
    y = -sin_lat * cos_lon * delta_x - sin_lat * sin_lon * delta_y + cos_lat * delta_z;
    z = cos_lat * cos_lon * delta_x + cos_lat * sin_lon * delta_y + sin_lat * delta_z;
    return true;
}

ros::Time SensorGnss::get_time_from_gnss(const boost::posix_time::ptime &time) {
    ros::Time time_now;
    auto ms = (time - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1))).total_milliseconds();
    time_now.sec = static_cast<uint32_t>(ms / 1000);
    time_now.nsec = static_cast<uint32_t>((ms % 1000) * 1000000);
    return time_now;
}

// bool SensorGnss::update_gnss_data() {
//     // ... [保持不变]
// }

// bool SensorGnss::handle_write(boost::asio::yield_context yield, std::shared_ptr<std::vector<uint8_t>> p_frame, SerialPortPtr p_serial) {
//     // ... [保持不变]
// }

bool SensorGnss::update_gnss_data(){
    auto p_gpgga = mp_gpgga;
    auto p_gphdt = mp_gphdt;
    
    if(!p_gpgga){
        return false;
    }
    double yaw = 0.0;
    if(p_gphdt && 200 >= abs((p_gpgga->time_system - p_gphdt->time_system).total_milliseconds())){
        yaw = p_gphdt->yaw;
    }
    //std_msgs::msg::Float64MultiArray msg;
    std_msgs::Float64MultiArray msg;
    auto t = get_time_from_gnss(p_gpgga->time_system);
    // 时间
    msg.data.push_back(t.sec + 0.000000001 * t.nsec);
    // GNSS状态
    msg.data.push_back(p_gpgga->status);
    //经纬高
    msg.data.push_back(p_gpgga->longitude);
    msg.data.push_back(p_gpgga->latitude);
    msg.data.push_back(p_gpgga->altitude);
    // 方位角
    msg.data.push_back(yaw);
    // UTC时间
    t = get_time_from_gnss(p_gpgga->time);
    msg.data.push_back(t.sec + 0.000000001 * t.nsec);
    // hdop
    msg.data.push_back(p_gpgga->hdop);
    // 卫星个数
    msg.data.push_back(p_gpgga->satellite_count);
    // 海拔原始值
    msg.data.push_back(p_gpgga->alt_origin);
    // 地球椭球面相对大地水准面的高度
    msg.data.push_back(p_gpgga->alt_correct);
    if(0 < mp_pub_gnss.getNumSubscribers())
    {
        mp_pub_gnss.publish(msg);
    }

    if((0 == m_gnss_pose_rtk_status &&  GNSS_STATUS_RTK_FIX == p_gpgga->status) 
        || (1 == m_gnss_pose_rtk_status && (GNSS_STATUS_RTK_FIX == p_gpgga->status || GNSS_STATUS_RTK_FLOAT == p_gpgga->status))
        || (2 == m_gnss_pose_rtk_status && (GNSS_STATUS_SINGLE <= p_gpgga->status)))
    {
        if (0.0 > m_hdop || m_hdop > p_gpgga->hdop)
        {
            if (!mp_map_origin)
            {
                auto p_map_origin = std::make_shared<std::vector<double>>();
                init_origin(p_gpgga->longitude, p_gpgga->latitude, p_gpgga->altitude, *p_map_origin);
                mp_map_origin = p_map_origin;
                LOG_INFO("建立地图的经纬度与坐标的对应关系:" << (boost::format("%.2f,%.2f,%.2f -> %.10f,%.10f,%.2f") % (*p_map_origin)[0] % (*p_map_origin)[1] % (*p_map_origin)[2] % p_gpgga->longitude % p_gpgga->latitude % p_gpgga->altitude).str());
            }
            // geometry_msgs::msg::PoseStamped msg;
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = get_time_from_gnss(p_gpgga->time_system);
            msg.header.frame_id = m_frame_id;
            wgs84_to_enu(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, p_gpgga->longitude, p_gpgga->latitude, p_gpgga->altitude);
            // tf2::Quaternion orientation;
            tf::Quaternion orientation;
            // GNSS的坐标系正北为0，顺时针为正，ROS坐标系为东北天坐标系，正东为0，逆时针为正
            orientation.setRPY(0.0, 0.0, -(yaw)*M_PI / 180.0 + 0.5 * M_PI + m_gnss_rotation * M_PI / 180.0);
            msg.pose.orientation.x = orientation.x();
            msg.pose.orientation.y = orientation.y();
            msg.pose.orientation.z = orientation.z();
            msg.pose.orientation.w = orientation.w();
            if (0 < mp_pub_pose.getNumSubscribers())
            {
                mp_pub_pose.publish(msg);
            }
        }else{
            auto time_current = boost::posix_time::microsec_clock::universal_time();
            if(m_time_hdop.is_not_a_date_time() || m_time_hdop + boost::posix_time::seconds(1) <= time_current){
                m_time_hdop = time_current;
                LOG_WARN("定位精度超限; 定位状态:"<<p_gpgga->status<<"; 卫星数:"<<p_gpgga->satellite_count<<"; hdop["<<m_hdop<<"]:"<<p_gpgga->hdop);
            }
        }
    }

    // 清除gpss，避免多次发送定位
    mp_gpgga = nullptr;
    return true;
}

bool SensorGnss::handle_write(boost::asio::yield_context yield, FramePtr p_frame, SerialPortPtr p_serial)
{
    //LOG_INFO("发送数据帧:"<<frame_to_str(p_frame->begin(), p_frame->end()));
    std::size_t len_send = 0;
    boost::system::error_code ec;
    while (len_send < p_frame->size())
    {
        auto count = p_serial->async_write_some(boost::asio::buffer(p_frame->data() + len_send, p_frame->size() - len_send), yield[ec]);
        if (ec)
        {
            LOG_ERROR("发送数据时发生错误:" << ec.message());
            return false;
        }
        if (0 >= count)
        {
            return false;
        }
        len_send += count;
    }
    return true;
}

// rcl_interfaces::msg::SetParametersResult handle_callback_set_parameters(const std::vector<rclcpp::Parameter> &parameters);
// rcl_interfaces::msg::SetParametersResult SensorGnss::handle_callback_set_parameters(const std::vector<rclcpp::Parameter>& parameters) {
//     // Note: ROS 1 does not have the same parameter callback system as ROS 2.
//     // You may need to implement a service or topic-based parameter update mechanism.
//     return rcl_interfaces::msg::SetParametersResult(); // Placeholder for compatibility
// }

//待定
// void SensorGnss::reconfigureCallback(your_package::YourConfig& config, uint32_t level) {
//     // 在此处理参数更新逻辑
//     ROS_INFO("Updated parameter: param1 = %d, param2 = %f", config.param1, config.param2);
// }

std::string SensorGnss::ptime_to_str(const boost::posix_time::ptime& data) {
    try {
        return boost::gregorian::to_iso_extended_string(data.date()) + " " + boost::posix_time::to_simple_string(data.time_of_day());
    } catch (std::exception& e) {
        ROS_ERROR("时间点转换失败: %s", e.what());
    }
    return "";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_gnss_node");
    ros::NodeHandle nh;
    SensorGnss gnss(nh);
    ros::spin();
    return 0;
}