#include "ProtocolNmea.h"
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
// #include "../../public/UtilityTool.h"

#define FRAME_LEN 38

bool ProtocolNmea::decode(InfoNmeaPtr &p_response, FramePtr &p_frame)
{
    if(!p_response){
        p_response = std::make_shared<InfoNmea>();
    }
    p_response->params.clear();

    auto iter_begin = p_frame->begin() + 1;
    auto iter_frame_end = p_frame->end() - 3;
    std::vector<std::string> params;
    for (auto iter = p_frame->begin() + 1; iter != iter_frame_end; ++iter)
    {
        if (',' == *iter)
        {
            params.push_back(std::string(iter_begin, iter));
            iter_begin = iter + 1;
        }
    }
    if (iter_frame_end != iter_begin)
    {
        params.push_back(std::string(iter_begin, iter_frame_end));
    }
    if ("GNGGA" == params[0] || "GPGGA" == params[0])
    {
        if (13 > params.size())
        {
            // LOG_ERROR("结果数量错误[13]:" << params.size());
            LOG_ERROR("结果数量错误[13]: " + std::to_string(params.size()));
            return false;
        }
        if (5 > params[2].size() || 6 > params[4].size() || params[9].empty() || params[6].empty())
        {
            //LOG_ERROR("数据错误["<<params[0]<<"]:" <<std::string(reinterpret_cast<const char *>(p_frame->data()), p_frame->size()));
            return false;
        }
        p_response->id = params[0];
        p_response->longitude = atof(params[4].substr(0, 3).c_str()) + (atof(params[4].substr(3).c_str()) / 60.0);
        p_response->latitude = atof(params[2].substr(0, 2).c_str()) + (atof(params[2].substr(2).c_str()) / 60.0);
        p_response->altitude = atof(params[9].c_str());
        p_response->alt_origin = atof(params[9].c_str());
        p_response->alt_correct = atof(params[11].c_str());
        if(!params[6].empty()){
            p_response->status = atoi(params[6].c_str());
        }
        std::vector<std::string> ps0;
        boost::split(ps0, params[1], boost::is_any_of("."));
        if(2 != ps0.size()){
            LOG_ERROR("时间格式非法:"<<params[1]);
            return false;
        }
        if(6 > ps0[0].size()){
            ps0[0].insert(0, 6 - ps0[0].size(), '0');
        }else if(6 < ps0[0].size()){
            LOG_ERROR("时间格式非法:"<<params[1]);
            return false;
        }
        if(3 > ps0[1].size()){
            ps0[1].push_back('0');
        }else if(3 < ps0[1].size()){
            LOG_ERROR("时间格式非法:"<<params[1]);
            return false;
        }
        try{
            p_response->time = boost::posix_time::ptime(boost::posix_time::second_clock::universal_time().date()
                , boost::posix_time::time_duration(atoi(ps0[0].substr(0, 2).c_str()), atoi(ps0[0].substr(2, 2).c_str()), atoi(ps0[0].substr(4, 2).c_str())))
                + boost::posix_time::milliseconds(atoi(ps0[1].c_str()));
            if(p_response->time.is_not_a_date_time()){
                LOG_ERROR("时间日期转换失败:"<<params[1]<<" "<<ps0[0]<<"."<<ps0[1]);
                return false;
            }
        }catch(const std::exception &e){
            LOG_ERROR("时间日期转换失败:"<<e.what()<<"; "<<params[9]<<" "<<ps0[0]<<"."<<ps0[1]);
            return false;
        }
        p_response->satellite_count = atoi(params[7].c_str());
        p_response->hdop = atof(params[8].c_str());
    }else if ("GNRMC" == params[0] || "GPRMC" == params[0]){
        if (10 > params.size())
        {
            LOG_ERROR("结果数量错误[10]:" << params.size());
            return false;
        }
        if (5 > params[3].size() || 6 > params[5].size())
        {
            //LOG_ERROR("数据错误["<<params[0]<<"]:" <<std::string(reinterpret_cast<const char *>(p_frame->data()), p_frame->size()));
            return false;
        }
        p_response->id = params[0];
        p_response->longitude = atof(params[5].substr(0, 3).c_str()) + (atof(params[5].substr(3).c_str()) / 60.0);
        p_response->latitude = atof(params[3].substr(0, 2).c_str()) + (atof(params[3].substr(2).c_str()) / 60.0);
        p_response->speed = atof(params[7].c_str());
        p_response->yaw = atof(params[8].c_str());
        std::vector<std::string> ps0;
        boost::split(ps0, params[1], boost::is_any_of("."));
        if(2 != ps0.size()){
            LOG_ERROR("时间格式非法:"<<params[1]);
            return false;
        }
        if(6 > ps0[0].size()){
            ps0[0].insert(0, 6 - ps0[0].size(), '0');
        }else if(6 < ps0[0].size()){
            LOG_ERROR("时间格式非法:"<<params[1]);
            return false;
        }
        if(3 > ps0[1].size()){
            ps0[1].push_back('0');
        }else if(3 < ps0[1].size()){
            LOG_ERROR("时间格式非法:"<<params[1]);
            return false;
        }

        if(6 > params[9].size()){
            params[9].insert(0, 6 - params[9].size(), '0');
        }else if(6 < params[9].size()){
            LOG_ERROR("日期格式非法:"<<params[9]);
            return false;
        }
        try{
            p_response->time = boost::posix_time::ptime(boost::gregorian::date(2000 + atoi(params[9].substr(4, 2).c_str()), atoi(params[9].substr(2, 2).c_str()), atoi(params[9].substr(0, 2).c_str()))
                , boost::posix_time::time_duration(atoi(ps0[0].substr(0, 2).c_str()), atoi(ps0[0].substr(2, 2).c_str()), atoi(ps0[0].substr(4, 2).c_str())))
                + boost::posix_time::milliseconds(atoi(ps0[1].c_str()));
            if(p_response->time.is_not_a_date_time()){
                LOG_ERROR("时间日期转换失败:"<<params[9]<<" "<<ps0[0]<<"."<<ps0[1]);
                return false;
            }
        }catch(const std::exception &e){
            LOG_ERROR("时间日期转换失败:"<<e.what()<<"; "<<params[9]<<" "<<ps0[0]<<"."<<ps0[1]);
            return false;
        }
    }else if ("GNHDT" == params[0] || "GPHDT" == params[0])
    {
        if (3 > params.size())
        {
            LOG_ERROR("结果数量错误[3]:" << params.size());
            return false;
        }
        if (params[1].empty())
        {
            LOG_ERROR("数据错误["<<params[0]<<"]:" <<std::string(reinterpret_cast<const char *>(p_frame->data()), p_frame->size()));
            return false;
        }
        p_response->id = params[0];
        p_response->yaw = atof(params[1].c_str());
    }else if (NMEA_CONTROL_COMMAND == params[0])
    {
        if (3 > params.size())
        {
            LOG_ERROR("结果数量错误[3]:" << params.size());
            return false;
        }
        p_response->id = params[0];
        p_response->params.push_back(params[1]);
        p_response->params.push_back(remove_calc(params[2]));
    }else if ("devicename" == params[0])
    {
        if (2 > params.size())
        {
            LOG_ERROR("结果数量错误[2]:" << params.size());
            return false;
        }
        p_response->id = NMEA_CONTROL_REBOOT;
        p_response->params.push_back(remove_calc(params[1]));
    }else{
        return false;
    }
    
    return true;
}

FramePtr ProtocolNmea::split(FramePtr &p_frame_total)
{
    auto iter_begin = p_frame_total->begin();
    FramePtr p;
    for (auto iter = p_frame_total->begin(); iter != p_frame_total->end(); ++iter)
    {
        if (*iter == '$'){
            iter_begin = iter;
        }else if (*iter == '\n')
        {
            auto iter_remove_end = iter + 1;
            if (p_frame_total->begin() != iter && *(iter - 1) == '\r')
            {
                p = std::make_shared<FramePtr::element_type>(iter_begin, iter - 1);
            }
            else
            {
                p = std::make_shared<FramePtr::element_type>(iter_begin, iter);
            }
            iter_begin = iter + 1;
            if (5 > p->size())
            {
                //LOG_ERROR("数据帧长度小于最小值[5]:" << p->size());
                continue;
            }
            if ('$' != p->at(0))
            {
                //LOG_ERROR("开始标志错误:" << static_cast<char>(p->at(0)));
                continue;
            }
            if ('*' != p->at(p->size() - 3))
            {
                //LOG_ERROR("结束标志错误:" << static_cast<char>(p->at(p->size() - 3))<<"; 数据帧:"<<std::string(reinterpret_cast<const char*>(p->data()), p->size()));
                continue;
            }
            unsigned char crc = p->at(1) ^ p->at(2);
            for (std::size_t i = 3; i < p->size() - 3; ++i)
            {
                crc = crc ^ p->at(i);
            }
            unsigned char crc_frame = 0x00;
            if (!char_to_hex(crc_frame, p->at(p->size() - 2), p->at(p->size() - 1)))
            {
                //LOG_ERROR("CRC数据错误:" << static_cast<char>(p->at(p->size() - 2)) << static_cast<char>(p->at(p->size() - 1))<<"; 数据帧:"<<std::string(reinterpret_cast<const char*>(p->data()), p->size()));
                continue;
            }
            if (crc_frame != crc)
            {
                //LOG_ERROR("CRC校验错误; 计算值:" << (boost::format("%02X") % static_cast<int>(crc)).str() << "; 帧值:" << (boost::format("%02X") % static_cast<int>(crc_frame)).str()<<"; 数据帧:"<<std::string(reinterpret_cast<const char*>(p->data()), p->size()));
                //continue;
            }

            p_frame_total->erase(p_frame_total->begin(), iter_remove_end);
            return p;
        }
    }
    return nullptr;
}

bool ProtocolNmea::char_to_hex(unsigned char &data, const char &h, const char &l)
{
    unsigned char hd = 0x00, ld = 0x00;
    if ('0' <= h && '9' >= h)
    {
        hd = h - '0';
    }
    else if ('a' <= h && 'f' >= h)
    {
        hd = h - 'a' + 0x0a;
    }
    else if ('A' <= h && 'F' >= h)
    {
        hd = h - 'A' + 0x0a;
    }
    else
    {
        return false;
    }
    if ('0' <= l && '9' >= l)
    {
        ld = l - '0';
    }
    else if ('a' <= l && 'f' >= l)
    {
        ld = l - 'a' + 0x0a;
    }
    else if ('A' <= l && 'F' >= l)
    {
        ld = l - 'A' + 0x0a;
    }
    else
    {
        return false;
    }
    data = ((hd << 4) | ld);
    return true;
}

std::string ProtocolNmea::remove_calc(const std::string &data){
    auto pose = data.rfind("*");
    if(std::string::npos == pose){
        return data;
    }else{
        return data.substr(0, pose);
    }
}