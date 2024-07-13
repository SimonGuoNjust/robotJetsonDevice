#ifndef ROSPARM_H
#define ROSPARM_H
#include <string>
#include <fstream>
#include <map>
#include <ros/ros.h>
#include <ros/console.h>

template<class T>
class Parameters
{
public:
    Parameters(std::string name):
        paramName(name)
    {}
    int setParam(T value)
    {
        if (hasParam())
        {
            T oValue = getParam(paramName);
            ROS_INFO_STREAM(std::string("override ") << paramName << " = " << oValue << " with " << value << std::endl);
            ros::param::set(paramName, value);
        }
        else
        {
            ROS_INFO_STREAM(std::string("set ") << paramName << " = "  << value << std::endl);
        }
    }

    inline bool hasParam()
    {
        return ros::param::has(paramName);
    }

    T getParam()
    {
        T value;
        ros::param::get(paramName, value);
        return value;
    }

    void unsetParam()
    {
        ros::param::del(paramName);
    }

    static T getParam(std::string name)
    {
        T value;
        ros::param::get(name, value);
        return value;
    }

    T operator()(){
        return getParam();
    }
private:
    std::string paramName;
};

struct CameraConfig
{
    CameraConfig() :
        rgbStreamWidth("cameraCfg/rgbStreamWidth"),
        rgbStreamHeight("cameraCfg/rgbStreamHeight"),
        rgbStreamFPS("cameraCfg/rgbStreamFPS"),
        depthStreamWidth("cameraCfg/depthStreamWidth"),
        depthStreamHeight("cameraCfg/depthStreamHeight"),
        depthStreamFPS("cameraCfg/depthStreamFPS"),
        depthEnabled("cameraCfg/depthEnabled"),
        infraredEnabled("cameraCfg/infraredEnabled")
    {
        rgbStreamWidth.setParam(1280);
        rgbStreamHeight.setParam(720);
        rgbStreamFPS.setParam(30);
        depthStreamWidth.setParam(640);
        depthStreamHeight.setParam(480);
        depthStreamFPS.setParam(30);
        depthEnabled.setParam(true);
        infraredEnabled.setParam(true);
    }
    Parameters<int> rgbStreamWidth;
    Parameters<int> rgbStreamHeight;
    Parameters<int> rgbStreamFPS;
    Parameters<int> depthStreamWidth;
    Parameters<int> depthStreamHeight;
    Parameters<int> depthStreamFPS;
    Parameters<bool> depthEnabled;
    Parameters<bool> infraredEnabled;
};

#endif