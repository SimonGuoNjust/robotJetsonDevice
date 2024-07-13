#ifndef DETECT_H
#define DETECT_H
#include <iostream>
#include <string>
#include "rosparam.h"
#include "std_msgs/String.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2-gl/rs_processing_gl.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc/types_c.h"
#include <memory>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include "sharedMemory.h"
  
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libavutil/hwcontext.h>
    #include <libavutil/opt.h>
}
using namespace std;

class RealsenseContext
{
public:
    RealsenseContext()
    {
        rscfg = std::unique_ptr<rs2::config>(new rs2::config());
        rspipe = std::unique_ptr<rs2::pipeline>(new rs2::pipeline());
        aligner = std::unique_ptr<rs2::align>(nullptr);
    }

    void init()
    {
        rscfg->enable_stream(RS2_STREAM_COLOR, 
            Parameters<int>::getParam("rgbStreamWidth"),
            Parameters<int>::getParam("rgbStreamHeight"),
            RS2_FORMAT_RGB8,
            Parameters<int>::getParam("rgbStreamFPS"));
        if (Parameters<bool>::getParam("infraredEnabled"))
        {
            rscfg->enable_stream(RS2_STREAM_INFRARED);
        }
        if (Parameters<bool>::getParam("depthEnabled"))
        {
            rscfg->enable_stream(RS2_STREAM_DEPTH, 
            Parameters<int>::getParam("depthStreamWidth"),
            Parameters<int>::getParam("depthStreamHeight"),
            RS2_FORMAT_Z16,
            Parameters<int>::getParam("depthStreamFPS"));
        }
    }

    void init(const std::string& bag_path)
    {
        rscfg->enable_device_from_file(bag_path);
    }
    inline void start()
    {
        rspipe->start(*rscfg);
    }
    inline rs2::frameset getFrames()
    {
        return rspipe->wait_for_frames();
    }

private:
    std::unique_ptr<rs2::config> rscfg;
    std::unique_ptr<rs2::pipeline> rspipe;
    std::unique_ptr<rs2::align> aligner;
};


class ImageProcessor{
public:
    ImageProcessor();
    ~ImageProcessor();
    void stop();
    bool isStreamInit();
    void run();

private:
    void draw_results(cv::Mat& img);
    std::unique_ptr<RealsenseContext> rs_ctx;
    cv::Mat yuv_mat;
    bool modelComplete = false;
    bool modelReady = false;
    char detectedResult[2000];
    AVFormatContext *m_fmt_ctx;
    AVCodecContext *m_codec_ctx;
    AVPacket *srcPkt;
    AVPacket *desPkt;
    AVFrame *srcFrame;
    AVFrame *desFrame;
};


#endif