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
#include "model_infer.h"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/thread/mutex.hpp> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
// #include <pcl/filters/passthrough.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <cuda_runtime.h>
#include <cudaFilter.h>
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libavutil/hwcontext.h>
    #include <libavutil/opt.h>
}
using namespace std;
using boost::asio::ip::tcp;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointTRGB> PointCloudRGB;
typedef PointCloudRGB::Ptr PointCloudRGB_ptr;
typedef PointCloudT::Ptr PointCloudT_ptr;

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
    void draw_results(std::vector<Detection>& res, cv::Mat& img);
    void tcp_accept_handler(const boost::system::error_code& ec);
    bool tcp_connected = false;
    void init_cudaFilter(uint32_t width, uint32_t height);
    void cuda_filter_cloud();
    void convert2PCL(const rs2::points&);
    void encode_pointcloud(tcp::iostream* socketStream);

    std::unique_ptr<RealsenseContext> rs_ctx;
    std::shared_ptr<pcl::io::OctreePointCloudCompression<PointT>> pc_compressor;
      
    cv::Mat yuv_mat;
    cv::Mat frame;
    bool modelComplete = false;
    bool modelReady = false;
    char detectedResult[2000];
    std::shared_ptr<YOLOv8ImageInferencer> detector;
    AVFormatContext *m_fmt_ctx;
    AVCodecContext *m_codec_ctx;
    AVPacket *srcPkt;
    AVPacket *desPkt;
    AVFrame *srcFrame;
    AVFrame *desFrame;

    cudaStream_t cuda_stream = NULL;
    PointCloudT_ptr cloud_src;
    PointCloudT_ptr cloud_dst;
    float* device_input_mem_ptr = NULL;
    float* device_output_mem_ptr = NULL;
    std::shared_ptr<cudaFilter> cudaFilter_ptr = NULL;
    bool pointcloud_update = false;
    boost::mutex lock;
};


#endif