#include "camera.h"
#define THREAD_DETECT 1
using namespace std;
using namespace chrono;
extern "C" void convert_rgb_to_yu12(uint8_t *input, uint8_t *output);

char resultBuffer[2000];
bool update = false;
boost::mutex mtx_;

void receive_results(const std_msgs::String::ConstPtr &msg)
{
    char* results = (char*)msg->data.c_str();
    int frame_cnt = *(int*)results;
    int res_cnt = *(int*)(results + 4);
    int results_len = 4 * (2 + res_cnt * 7);
    if (results_len < 0) 
    {
        ROS_INFO_STREAM("r_len: " << results_len);
        return;
    }
    ROS_INFO_STREAM("length " << results_len << " bytes");
    mtx_.lock();
    memcpy(resultBuffer, results, results_len);
    update = true;
    mtx_.unlock();
}

cv::Rect get_rect(int img_width, int img_height, float bbox[4]) {
    float l, r, t, b;
    float r_w = 640 / (img_width * 1.0);
    float r_h = 640 / (img_height * 1.0);

    if (r_h > r_w) {
        l = bbox[0];
        r = bbox[2];
        t = bbox[1] - (640 - r_w * img_height) / 2;
        b = bbox[3] - (640 - r_w * img_height) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = bbox[0] - (640 - r_h * img_width) / 2;
        r = bbox[2] - (640 - r_h * img_width) / 2;
        t = bbox[1];
        b = bbox[3];
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }
    l = std::max(0.0f, l);
    t = std::max(0.0f, t);
    int width = std::max(0, std::min(int(round(r - l)), img_width - int(round(l))));
    int height = std::max(0, std::min(int(round(b - t)), img_height - int(round(t))));

    return cv::Rect(int(round(l)), int(round(t)), width, height);
}

void ImageProcessor::draw_results(cv::Mat& img)
{ 
    int res_cnt = 0;
    if (mtx_.try_lock())
    {
        if (update)
        {
            update=false;
            res_cnt = *(int*)(resultBuffer + 4);
            memcpy(detectedResult, resultBuffer+4, 4 * (1 + res_cnt) * 7);
        }
        mtx_.unlock();
    }
    res_cnt = *(int*)(resultBuffer + 4);
    float* results = (float*)(detectedResult + 4);
    float bbox[4];
    for (int i = 0; i < res_cnt; i++) {
        int basic_pos = i * 7;
        int keep_flag = results[basic_pos + 6];
        if (keep_flag == 1) {
            bbox[0] = results[basic_pos + 0];
            bbox[1] = results[basic_pos + 1];
            bbox[2] = results[basic_pos + 2];
            bbox[3] = results[basic_pos + 3];
            float conf = results[basic_pos + 4];
            cv::Rect r = get_rect(img.cols, img.rows, bbox);
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(img, "apple", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            cv::putText(img, std::to_string(conf), cv::Point(r.x + 5, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                            cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        } 
    }
}

ImageProcessor::ImageProcessor()
{
    rs_ctx = std::move(std::unique_ptr<RealsenseContext>(new RealsenseContext()));
    char *codec_name = "h264_nvmpi";
    enum AVCodecID decodec_id = AV_CODEC_ID_H264;

    av_register_all();
    avformat_network_init();
    AVOutputFormat *fmt = av_guess_format("rtsp", NULL, NULL);
    if (!fmt) {
        std::cout << "Could not guess RTSP format";
        return;
    }

    std::string videoPath = Parameters<std::string>::getParam("/ffmpeg/videoPath");
    avformat_alloc_output_context2(&m_fmt_ctx, nullptr, "rtsp", videoPath.c_str());
    av_opt_set(m_fmt_ctx->priv_data, "rtsp_transport", Parameters<std::string>::getParam("/ffmpeg/transport").c_str(), 0);
    m_fmt_ctx->oformat = fmt;
    // AVCodec *codec = avcodec_find_encoder(decodec_id);
    AVCodec *codec = avcodec_find_encoder_by_name(codec_name);
    if (!codec) {
        std::cout << "H.264 encoder not found";
        return;
    }
    else
    {
        std::cout << "found";
    }
    
    if (m_fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        m_fmt_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    m_codec_ctx = avcodec_alloc_context3(codec);
    m_codec_ctx->width =        Parameters<int>::getParam("/camera/rgbStreamWidth");
    m_codec_ctx->height =       Parameters<int>::getParam("/camera/rgbStreamHeight");
    m_codec_ctx->pix_fmt =      AV_PIX_FMT_YUV420P;
    m_codec_ctx->time_base =    {1, Parameters<int>::getParam("/camera/rgbStreamFPS")};  // RTP timestamp clock rate
    m_codec_ctx->framerate =    {Parameters<int>::getParam("/camera/rgbStreamFPS"), 1};
    m_codec_ctx->thread_count = Parameters<int>::getParam("/ffmpeg/threads");

    m_codec_ctx->gop_size=      Parameters<int>::getParam("/ffmpeg/gop_size");
    m_codec_ctx->max_b_frames=  Parameters<int>::getParam("/ffmpeg/b_frames");
 
    //最大和最小量化系数
    m_codec_ctx->qmin =         Parameters<int>::getParam("/ffmpeg/qmin");
    m_codec_ctx->qmax =         Parameters<int>::getParam("/ffmpeg/qmax");

    m_codec_ctx->bit_rate =     Parameters<int>::getParam("/ffmpeg/bit_rate");

    if (m_fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        m_codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    av_opt_set(m_codec_ctx->priv_data, "tune", "zerolatency", 0);
    av_opt_set(m_codec_ctx->priv_data, "preset","ultrafast",0);
    // av_opt_set(m_codec_ctx->priv_data,"crf", "28", AV_OPT_SEARCH_CHILDREN);
    if (avcodec_open2(m_codec_ctx, codec, NULL) < 0) {
        std::cout << "Could not open H.264 encoder";
        return;
    }

    AVStream *stream = avformat_new_stream(m_fmt_ctx, codec);
    avcodec_parameters_from_context(stream->codecpar, m_codec_ctx);
    av_dump_format(m_fmt_ctx, 0, m_fmt_ctx->filename, 1);
    if (!(fmt -> flags & AVFMT_NOFILE)) {
        if (avio_open(&m_fmt_ctx -> pb, m_fmt_ctx->filename, AVIO_FLAG_WRITE) < 0) {
            std::cout << "Could not open output file";
            return;
        }
    }
    avformat_write_header(m_fmt_ctx, NULL);

    desPkt = av_packet_alloc();
    desFrame = av_frame_alloc();
    desFrame->width = m_codec_ctx->width;
    desFrame->height = m_codec_ctx->height;
    desFrame->format = m_codec_ctx->pix_fmt;
 
    int stride_y = desFrame->width;
    int stride_uv = desFrame->width / 2;
 
    desFrame->linesize[0] = stride_y;
    desFrame->linesize[1] = stride_uv;
    desFrame->linesize[2] = stride_uv;
    int buffer_size = av_image_get_buffer_size(m_codec_ctx->pix_fmt, desFrame->width, desFrame->height, 32);
    uint8_t *buffer = (uint8_t *)av_malloc(buffer_size);
    av_image_fill_arrays(desFrame->data, desFrame->linesize, buffer,
        m_codec_ctx->pix_fmt, desFrame->width, desFrame->height, 1);
    yuv_mat.create(720 * 3 / 2, 1280, CV_8UC1);
}

void ImageProcessor::run()
{
    ros::Rate rate(30);
    if (ros::param::has("bag_path"))
    {
        std::string bag_path = Parameters<std::string>::getParam("bag_path");
        rs_ctx->init(bag_path);
    }
    else
    {
        rs_ctx->init();
    }
    rs_ctx->start();
    for (int i = 0; i < 30; i++) {
        auto frames = rs_ctx->getFrames(); //Drop several frames for auto-exposure
    }
    int cnt = 0;
    SharedMemoryROSCommunicator<cv::Mat> msg_que("cvMat", 1280 * 720 * 3 + 4);
    auto start_clk = system_clock::now();
    while(ros::param::has("/ffmpeg/videoPath")) {
        ros::spinOnce();  
        auto frames = rs_ctx->getFrames();
        rs2::frame color_frame = frames.get_color_frame();
        // auto cnt = (int)color_frame.get_frame_number();
        if (!color_frame) continue;
		const int width = color_frame.as<rs2::video_frame>().get_width();
		const int height = color_frame.as<rs2::video_frame>().get_height();
		cv::Mat input_mat(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        // send to model_node for detect
        msg_que.publish(input_mat.data, cnt);

        draw_results(input_mat);

        convert_rgb_to_yu12(input_mat.data,(uint8_t*)yuv_mat.data);
        int frame_size = width * height;
        unsigned char *data = yuv_mat.data;
        memcpy(desFrame->data[0], data, frame_size);
        memcpy(desFrame->data[1], data + frame_size, frame_size/4);
        memcpy(desFrame->data[2], data + frame_size * 5/4, frame_size/4);
        desFrame->pts=cnt;
        int ret;
		if ((ret = avcodec_send_frame(m_codec_ctx, desFrame)) < 0) {
			ROS_INFO_STREAM("Error sending frame to encoder");
			continue;
		}
        ret = avcodec_receive_packet(m_codec_ctx, desPkt);
        if (ret < 0) {
            ROS_INFO_STREAM("Error encoding video frame" << ret);
            continue;	
        }
        desPkt->stream_index = 0;
        desPkt->pts = cnt;
        desPkt->dts = cnt++;
        // if (cnt % 30 == 0)
        // {
        //     auto duration = duration_cast<microseconds>(system_clock::now() - start_clk);
        //     ROS_INFO_STREAM("cost" 
        //     << double(duration.count()) * microseconds::period::num / microseconds::period::den 
        //     << "s");
        //     start_clk = system_clock::now();
        // }
        ret = av_interleaved_write_frame(m_fmt_ctx, desPkt);
        if (ret < 0) {
			ROS_INFO_STREAM("write error");
            continue;
        }
        av_packet_unref(desPkt);      
        // rate.sleep();
    }
}

ImageProcessor::~ImageProcessor()
{

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("/results", 10, receive_results);
    while(ros::ok())
    {
        if (ros::param::has("/ffmpeg/videoPath"))
        {
            ImageProcessor worker;
            worker.run();
        }
        rate.sleep();
        ros::spinOnce();
    }

}