#include "camera.h"
#define THREAD_DETECT 1
using namespace std;
using namespace chrono;
extern "C" void convert_rgb_to_yu12(uint8_t *input, uint8_t *output);

// char resultBuffer[2000];
// bool update = false;
// boost::mutex mtx_;


void ImageProcessor::convert2PCL(const rs2::points& points){
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud_src->width  = static_cast<uint32_t>( sp.width()  );   
    cloud_src->height = static_cast<uint32_t>( sp.height() );
    cloud_src->is_dense = false;
    cloud_src->points.resize( points.size() );
    // clock_t t1 = clock();
    // std::cout << (t1 - t0) / (double) CLOCKS_PER_SEC<< "s" << endl;

    // auto Texture_Coord = points.get_texture_coordinates();
    // clock_t t2 = clock();
    // std::cout << (t2 - t1) / (double) CLOCKS_PER_SEC << "s" << endl;
    auto Vertex = points.get_vertices();
    // clock_t t3 = clock();
    // std::cout << (t3 - t2) / (double) CLOCKS_PER_SEC<< "s" << endl;
    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud_src->points[i].x = Vertex[i].x;
        cloud_src->points[i].y = Vertex[i].y;
        cloud_src->points[i].z = Vertex[i].z;
    }
}

// void receive_results(const std_msgs::String::ConstPtr &msg)
// {
//     char* results = (char*)msg->data.c_str();
//     int frame_cnt = *(int*)results;
//     int res_cnt = *(int*)(results + 4);
//     int results_len = 4 * (2 + res_cnt * 7);
//     if (results_len < 0) 
//     {
//         ROS_INFO_STREAM("r_len: " << results_len);
//         return;
//     }
//     ROS_INFO_STREAM("length " << results_len << " bytes");
//     mtx_.lock();
//     memcpy(resultBuffer, results, results_len);
//     update = true;
//     mtx_.unlock();
// }

// cv::Rect get_rect(cv::Mat& img, float bbox[4]) {
//     float l, r, t, b;
//     float r_w = 640 / (img.cols * 1.0);
//     float r_h = 640 / (img.rows * 1.0);

//     if (r_h > r_w) {
//         l = bbox[0];
//         r = bbox[2];
//         t = bbox[1] - (640 - r_w * img.rows) / 2;
//         b = bbox[3] - (640 - r_w * img.rows) / 2;
//         l = l / r_w;
//         r = r / r_w;
//         t = t / r_w;
//         b = b / r_w;
//     } else {
//         l = bbox[0] - (640 - r_h * img.cols) / 2;
//         r = bbox[2] - (640 - r_h * img.cols) / 2;
//         t = bbox[1];
//         b = bbox[3];
//         l = l / r_h;
//         r = r / r_h;
//         t = t / r_h;
//         b = b / r_h;
//     }
//     l = std::max(0.0f, l);
//     t = std::max(0.0f, t);
//     int width = std::max(0, std::min(int(round(r - l)), img.cols - int(round(l))));
//     int height = std::max(0, std::min(int(round(b - t)), img.rows - int(round(t))));

//     return cv::Rect(int(round(l)), int(round(t)), width, height);
// }


// void ImageProcessor::draw_results(cv::Mat& img)
// { 
//     int res_cnt = 0;
//     if (mtx_.try_lock())
//     {
//         if (update)
//         {
//             update=false;
//             res_cnt = *(int*)(resultBuffer + 4);
//             memcpy(detectedResult, resultBuffer+4, 4 * (1 + res_cnt) * 7);
//         }
//         mtx_.unlock();
//     }
//     res_cnt = *(int*)(resultBuffer + 4);
//     float* results = (float*)(detectedResult + 4);
//     float bbox[4];
//     for (int i = 0; i < res_cnt; i++) {
//         int basic_pos = i * 7;
//         int keep_flag = results[basic_pos + 6];
//         if (keep_flag == 1) {
//             bbox[0] = results[basic_pos + 0];
//             bbox[1] = results[basic_pos + 1];
//             bbox[2] = results[basic_pos + 2];
//             bbox[3] = results[basic_pos + 3];
//             float conf = results[basic_pos + 4];
//             cv::Rect r = get_rect(img, bbox);
//             cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
//             cv::putText(img, "apple", cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
//                             cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//             cv::putText(img, std::to_string(conf), cv::Point(r.x + 5, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
//                             cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//         } 
//     }
// }

void ImageProcessor::draw_results(std::vector<Detection>& res, cv::Mat& img)
{ 
    for (size_t j = 0; j < res.size(); j++) {
        cv::Rect r = get_rect(img, res[j].bbox);
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::putText(img, "apple", cv::Point(r.x+10, r.y), cv::FONT_HERSHEY_PLAIN, 1.2,
                    cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
}

void ImageProcessor::init_cudaFilter(uint32_t point_width, uint32_t point_height)
{
    uint32_t point_count = point_width * point_height;
    ROS_INFO_STREAM("init point count " << point_count);
    // cloud_src->width = point_width;
    // cloud_src->height = point_height;
    // cloud_src->resize(point_count);

    cloud_dst->width = point_count;
    cloud_dst->height = 1;
    cloud_dst->resize(point_count);

    // float* input_cloud_data = (float*) cloud_src->points.data();
    // memset(input_cloud_data, 0, sizeof(float) * 4 * point_count);

    // float* output_cloud_data = (float*) cloud_dst->points.data();
    // memset(output_cloud_data, 0, sizeof(float) * 4 * point_count);
    
    checkCudaErrors(cudaStreamCreate(&cuda_stream));

    checkCudaErrors(cudaMallocManaged(&device_input_mem_ptr, sizeof(float) * 4 * point_count, cudaMemAttachHost));
    checkCudaErrors(cudaStreamAttachMemAsync (cuda_stream, device_input_mem_ptr));
    // checkCudaErrors(cudaMemcpyAsync(device_input_mem_ptr, input_cloud_data, sizeof(float) * 4 * point_count, cudaMemcpyHostToDevice, cuda_stream));
    checkCudaErrors(cudaStreamSynchronize(cuda_stream));


    cudaMallocManaged(&device_output_mem_ptr, sizeof(float) * 4 * point_count, cudaMemAttachHost);
    cudaStreamAttachMemAsync (cuda_stream, device_output_mem_ptr);
    cudaStreamSynchronize(cuda_stream);

    cudaFilter_ptr.reset(new cudaFilter(cuda_stream));
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
    
    auto device_name = "cuda";
    auto model_path = "/home/nvidia/ros_mmdeploy_ws/src/qtVis/src/yolov8/build/apple.engine";
    // auto detector = new YOLOv8ImageInferencer(model_path);
    detector = std::make_shared<YOLOv8ImageInferencer>(model_path);
    cout << "model initial success" << endl;
    frame = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
    detector->inferOneImage(frame);

    cloud_src.reset(new PointCloudT);
    cloud_dst.reset(new PointCloudT);

}

void ImageProcessor::cuda_filter_cloud()
{
    if (cuda_stream == NULL)
    {
        init_cudaFilter(cloud_src->width, cloud_src->height);
    }
    uint32_t point_count = cloud_src->width * cloud_src->height;
    ROS_INFO_STREAM("filter before " << point_count);
    PointT min, max;
    pcl::getMinMax3D(*cloud_src, min, max);
    ROS_INFO_STREAM("min x: " << min.x << "max x: " << max.x);
    ROS_INFO_STREAM("min y: " << min.y << "max y: " << max.y);
    ROS_INFO_STREAM("min z: " << min.z << "max z: " << max.z);
    uint32_t count_left = 0;
    int status = 0;
    FilterParam_t setP;
    FilterType_t type = VOXELGRID;
    setP.type = type;
    setP.voxelX =  Parameters<float>::getParam("voxelX");
    setP.voxelY =  Parameters<float>::getParam("voxelY");
    setP.voxelZ =  Parameters<float>::getParam("voxelZ");
    cudaFilter_ptr->set(setP);

    float* input_cloud_data = (float*) cloud_src->points.data();
    checkCudaErrors(cudaStreamAttachMemAsync (cuda_stream, device_input_mem_ptr));
    checkCudaErrors(cudaMemcpyAsync(device_input_mem_ptr, input_cloud_data, sizeof(float) * 4 * point_count, cudaMemcpyHostToDevice, cuda_stream));
    cudaStreamSynchronize(cuda_stream);

    cudaDeviceSynchronize();
    status = cudaFilter_ptr->filter(device_output_mem_ptr, &count_left, device_input_mem_ptr, point_count);
    checkCudaErrors(cudaDeviceSynchronize());
    // checkCudaErrors(cudaMemcpyAsync(output_cloud_data, device_output_mem_ptr, sizeof(float) * 4 * count_left, cudaMemcpyDeviceToHost, cuda_stream));
    // checkCudaErrors(cudaDeviceSynchronize());
    ROS_INFO_STREAM("filter remain " << count_left);
    if (status != 0) return; 
    cloud_dst->width = count_left;
    cloud_dst->height = 1;
    cloud_dst->resize(count_left);

    for (std::size_t i = 0; i < cloud_dst->size(); ++i)
    {
        cloud_dst->points[i].x = device_output_mem_ptr[i*4+0];
        cloud_dst->points[i].y = device_output_mem_ptr[i*4+1];
        cloud_dst->points[i].z = device_output_mem_ptr[i*4+2];
    }
}

void ImageProcessor::tcp_accept_handler(const boost::system::error_code& ec)
{
    if (ec) {
        ROS_INFO_STREAM("connect error" << ec);
        return;
    }
    ROS_INFO_STREAM("tcp connected");
    tcp_connected = true;
}

void ImageProcessor::encode_pointcloud(tcp::iostream* socketStream)
{
    while(ros::param::has("/ffmpeg/videoPath"))
    {
        lock.lock();
        if (pointcloud_update)
        {
            pointcloud_update = false;
            lock.unlock();
            pc_compressor->encodePointCloud(cloud_src, *socketStream);
        }
        else
        {
            lock.unlock();
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        }
    }
}

void ImageProcessor::run()
{
    std::vector<Detection> res;
    // ros::Rate rate(30);
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
    // SharedMemoryROSCommunicator<cv::Mat> msg_que("cvMat", 1280 * 720 * 3 + 4);
    auto start_clk = system_clock::now();
    rs2::gl::pointcloud pc;
    pcl::io::compression_Profiles_e compressionProfile;
    compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    const pcl::io::configurationProfile_t selectedProfile =
          pcl::io::compressionProfiles_[compressionProfile];
    bool showStatistics;
    double pointResolution;
    float octreeResolution;
    bool doVoxelGridDownDownSampling;
    unsigned int iFrameRate;
    bool doColorEncoding;
    unsigned int colorBitResolution;

    pointResolution = selectedProfile.pointResolution;
    octreeResolution = float(selectedProfile.octreeResolution);
    doVoxelGridDownDownSampling = false;
    iFrameRate = selectedProfile.iFrameRate;
    doColorEncoding = selectedProfile.doColorEncoding;
    colorBitResolution = selectedProfile.colorBitResolution;
    pc_compressor = std::make_shared<pcl::io::OctreePointCloudCompression<PointT>>(
        compressionProfile,
        showStatistics,
        pointResolution,
        octreeResolution,
        doVoxelGridDownDownSampling,
        iFrameRate,
        doColorEncoding,
        static_cast<unsigned char>(colorBitResolution)
    );
    boost::asio::io_service io_service;
    tcp::endpoint ep(tcp::v4(), 6666);
    tcp::acceptor acceptor(io_service, ep);
    tcp::iostream socketStream;
    acceptor.accept(*socketStream.rdbuf());
    // pcl::PassThrough<PointT> pass_;
    // pass_.setFilterFieldName("z");
    // pass_.setFilterLimits(0, 3);
    boost::thread thread_pointcloud(boost::bind(&ImageProcessor::encode_pointcloud, this, &socketStream));
    while(ros::param::has("/ffmpeg/videoPath")) {
        ros::spinOnce();  
        auto frames = rs_ctx->getFrames();
        auto duration = duration_cast<microseconds>(system_clock::now() - start_clk);
        ROS_INFO_STREAM("get frame cost " 
        << double(duration.count()) * microseconds::period::num / microseconds::period::den 
        << "s");
        start_clk = system_clock::now();
        rs2::frame color_frame = frames.get_color_frame();
        if (cnt % 5 == 0 && !socketStream.fail())
        {
            auto depth = frames.get_depth_frame();
            auto points = pc.calculate(depth);
            ROS_INFO_STREAM("encoding");
            start_clk = system_clock::now();
            convert2PCL(points);
            duration = duration_cast<microseconds>(system_clock::now() - start_clk);
            ROS_INFO_STREAM("conver pointcloud cost " 
            << double(duration.count()) * microseconds::period::num / microseconds::period::den 
            << "s");
            // cuda_filter_cloud();
            lock.lock();
            pointcloud_update=true;
            lock.unlock();
            // PointCloudT_ptr cloud_out(new PointCloudT);
            // pass_.setInputCloud(pcl_pointcloud);
            // pass_.filter(*cloud_out);
            duration = duration_cast<microseconds>(system_clock::now() - start_clk);
            ROS_INFO_STREAM("encode pointcloud cost " 
            << double(duration.count()) * microseconds::period::num / microseconds::period::den 
            << "s");
        }
        // auto cnt = (int)color_frame.get_frame_number();
        if (!color_frame) continue;
		const int width = color_frame.as<rs2::video_frame>().get_width();
		const int height = color_frame.as<rs2::video_frame>().get_height();
		cv::Mat input_mat(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        // send to model_node for detect
        // msg_que.publish(input_mat.data, cnt);
        cv::cvtColor(input_mat, frame, CV_BGR2RGB);
        detector->inferOneImage(res, frame);
        draw_results(res, input_mat);
        res.clear();
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

        ret = av_interleaved_write_frame(m_fmt_ctx, desPkt);
        duration = duration_cast<microseconds>(system_clock::now() - start_clk);
        ROS_INFO_STREAM("one frame cost " 
        << double(duration.count()) * microseconds::period::num / microseconds::period::den 
        << "s");
        start_clk = system_clock::now();
        if (ret < 0) {
			ROS_INFO_STREAM("write error");
            continue;
        }
        av_packet_unref(desPkt);      
        // rate.sleep();
    }
    thread_pointcloud.join();
}

ImageProcessor::~ImageProcessor()
{
    // cudaFree(device_input_mem_ptr);
    // cudaFree(device_output_mem_ptr);
    // cudaStreamDestroy(cuda_stream);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    // ros::Subscriber sub = nh.subscribe<std_msgs::String>("/results", 10, receive_results);
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