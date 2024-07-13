#include "model_infer.h"
using namespace std;

bool mUpdate = false;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_node");
    ros::NodeHandle nh;
    ros::Rate rate1(5);
    ros::Rate rate2(20);
    ros::Publisher pub = nh.advertise<std_msgs::String>("/results", 100);
    SharedMemoryROSCommunicator<cv::Mat> sharedMemoryRosCommunicator("cvMat", MAXBUFFSIZE + 4);
    // SharedMemoryROSCommunicator<cv::Mat> resultPublisher("result", 4 * (2 + kMaxNumOutputBbox * bbox_element));
    SharedMsg msg_;
    msg_.cnt = -1;
    std_msgs::String r_msg;
    // std::vector<Detection> res;
    
    auto device_name = "cuda";
    auto model_path = "/home/nvidia/ros_mmdeploy_ws/src/qtVis/src/yolov8/build/apple.engine";
    auto detector = new YOLOv8ImageInferencer(model_path);
    cout << "model initial success" << endl;
    cv::Mat frame = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
    detector->inferOneImage(frame);
    int cnt = 0;
    float result_buffer[1 + kMaxNumOutputBbox * bbox_element];
    while(ros::ok())
    {
        if (sharedMemoryRosCommunicator.read(&msg_))
        {
            if (msg_.cnt > cnt)
            {
                frame.data = (uchar*)msg_.buf;
                cv::cvtColor(frame, frame, CV_BGR2RGB);
                detector->inferOneImage(frame);
                cnt = msg_.cnt;
                int res_count = static_cast<int>(*(detector->decode_ptr_host));
                int keep_cnt = 0;
                for (int i = 0; i < res_count; i++) {
                    int basic_pos = 1 + i * bbox_element;
                    int keep_flag = detector->decode_ptr_host[basic_pos + 6];
                    if (keep_flag == 1)
                    {
                        memcpy(result_buffer + 2 + keep_cnt * 7, detector->decode_ptr_host + basic_pos, 28);
                        keep_cnt++;
                    }
                }
                memcpy(result_buffer, &cnt, 4);
                memcpy(result_buffer + 1, &keep_cnt, 4);
                std::string r_msg_std;
                r_msg_std.assign((char*)result_buffer, (2 + keep_cnt * 7) * 4);
                r_msg.data = r_msg_std;
                pub.publish(r_msg);
                // resultPublisher.publish(result_buffer, (1 + keep_cnt * 7) * 4, cnt);
                ROS_INFO_STREAM(" " << keep_cnt << " apples detected in frame " << cnt);
            }
            else
            { 
                rate2.sleep();
            }
        }
        rate1.sleep();
        ros::spinOnce();
    }}

