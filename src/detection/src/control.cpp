#include <iostream>
#include <string>
#include "timer.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "msg.h"
#include "sharedMemory.h"
#include "types.h"

using namespace::boost::asio;
using boost::asio::ip::udp;
using boost::asio::ip::tcp;

enum { max_length = 2048};
char dataBuffer[max_length];
char resultBuffer[4096];
// SharedMemoryROSCommunicator<int> resultPublisher("result", 4 * (2 + kMaxNumOutputBbox * bbox_element));
// float recvBuf[2 + kMaxNumOutputBbox * bbox_element];
int last_frame = -1;
int frame_cnt = 0;
int res_cnt = -1;
udp::endpoint sender_endpoint;

void recConnectMsg(
        udp::socket* sock,
        udp::endpoint* sender_endpoint,
        TimeWatcher<std::chrono::seconds>* watcher,
        const boost::system::error_code& error, 
        std::size_t)
{ 
    // ROS_INFO_STREAM("call back");
    if (error)
    {   
        return;
    }
    int length = sizeof(connect_msg);
    // int length = 101;
    // ROS_INFO_STREAM( "length" << length );
    watcher->resetStartTime();
    connect_msg* msg = (connect_msg*)dataBuffer;
    if (msg->status == MSEARCH)
    {
        std::string name;
        ros::param::get("deviceName", name);
        const char* name_ = name.c_str();
        memcpy(msg->senderName, name_ , std::strlen(name_ ) + 1);
        ROS_INFO_STREAM("recv " << msg->senderName );
        ROS_INFO_STREAM("sender ip " << sender_endpoint->address().to_string() );
        ROS_INFO_STREAM("sender port " << sender_endpoint->port() );
        sock->send_to(buffer(dataBuffer,length), *sender_endpoint);
    }
    else if(msg->status == MREADY)
    {
        watcher->setConnected(true);
        std::string video_path(msg->videoPath);
        int pos = video_path.find("localhost");
        video_path.replace(pos, 9, sender_endpoint->address().to_string());
        ROS_INFO_STREAM("sender video path: " << video_path );
        ros::param::set("/ffmpeg/videoPath", video_path);
        // ros_msg.data = video_path;
        sock->send_to(buffer(dataBuffer,length), *sender_endpoint);
        // break;
    }
    else if (msg->status==MRUNNING)
    {
        watcher->resetStartTime();
    }
    else if (msg->status==MCLOSE)
    {
        ros::param::del("/ffmpeg/videoPath");
        ROS_INFO_STREAM("connect close");
    }
}

void receive_results(const std_msgs::String::ConstPtr &msg,
                    udp::socket* sock,
                    udp::endpoint* sender_endpoint)
{
    char* results = (char*)msg->data.c_str();
    frame_cnt = *(int*)results;
    res_cnt = *(int*)(results + 4);
    int results_len = 4 * (2 + res_cnt * 7);
    if (results_len < 0) 
    {
        ROS_INFO_STREAM("r_len: " << results_len);
        return;
    }
    result_msg* r_msg = (result_msg*)resultBuffer;
    ROS_INFO_STREAM("length " << results_len << " bytes");
    memcpy(r_msg->results, results, results_len);
    memcpy(resultBuffer, dataBuffer, 21);
    if (frame_cnt <= last_frame)
    {
        ROS_INFO_STREAM("frame_cnt: " << frame_cnt << " last_frame: " << last_frame);
        return;
    }
    last_frame = frame_cnt;
    sock->send_to(buffer(resultBuffer, results_len + 21), *sender_endpoint);
    ROS_INFO_STREAM("send frame_cnt " << frame_cnt << " result");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    
    std_msgs::String ros_msg;
    ros::Rate rate(5);
    auto watcher = new TimeWatcher<std::chrono::seconds>(10);
    io_service io_sev;
    unsigned short port(27777);
    auto ep = udp::endpoint(udp::v4(), port);
    ROS_INFO_STREAM( ep.address().to_string() );
    ROS_INFO_STREAM( ep.port() );
    udp::socket sock(io_sev, ep);
    sock.set_option(boost::asio::socket_base::receive_buffer_size(256));
    sock.set_option(boost::asio::socket_base::reuse_address(true));
    boost::asio::socket_base::bytes_readable command(true);
    sock.io_control(command);

    // ros::Subscriber sub = nh.subscribe<std_msgs::String>("/results", 10, 
    // boost::bind(receive_results, _1, &sock, &sender_endpoint));

    while(ros::ok()){
        if (watcher->isConnected() && watcher->isTimeOut())
        {
            ros::param::del("/ffmpeg/videoPath");
            ROS_INFO_STREAM("connect lost");
            watcher->setConnected(false);
        }
        sock.async_receive_from(boost::asio::buffer(dataBuffer), sender_endpoint,
            boost::bind(&recConnectMsg, &sock, &sender_endpoint, watcher, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
        io_sev.poll_one();
        // size_t length = sock.receive_from(buffer(data, max_length), sender_endpoint);
        rate.sleep();
        ros::spinOnce();
    }
    delete watcher;
    ros::param::del("model_valid");
    return 0;
}