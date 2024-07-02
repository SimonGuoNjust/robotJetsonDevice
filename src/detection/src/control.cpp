#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "msg.h"

using namespace::boost::asio;
using boost::asio::ip::udp;
using boost::asio::ip::tcp;
enum { max_length = 1024};


int main(int argc, char** argv)
{

    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/control_node", 10);
    std_msgs::String ros_msg;
    ros::Rate rate(30);

    // boost::asio::io_context io_context;
    io_service io_sev;
    const char name[20] = "nvidia-detection1";
    unsigned short port(27777);
    auto ep = udp::endpoint(udp::v4(), port);
    std::cout << ep.address().to_string() << std::endl;
    std::cout << ep.port() << std::endl;
    
    udp::socket sock(io_sev, ep);
    boost::asio::socket_base::bytes_readable command(true);
    sock.io_control(command);
    for(;;){
        char data[max_length];
        udp::endpoint sender_endpoint;
        size_t length = sock.receive_from(buffer(data, max_length), sender_endpoint);
        data[length] = 0;
        connect_msg* msg = (connect_msg*)data;
        if (msg->status == MSEARCH)
        {
            memcpy(msg->senderName, name, std::strlen(name) + 1);
            std::cout << "recv " << msg->senderName << std::endl;
            std::cout << "sender ip " << sender_endpoint.address().to_string() << std::endl;
            std::cout << "sender port " << sender_endpoint.port() << std::endl;
            sock.send_to(buffer(data,length), sender_endpoint);
        }
        else if(msg->status == MREADY)
        {
            std::string video_path(msg->videoPath);
            int pos = video_path.find("localhost");
            video_path.replace(pos, 9, sender_endpoint.address().to_string());
            std::cout << "sender video path: " << video_path << std::endl;
            ros_msg.data = video_path;
            sock.send_to(buffer(data,length), sender_endpoint);
            break;
        }
        ros::spinOnce();
    }
    while(ros::ok())
    {
        pub.publish(ros_msg);
        rate.sleep();
        ros::spinOnce();
    }
    // ip::tcp::acceptor acceptor(io_sev, ip::tcp::endpoint(ip::tcp::v4(), 1000));
    // for (;;)
    // {
    //     ip::tcp::socket socket(io_sev);
    //     acceptor.accept(socket);
    //     std::cout << socket.remote_endpoint().address() << std::endl;
    //     boost::system::error_code ec;
    //     socket.write_some(buffer("hello world!"), ec);
    //     if (ec)
    //     {
    //         std::cout << boost::system::system_error(ec).what() << std::endl;
    //         break;
    //     }
    // }
    return 0;
}