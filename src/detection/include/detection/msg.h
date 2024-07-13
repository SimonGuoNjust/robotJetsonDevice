#ifndef MSG_H
#define MSG_H
#define MREADY 1
#define MCONFIRM 2
#define MSEARCH 3
#define MRUNNING 4
#define MCLOSE 5
#include <string>

class connect_msg
{
public:
    char senderName[20];
    char status;
    char ip_addr[16];
    unsigned short ip_len;
    unsigned short udp_port;
    char videoPath[60];
    connect_msg(){}
    connect_msg(const char* _user_name, const char _status, const char* _ip_addr, int _ip_len, int _udp_port,
    const char* _videoPath)
    {
        memcpy(senderName, _user_name, std::strlen(_user_name));
        senderName[std::strlen(_user_name)] = 0;
        memcpy(videoPath,_videoPath, std::strlen(_videoPath));
        videoPath[std::strlen(_videoPath)] = 0;
        status = _status;
        ip_len = _ip_len;
        memcpy(ip_addr, _ip_addr, _ip_len);
        ip_addr[_ip_len] = 0;
        udp_port = _udp_port;
    }
    char* to_string()
    {
        return (char*) this;
    }
};

class result_msg
{
public:
    char senderName[20];
    char status;
    char results[2000];
};

typedef connect_msg* msg_ptr;
#endif