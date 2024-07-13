#ifndef __TIMEWATCHER_H__
#define __TIMEWATCHER_H__
#include <chrono>
 
template<typename Duration>
class TimeWatcher
{
public:
	TimeWatcher(unsigned int timeout = 3) :_time_out(timeout) {}
	~TimeWatcher() = default;
 
public:
	//overwrite call operator
	bool operator()()
	{
		return isTimeOut();
	}
	//is timeout
	inline bool isTimeOut()
	{
		std::chrono::time_point<std::chrono::high_resolution_clock> _now = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<Duration>(_now - _start) >= Duration(_time_out);
	}
	//reset start time
	inline void resetStartTime()
	{
		_start = std::chrono::high_resolution_clock::now();
	}
	//reset timeout
	inline void resetTimeOut(unsigned int time_out)
	{
		_time_out = time_out;
	}
    inline bool isConnected()
    {
        return connected;
    }

    inline void setConnected(bool s)
    {
        connected = s;
    }
 
private:
	//start time_poing
	std::chrono::time_point<std::chrono::high_resolution_clock> _start{ std::chrono::high_resolution_clock::now() };
	//time out
	unsigned int _time_out;
    bool connected = false;
};
 
#endif//__TIMEWATCHER_H__