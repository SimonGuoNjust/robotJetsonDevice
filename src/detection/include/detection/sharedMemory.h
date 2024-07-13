#pragma once
#include <iostream>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_recursive_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <memory>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <utility>
#define MAXBUFFSIZE 1280 * 720 * 3
using namespace boost::interprocess;
namespace Serializer = ros::serialization;

using ShareMemory = boost::interprocess::shared_memory_object;
using ShareMemoryRegion = boost::interprocess::mapped_region;
using ShareMemoryMutex = boost::interprocess::named_recursive_mutex;


struct SharedMsg
{
  int cnt;
  char buf[MAXBUFFSIZE];
};

template <typename MessageType>
class SharedMemoryROSCommunicator
{
public:
  explicit SharedMemoryROSCommunicator(std::string channel,
                                       std::size_t memory_size = MAXBUFFSIZE+ 8)
      : channel_(std::move(channel)), memory_size_(memory_size)
  {
    try
    {
      creator_ = false;
      sharedMemoryPtr_ = std::make_unique<ShareMemory>(
          open_only, channel_.c_str(), read_write);
    }
    catch (const boost::interprocess::interprocess_exception &ex)
    {
      creator_ = true;
      sharedMemoryPtr_ = std::make_unique<ShareMemory>(
          open_or_create, channel_.c_str(), read_write);
    }
    sharedMemoryPtr_->truncate(memory_size_);
    regionPtr_ =
        std::make_unique<ShareMemoryRegion>(*sharedMemoryPtr_, read_write);
    std::string mux_name = channel_ + "_mux";
    mutexPtr_ =
        std::make_unique<ShareMemoryMutex>(open_or_create, mux_name.c_str());
  }

  // for results
  bool publish(const float* decode_ptr_host, int res_len, const int& frame_cnt)
  {
    bool ec = mutexPtr_->try_lock();
    // ROS_INFO_STREAM(channel_ << " pub: " << ec);
    if (ec)
    {
      char* shared_memory_start = (char*)regionPtr_->get_address();
      memcpy(shared_memory_start, (const char*)&frame_cnt, 4);
      memcpy(shared_memory_start + 4, (const char*)decode_ptr_host, res_len);
      mutexPtr_->unlock();
    }
    return ec;
  }

  bool read(float* decode_ptr_host, int& res_cnt, int& frame_cnt)
  {
    bool ec = mutexPtr_->try_lock();
    // ROS_INFO_STREAM(channel_ << " pub: " << ec);
    if (ec)
    {
      char* shared_memory_start = (char*)regionPtr_->get_address();
      memcpy(&frame_cnt, shared_memory_start, 4);
      memcpy(&res_cnt, shared_memory_start + 4, 4);
      memcpy((char*)decode_ptr_host, shared_memory_start, 4 * (2 + res_cnt * 7));
      mutexPtr_->unlock();
    }
    else
    {
      ROS_INFO_STREAM("control node read error");
    }
    return ec;
  }
  // for image
  bool publish(const uint8_t* matPtr, const int& cnt)
  {
    // start_ = boost::posix_time::microsec_clock::local_time();
    // ROS_INFO_STREAM(dataPtr);
    bool ec = mutexPtr_->try_lock();
    // ROS_INFO_STREAM(channel_ << " pub: " << ec);
    if (ec)
    {
      memcpy((char*)regionPtr_->get_address(), (char*)&cnt, 4);
      memcpy((char*)regionPtr_->get_address() + 4, (char*)matPtr, MAXBUFFSIZE);
      mutexPtr_->unlock();
    }
    return ec;
  }

  bool read(SharedMsg* msgPtr)
  {
    // start_ = boost::posix_time::microsec_clock::local_time();
    // boost::shared_array<uint8_t> buffer(new uint8_t[memory_size_]);
    bool ec = mutexPtr_->try_lock();
    // ROS_INFO_STREAM(channel_ << " read: " << ec);
    if (ec)
    {
      memcpy((char*)msgPtr, regionPtr_->get_address(), MAXBUFFSIZE + 4);
      mutexPtr_->unlock();
      // Serializer::IStream stream(buffer.get(), memory_size_);
      // Serializer::deserialize(stream, msg);
    }
    return ec;
  }

  ~SharedMemoryROSCommunicator()
  {
    if (creator_)
    {
      sharedMemoryPtr_->remove(channel_.c_str());
      std::string mux_name = channel_ + "_mux";
      ShareMemoryMutex::remove(mux_name.c_str());
    }
  }

private:
  std::string channel_;
  std::size_t memory_size_;
  bool creator_;
  std::unique_ptr<ShareMemory> sharedMemoryPtr_;
  std::unique_ptr<ShareMemoryRegion> regionPtr_;
  std::unique_ptr<ShareMemoryMutex> mutexPtr_;
};
