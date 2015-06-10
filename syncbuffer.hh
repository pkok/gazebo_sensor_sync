#pragma once

#include "protobuf_helper.hh"

#include <boost/smart_ptr/shared_ptr.hpp>

#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <algorithm>
#include <tuple>
#include <utility> 

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

template <class... MsgTypes>
class SyncBuffer
{
  public:
    SyncBuffer(std::string filename) 
      : freshness(sizeof...(MsgTypes), false)
      //: fresh_buffer_imu(false), fresh_buffer_camera(false), fresh_buffer_pose(false)
    {
      open_file(filename);
    }

    template <std::size_t I, class T>
    void buffer(const boost::shared_ptr<const T>& msg)
    {
      std::get<I>(data) = T(*msg);
      freshness[I] = true;
      attempt_flush();
    }

  protected:
    virtual bool buffer_policy() 
    {
      return std::all_of(freshness.cbegin(), freshness.cend(), 
          [](bool is_fresh){ return is_fresh; });
    }

    virtual void open_file(std::string filename) 
    {
      int logfile_descriptor = open(filename.c_str(), O_WRONLY | O_TRUNC);
      logfile = new google::protobuf::io::FileOutputStream(logfile_descriptor);
      logfile->SetCloseOnDelete(true);
    }

    virtual void write_element(google::protobuf::MessageLite& msg)
    {
      writeDelimitedTo(msg, this->logfile);
    }

    bool attempt_flush() 
    {
      if (buffer_policy()) {
        for_each(data, 
                 std::bind(&SyncBuffer<MsgTypes...>::write_element,
                 &*this, 
                 std::placeholders::_1));
        logfile->Flush();
        freshness = std::vector<bool>(sizeof...(MsgTypes), false);
        return true;
      }
      return false;
    }

    google::protobuf::io::FileOutputStream* logfile;
    std::vector<bool> freshness;
    std::tuple<MsgTypes...> data;
};

template<std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type
  for_each(std::tuple<Tp...> &, FuncT) // Unused arguments are given no names.
  { }

template<std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I < sizeof...(Tp), void>::type
  for_each(std::tuple<Tp...>& t, FuncT f)
  {
    f(std::get<I>(t));
    for_each<I + 1, FuncT, Tp...>(t, f);
  }
