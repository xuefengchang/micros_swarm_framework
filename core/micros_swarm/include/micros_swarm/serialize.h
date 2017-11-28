/**
Software License Agreement (BSD)
\file      serialize.h
\authors Xuefeng Chang <changxuefengcn@163.com>
\copyright Copyright (c) 2016, the micROS Team, HPCL (National University of Defense Technology), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of micROS Team, HPCL, nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SERIALIZE_H_
#define SERIALIZE_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>

namespace micros_swarm{

    // serializer using ROS.
    template<class T>
    std::vector<uint8_t> serialize_ros(T t)
    {
        std::vector<uint8_t> vec;
        uint32_t serial_size = ros::serialization::serializationLength(t);
        boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
        ros::serialization::OStream ostream(buffer.get(), serial_size);
        ros::serialization::serialize(ostream, t);
        vec.resize(serial_size);
        std::copy(buffer.get(), buffer.get() + serial_size, vec.begin());
        return vec;
    }
    // deserializer using ROS.
    template<class T>
    T deserialize_ros(const std::vector<uint8_t>& vec)
    {
        T t;
        uint32_t serial_size = vec.size();
        std::vector<uint8_t> buffer(serial_size);
        std::copy(vec.begin(), vec.begin() + serial_size, buffer.begin());
        ros::serialization::IStream istream(buffer.data(), serial_size);
        ros::serialization::Serializer<T>::read(istream, t);
        return t;
    }

    // serializer using Boost.
    template<class T>
    std::string serialize_boost(T t)
    {
        std::ostringstream archiveStream;
        boost::archive::text_oarchive archive(archiveStream);
        archive<<t;
        return archiveStream.str();
    }
    // deserializer using Boost.
    template<class T>
    T deserialize_boost(std::string str)
    {
        T t;
        std::istringstream archiveStream(str);
        boost::archive::text_iarchive archive(archiveStream);
        archive>>t;
        return t;
    }
};
#endif
