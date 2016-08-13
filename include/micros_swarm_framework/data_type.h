/**
Software License Agreement (BSD)
\file      data_type.h
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

#ifndef DATA_TYPE_H_
#define DATA_TYPE_H_

#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/variant.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include "ros/ros.h"

namespace micros_swarm_framework{
    //Robot position in the world coordinate system
    class Base{
        private:
            float x_;
            float y_;
            float z_;
            
            //linear velocity
            float vx_;
            float vy_;
            float vz_;
        public:
            Base() : x_(-1),y_(-1),z_(-1),vx_(-1),vy_(-1),vz_(-1){}
            Base(float x,float y,float z,float vx,float vy,float vz) : x_(x),y_(y),z_(z),vx_(vx),vy_(vy),vz_(vz){}
            float getX(){return x_;}
            void setX(float x){x_=x;}
            
            float getY(){return y_;}
            void setY(float y){y_=y;}
            
            float getZ(){return z_;}
            void setZ(float z){z_=z;}
            
            float getVX(){return vx_;}
            void setVX(float vx){vx_=vx;}
            
            float getVY(){return vy_;}
            void setVY(float vy){vy_=vy;}
            
            float getVZ(){return vz_;}
            void setVZ(float vz){vz_=vz;}
    };
    
    class NeighborBase{
        private:
            //Polar coordinates of the neighbor robot with respect to the local robot
            float distance_;
            float azimuth_;
            float elevation_;
        
            //Cartesian coordinates of the neighbor robot with respect to the local robot
            float x_;
            float y_;
            float z_;
            
            //linear velocity
            float vx_;
            float vy_;
            float vz_;
        public:
            NeighborBase( float distance,float azimuth,float elevation,float x,float y,float z,float vx,float vy,float vz) \
              :distance_(distance),azimuth_(azimuth),elevation_(elevation),x_(x),y_(y),z_(z),vx_(vx),vy_(vy),vz_(vz){}
              
            float getDistance(){return distance_;}
            void setDistance(float distance){distance_=distance;}
            
            float getAzimuth(){return azimuth_;}
            void setAzimuth(float azimuth){azimuth_=azimuth;}
            
            float getElevation(){return elevation_;}
            void setElevation(float elevation){elevation_=elevation;}
            
            float getX(){return x_;}
            void setX(float x){x_=x;}
            
            float getY(){return y_;}
            void setY(float y){y_=y;}
            
            float getZ(){return z_;}
            void setZ(float z){z_=z;}
            
            float getVX(){return vx_;}
            void setVX(float vx){vx_=vx;}
            
            float getVY(){return vy_;}
            void setVY(float vy){vy_=vy;}
            
            float getVZ(){return vz_;}
            void setVZ(float vz){vz_=vz;}
    };
    
    //This data structure contains all other robots's swarm information
    class NeighborSwarmTuple{
        private:
            std::vector<int> swarm_id_vector_;
            //when age_ is larger than the threshold, remove the tuple of the map 
            int age_;
        public:
            NeighborSwarmTuple(std::vector<int> swarm_id_vector, int age):swarm_id_vector_(swarm_id_vector), age_(age){}
            ~NeighborSwarmTuple(){}
    
        void addSwarmID(int swarm_id)
        {
            swarm_id_vector_.push_back(swarm_id);
        }
        
        void removeSwarmID(int swarm_id)
        {
            swarm_id_vector_.erase(std::remove(swarm_id_vector_.begin(), swarm_id_vector_.end(), swarm_id), swarm_id_vector_.end());
        }
    
        std::vector<int> getSwarmIDVector()
        {
            return swarm_id_vector_;
        }
    
        void clearSwarmIDVector()
        {
            swarm_id_vector_.clear();
        }
    
        void setAge(int age)
        {
            age_=age;
        }
    
        int getAge()
        {
            return age_;
        }
        
        bool swarmIDExist(int swarm_id)
        {
            std::vector<int>::iterator it;
            it=std::find(swarm_id_vector_.begin(), swarm_id_vector_.end(), swarm_id);
    
            if(it!=swarm_id_vector_.end())
            {
                return true;
            }
            
            return false; 
        }
    };
    
    class VirtualStigmergyTuple{
        private:
            std::string vstig_value_;
            time_t vstig_timestamp_;
            //the id of the robot which last change the virtual stigmergy
            int robot_id_;
        public:
            VirtualStigmergyTuple( std::string value, time_t time, int id)\
              :vstig_value_(value), vstig_timestamp_(time), robot_id_(id){}

            std::string getVirtualStigmergyValue(){return vstig_value_;}
            void setVirtualStigmergyValue(std::string value)
            {
                vstig_value_=value;
            }
    
            time_t getVirtualStigmergyTimestamp(){return vstig_timestamp_;}
            void setVirtualStigmergyTimestamp(time_t time_now){vstig_timestamp_=time_now;}
    
            int getRobotID(){return robot_id_;}
            void setRobotID(int robot_id){robot_id_=robot_id;}
            
            void print()
            {
                std::cout<<vstig_value_<<", "<<vstig_timestamp_<<", "<<robot_id_<<std::endl;
            }
    };
    
    //this macro definition is used to serialize the user-defined data type
    #define BOOST_SERIALIZE  friend class boost::serialization::access;\
                       template<class Archive>\
                       void serialize(Archive & ar, const unsigned int version)
                       
    #define MEMBER ar&
    
    class TestVstigDataType{
        private:
            int a_;
            float b_;
            std::string c_;
            
            BOOST_SERIALIZE
            {
                MEMBER a_;
                MEMBER b_;
                MEMBER c_;
            }
            
        public:
            TestVstigDataType(){}
            TestVstigDataType(int a, float b, std::string c)
            {
                a_=a;
                b_=b;
                c_=c;
            }
            
            void printTestVstigDataType()
            {
                std::cout<<"a_ = "<<a_<<std::endl;
                std::cout<<"b_ = "<<b_<<std::endl;
                std::cout<<"c_ = "<<c_<<std::endl;
            }
    };
};

#endif
