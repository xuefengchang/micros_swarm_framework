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
#include <boost/foreach.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include "ros/ros.h"

namespace micros_swarm_framework{
    //Robot position in the world coordinate system
    struct Base{
        float x;
        float y;
        float z;
            
        //linear velocity
        float vx;
        float vy;
        float vz;
        
        int valid;  //check if the data is valid. 0(unvalid), 1(valid), -1(unknown)
            
        Base() : x(0),y(0),z(0),vx(0),vy(0),vz(0),valid(-1){}
        Base(float x_,float y_,float z_,float vx_,float vy_,float vz_) : x(x_),y(y_),z(z_),vx(vx_),vy(vy_),vz(vz_),valid(-1){}
        Base(float x_,float y_,float z_,float vx_,float vy_,float vz_,int valid_) : x(x_),y(y_),z(z_),vx(vx_),vy(vy_),vz(vz_),valid(valid_){}
    };
    
    struct NeighborBase{
        //Polar coordinates of the neighbor robot with respect to the local robot
        float distance;
        float azimuth;
        float elevation;
        
        //Cartesian coordinates of the neighbor robot with respect to the local robot
        float x;
        float y;
        float z;
            
        //linear velocity
        float vx;
        float vy;
        float vz;
            
        NeighborBase():distance(0),azimuth(0),elevation(0),x(0),y(0),z(0),vx(0),vy(0),vz(0){}
        NeighborBase( float distance_,float azimuth_,float elevation_,float x_,float y_,float z_,float vx_,float vy_,float vz_)
            :distance(distance_),azimuth(azimuth_),elevation(elevation_),x(x_),y(y_),z(z_),vx(vx_),vy(vy_),vz(vz_){}
    };
    
    //This data structure contains all other robots's swarm information
    struct NeighborSwarmTuple{
        std::vector<int> swarm_id_vector;
        //when age_ is larger than the threshold, remove the tuple of the map 
        int age;
            
        NeighborSwarmTuple(const std::vector<int>& swarm_id_vector_, int age_):swarm_id_vector(swarm_id_vector_), age(age_){}
        ~NeighborSwarmTuple(){}
    
        
        void addSwarmID(int swarm_id)
        {
            swarm_id_vector.push_back(swarm_id);
        }
        
        void removeSwarmID(int swarm_id)
        {
            swarm_id_vector.erase(std::remove(swarm_id_vector.begin(), swarm_id_vector.end(), swarm_id), swarm_id_vector.end());
        }
        
        bool swarmIDExist(int swarm_id)
        {
            std::vector<int>::iterator it;
            it=std::find(swarm_id_vector.begin(), swarm_id_vector.end(), swarm_id);
    
            if(it!=swarm_id_vector.end())
                return true;
            
            return false; 
        }
    };
    
    struct VirtualStigmergyTuple{
        std::string vstig_value;
        time_t vstig_timestamp;
        //the id of the robot which last change the virtual stigmergy
        int robot_id;
            
        VirtualStigmergyTuple():vstig_value(""), vstig_timestamp(0), robot_id(-1){}
            
        VirtualStigmergyTuple(const std::string& value_, time_t time_, int id_)
            :vstig_value(value_), vstig_timestamp(time_), robot_id(id_){}
            
        void print()
        {
            std::cout<<vstig_value<<", "<<vstig_timestamp<<", "<<robot_id<<std::endl;
        }
    };
    
    //this macro definition is used to serialize the user-defined data type
    #define BOOST_SERIALIZE  template<class Archive>\
                             void serialize(Archive & ar, const unsigned int version)
                       
    #define MEMBER ar&
    
    struct TestVstigDataType{
        int a;
        float b;
        std::string c;
            
        BOOST_SERIALIZE
        {
            MEMBER a;
            MEMBER b;
            MEMBER c;
        }
            
        TestVstigDataType(){}
        TestVstigDataType(int a_, float b_, std::string c_)
        {
            a=a_;
            b=b_;
            c=c_;
        }
            
        void printTestVstigDataType()
        {
            std::cout<<"a = "<<a<<std::endl;
            std::cout<<"b = "<<b<<std::endl;
            std::cout<<"c = "<<c<<std::endl;
        }
    };
};

#endif
