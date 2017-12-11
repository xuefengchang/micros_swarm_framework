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
#include <time.h>
#include <math.h>
#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <ros/time.h>

namespace micros_swarm{
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
        //Base(float x_,float y_,float z_,float vx_,float vy_,float vz_) : x(x_),y(y_),z(z_),vx(vx_),vy(vy_),vz(vz_),valid(-1){}
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
            it = std::find(swarm_id_vector.begin(), swarm_id_vector.end(), swarm_id);
    
            if(it != swarm_id_vector.end()) {
                return true;
            }
            
            return false; 
        }
    };
    
    struct VirtualStigmergyTuple{
        std::vector<uint8_t> vstig_value;
        int lamport_clock;
        time_t write_timestamp;
        unsigned int read_count;
        //the id of the robot which last change the virtual stigmergy
        int robot_id;
            
        VirtualStigmergyTuple(): lamport_clock(0), write_timestamp(0), read_count(0), robot_id(-1){}
            
        VirtualStigmergyTuple(const std::vector<uint8_t>& value_, int clock_, time_t time_, unsigned int count_, int id_)
            :vstig_value(value_), lamport_clock(clock_), write_timestamp(time_), read_count(count_), robot_id(id_){}
            
        void print()
        {
            for(int i = 0; i < vstig_value.size(); i++) {
                std::cout<<vstig_value[i];
            }
            std::cout<<", "<<lamport_clock<<", "<<write_timestamp<<", "<<read_count<<", "<<robot_id<<std::endl;
        }

        double getTemperature()
        {
            double Pw, Pr;
            time_t t = time(NULL) - write_timestamp;
            Pw = pow(2,  - (t - (t % 5)) / 5);
            if(read_count >= 100) {
                Pr = 1;
            }
            else {
                Pr = read_count/100.0;
            }

            //double P = 0.5*Pw + 0.5*Pr;
            double P = Pw + Pr;
            //std::cout<<"<"<<Pw<<", "<<Pr<<">"<<std::endl;
            return P;
        }
    };

    struct BlackBoardTuple{
        std::vector<uint8_t> bb_value;
        ros::Time timestamp;
        //the id of the robot which last change the blackboard
        int robot_id;

        BlackBoardTuple(): timestamp(ros::Time(0,0)), robot_id(-1){}

        BlackBoardTuple(const std::vector<uint8_t>& value_, const ros::Time& timestamp_, int id_)
                :bb_value(value_), timestamp(timestamp_), robot_id(id_){}

        void print()
        {
            for(int i = 0; i < bb_value.size(); i++) {
                std::cout<<bb_value[i];
            }
            std::cout<<", "<<timestamp<<", "<<robot_id<<std::endl;
        }
    };

    struct SCDSPSODataTuple{
        std::vector<float> pos;
        float val;
        int robot_id;
        int gen;
        time_t timestamp;

        SCDSPSODataTuple():val(0), robot_id(0), gen(0), timestamp(0){}

        SCDSPSODataTuple(const std::string& key_, const std::vector<float> pos_, float val_, int robot_id_, int gen_, time_t timestamp_)
                :pos(pos_), val(val_), robot_id(robot_id_), gen(gen_), timestamp(timestamp_){}

        void print()
        {
            for(int i = 0; i < pos.size(); i++) {
                std::cout<<pos[i]<< " ";
            }
            std::cout<<", "<<val<<", "<<robot_id<<", "<<gen<<timestamp<<std::endl;
        }
    };
};

#endif
