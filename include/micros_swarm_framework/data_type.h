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
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
 
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/allocators/allocator.hpp> 
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/offset_ptr.hpp>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <boost/thread.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp> 

#include "ros/ros.h"

namespace micros_swarm_framework{
    
    //shared memory data type and allocator definition.
    typedef boost::interprocess::managed_shared_memory::segment_manager  segment_manager_t;  

    typedef boost::interprocess::allocator<void, segment_manager_t>  VoidAllocator;  
    typedef boost::interprocess::allocator<int, segment_manager_t>  IntAllocator;  
    typedef boost::interprocess::vector<int, IntAllocator>  shm_int_vector;  

    typedef boost::interprocess::allocator<shm_int_vector, segment_manager_t>  IntVectorAllocator;  
    typedef boost::interprocess::vector<shm_int_vector, IntVectorAllocator>  shm_int_vector_vector;  

    typedef boost::interprocess::allocator<char, segment_manager_t>  CharAllocator;  
    typedef boost::interprocess::basic_string<char, std::char_traits<char>, CharAllocator>  shm_string;
    
    //Robot position in the world coordinate system
    class Location{
        private:
            float x_;
            float y_;
            float z_;
        public:
            Location() : x_(-1),y_(-1),z_(-1){}
            Location(float x,float y,float z) : x_(x),y_(y),z_(z){}
            float getX(){return x_;}
            void setX(float x){x_=x;}
            
            float getY(){return y_;}
            void setY(float y){y_=y;}
            
            float getZ(){return z_;}
            void setZ(float z){z_=z;}
    };
    
    class NeighborLocation{
        private:
            //Polar coordinates of the neighbor robot with respect to the local robot
            float distance_;
            float azimuth_;
            float elevation_;
        
            //Cartesian coordinates of the neighbor robot with respect to the local robot
            float x_;
            float y_;
            float z_;
        public:
            NeighborLocation( float distance,float azimuth,float elevation,float x,float y,float z) \
              :distance_(distance),azimuth_(azimuth),elevation_(elevation),x_(x),y_(y),z_(z){}
              
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
    };
    
    typedef std::pair<const unsigned int, NeighborLocation>  NeighborLocationType;  
    typedef boost::interprocess::allocator<NeighborLocationType, segment_manager_t>  NeighborLocationTypeAllocator;
    typedef boost::interprocess::map<unsigned int, NeighborLocation, std::less<unsigned int>, NeighborLocationTypeAllocator>  shm_neighbors_type;

    typedef std::pair<const unsigned int, bool>  SwarmsType;  
    typedef boost::interprocess::allocator<SwarmsType, segment_manager_t>  SwarmsTypeAllocator;
    typedef boost::interprocess::map<unsigned int, bool, std::less<unsigned int>, SwarmsTypeAllocator>  shm_swarms_type;
    
    //This data structure contains all other robots's swarm information
    class NeighborSwarm{
        private:
            shm_int_vector swarm_id_vector_;
            //when age_ is larger than the threshold, remove the tuple of the map 
            unsigned int age_;
        public:
            NeighborSwarm(const VoidAllocator &void_alloc, unsigned int age):swarm_id_vector_(void_alloc), age_(age){}
            ~NeighborSwarm(){}
    
        void addSwarmID(unsigned int swarm_id)
        {
            swarm_id_vector_.push_back(swarm_id);
        }
        
        void removeSwarmID(unsigned int swarm_id)
        {
            shm_int_vector::iterator iv_it;
            for(iv_it=swarm_id_vector_.begin();iv_it!=swarm_id_vector_.end();iv_it++)
            {
                if((*iv_it)==swarm_id)
                    iv_it = swarm_id_vector_.erase(iv_it);
                else
                    iv_it++;
            }
        }
    
        shm_int_vector getSwarmIDVector()
        {
            return swarm_id_vector_;
        }
    
        void clearSwarmIDVector()
        {
            swarm_id_vector_.clear();
        }
    
        void setAge(unsigned int age)
        {
            age_=age;
        }
    
        unsigned int getAge()
        {
            return age_;
        }
        
        bool swarmIDExist(unsigned int swarm_id)
        {
            for(int i=0;i<swarm_id_vector_.size();i++)
            {
                if(swarm_id_vector_[i]==swarm_id)
                    return true;
            }
            return false;
        }
    };

    typedef std::pair<const unsigned int, NeighborSwarm>  NeighborSwarmType;  
    typedef boost::interprocess::allocator<NeighborSwarmType, segment_manager_t>  NeighborSwarmTypeAllocator;
    typedef boost::interprocess::map<unsigned int, NeighborSwarm, std::less<unsigned int>, NeighborSwarmTypeAllocator>  shm_neighbor_swarm_type;
    
    class ShmVirtualStigmergyTuple{
        private:
            shm_string virtual_stigmergy_value_;
            time_t virtual_stigmergy_timestamp_;
            unsigned int robot_id_;
        public:
            ShmVirtualStigmergyTuple(shm_string value, time_t time, unsigned int id, const VoidAllocator &void_alloc)\
              :virtual_stigmergy_value_(value, void_alloc), virtual_stigmergy_timestamp_(time), robot_id_(id){}
             
            shm_string getVirtualStigmergyValue(){return virtual_stigmergy_value_;}
            void setVirtualStigmergyValue(shm_string value)
            {
                virtual_stigmergy_value_=value;
            }
    
            time_t getVirtualStigmergyTimestamp(){return virtual_stigmergy_timestamp_;}
            void setVirtualStigmergyTimestamp(time_t time_now){virtual_stigmergy_timestamp_=time_now;}
    
            unsigned int getRobotID(){return robot_id_;}
            void setRobotID(unsigned int robot_id){robot_id_=robot_id;}
    };

    typedef std::pair<const shm_string, ShmVirtualStigmergyTuple>  ShmVirtualStigmergyTupleType;  
    typedef boost::interprocess::allocator<ShmVirtualStigmergyTupleType, segment_manager_t>  ShmVirtualStigmergyTupleTypeAllocator;
    typedef boost::interprocess::map<shm_string, ShmVirtualStigmergyTuple, std::less<shm_string>, ShmVirtualStigmergyTupleTypeAllocator>  shm_virtual_stigmergy_tuple_type;

    typedef std::pair<const unsigned int, shm_virtual_stigmergy_tuple_type>  VirtualStigmergyType;  
    typedef boost::interprocess::allocator<VirtualStigmergyType, segment_manager_t>  VirtualStigmergyTypeAllocator;
    typedef boost::interprocess::map<unsigned int, shm_virtual_stigmergy_tuple_type, std::less<unsigned int>, VirtualStigmergyTypeAllocator>  shm_virtual_stigmergy_type;
    
    class VstigTuple{
        private:
            std::string vstig_value_;
            time_t vstig_timestamp_;
            //the id of the robot which last change the virtual stigmergy
            unsigned int robot_id_;
        public:
            VstigTuple( std::string value, time_t time, unsigned int id)\
              :vstig_value_(value), vstig_timestamp_(time), robot_id_(id){}

            std::string getVstigValue(){return vstig_value_;}
            void setVstigValue(std::string value)
            {
                vstig_value_=value;
            }
    
            time_t getVstigTimestamp(){return vstig_timestamp_;}
            void setVstigTimestamp(time_t time_now){vstig_timestamp_=time_now;}
    
            unsigned int getRobotID(){return robot_id_;}
            void setRobotID(unsigned int robot_id){robot_id_=robot_id;}
    };
    
    class CheckNeighborABC{
        public:
            virtual bool isNeighbor(Location self, Location neighbor)=0;
    };
    
    class CheckNeighbor : public CheckNeighborABC{
        public:
            bool isNeighbor(Location self, Location neighbor)
            {
                float distance=sqrt((self.getX()-neighbor.getX())*(self.getX()-neighbor.getX())+(self.getY()-neighbor.getY())*(self.getY()-neighbor.getY())+ \
                    (self.getZ()-neighbor.getZ())*(self.getZ()-neighbor.getZ()));
                    
                if(distance<50)
                    return true;
                    
                return false;
            }
    };
    
    //this macro definition is used to serialize the user-defined data type
    #define SERIALIZE  friend class boost::serialization::access;\
                       template<class Archive>\
                       void serialize(Archive & ar, const unsigned int version)
                       
    #define MEMBER ar&
    
    class TestVstigDataType{
        private:
            int a_;
            float b_;
            std::string c_;
            
            SERIALIZE
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
