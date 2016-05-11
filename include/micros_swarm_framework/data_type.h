/* 
 *  data_type.h - micros_swarm_framework data type
 *  Copyright (C) 2016 Xuefeng Chang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
            shm_string virtual_stigmergy_value_;  //使用string类型来存储value
            time_t virtual_stigmergy_timestamp_;  //虚拟共识结构最后修改的时间
            unsigned int robot_id_;  //最后修改该虚拟共识结构的机器人的id
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
    
    #define FUNCTION_OBJECT boost::function<void()>
    
    #define BOOL_FUNCTION_OBJECT boost::function<bool()>
    
    #define NEIGHBOR_FUNCTION_OBJECT(Type) boost::function<void(Type)>
    
    #define NB_VALUE _1 
    
    //this macro definition is used to bind the function and the parameter values
    #define BIND_FUNCTION_AND_PARAMETER_VALUES boost::bind
    
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
