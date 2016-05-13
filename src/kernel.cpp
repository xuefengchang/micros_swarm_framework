/**
Software License Agreement (BSD)
\file      kernel.cpp 
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

#include "ros/ros.h"

#include "micros_swarm_framework/kernel.h"

namespace micros_swarm_framework{

    int micros_swarm_framework::KernelInitializer::unique_robot_id_ = -1;
    
    KernelInitializer::KernelInitializer()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
         
        boost::interprocess::shared_memory_object::remove(shm_object_name.data());
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        try{
             unsigned int *i_p = segment.construct<unsigned int>("shm_robot_id_")();
        }catch(boost::interprocess::bad_alloc &ex){
             std::cerr << ex.what() << std::endl;
        }
        
        try{
             unsigned int *t_p = segment.construct<unsigned int>("shm_robot_type_")(); 
        }catch(boost::interprocess::bad_alloc &ex){
             std::cerr << ex.what() << std::endl;
        }
        
        try{
             unsigned int *s_p = segment.construct<unsigned int>("shm_robot_status_")(); 
        }catch(boost::interprocess::bad_alloc &ex){
             std::cerr << ex.what() << std::endl;
        }
        
        try{
             Location *l_p = segment.construct<Location>("shm_robot_location_")(); 
        }catch(boost::interprocess::bad_alloc &ex){
             std::cerr << ex.what() << std::endl;
        }
        
        try{
            shm_neighbors_type *snt_p = segment.construct<shm_neighbors_type>("shm_neighbors_")(std::less<unsigned int>(), alloc_inst);
        }catch(boost::interprocess::bad_alloc &ex){
             std::cerr << ex.what() << std::endl;
        }
        
        try{
            shm_swarms_type *sst_p = segment.construct<shm_swarms_type>("shm_swarms_")(std::less<unsigned int>(), alloc_inst);
        }catch(boost::interprocess::bad_alloc &ex){
             std::cerr << ex.what() << std::endl;
        }
        
        try{
            shm_neighbor_swarm_type *snst_p=segment.construct<shm_neighbor_swarm_type>("shm_neighbor_swarm_")(std::less<unsigned int>(), alloc_inst);
        }catch(boost::interprocess::bad_alloc &ex){
            std::cerr << ex.what() << std::endl;
        }
        
        try{
            shm_virtual_stigmergy_type *svstt_p=segment.construct<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_")(std::less<unsigned int>(), alloc_inst);
        }catch(boost::interprocess::bad_alloc &ex){
            std::cerr << ex.what() << std::endl;
        }
        
        ros::NodeHandle n;
        packet_subscriber_ = n.subscribe("/micros_swarm_framework_topic", 1000, &KernelInitializer::packetCallback, this);
        
    }
    
    void KernelInitializer::initRobotID(int robot_id)
    {
        if(robot_id<0)
        {
            std::cout<<"could not set id as negative"<<std::endl;
            exit(1);
        }
        else
        {
            unique_robot_id_ = robot_id;
        }
    }
    
    void KernelInitializer::PacketParser(const micros_swarm_framework::MSFPPacket& msfp_packet)
    {
        micros_swarm_framework::KernelHandle kh;
        unsigned int shm_rid=kh.getRobotID();
        
        //ignore the packet of the local robot
        if(msfp_packet.packet_source==shm_rid)
        {
            return;
        }
    
        const unsigned int packet_type=msfp_packet.packet_type;
        std::string packet_data=msfp_packet.packet_data;
        
        std::istringstream archiveStream(packet_data);
        boost::archive::text_iarchive archive(archiveStream);
        
        switch(packet_type)
        {
            case SINGLE_ROBOT_BROADCAST_ID:{
                micros_swarm_framework::SingleRobotBroadcastID srbi;
                
                archive>>srbi;
                
                unsigned int robot_id=srbi.getRobotID();
                float distance=0;
                float azimuth=0;
                float elevation=0;
                float x=srbi.getRobotX();
                float y=srbi.getRobotY();
                float z=srbi.getRobotZ();
                
                Location self=kh.getRobotLocation();
                Location neighbor(x, y, z);
                
                CheckNeighbor cn;
                micros_swarm_framework::CheckNeighborABC *cn_p=&cn;
                
                if(cn_p->isNeighbor(self, neighbor))
                {
                    kh.insertOrUpdateNeighbor(robot_id, distance, azimuth, elevation, x, y, z);
                }
                else
                {
                    kh.deleteNeighbor(robot_id);
                }
                
                break;
            }
            case SINGLE_ROBOT_JOIN_SWARM:{
                SingleRobotJoinSwarm srjs;
                archive>>srjs;
             
                unsigned int robot_id=srjs.getRobotID();
                unsigned int swarm_id=srjs.getSwarmID();
                kh.joinNeighborSwarm(robot_id, swarm_id);
                
                break;
            }
            case SINGLE_ROBOT_LEAVE_SWARM:
            {
                SingleRobotLeaveSwarm srls;
                archive>>srls;
                
                unsigned int robot_id=srls.getRobotID();
                unsigned int swarm_id=srls.getSwarmID();
                kh.leaveNeighborSwarm(robot_id, swarm_id);
                
                break;
            }
            case SINGLE_ROBOT_SWARM_LIST:
            {
                SingleRobotSwarmList srsl;
                archive>>srsl;
                
                unsigned int robot_id=srsl.getRobotID();
                std::vector<unsigned int> swarm_list=srsl.getSwarmList();
                kh.insertOrRefreshNeighborSwarm(robot_id, swarm_list);
                
                break;
            }
            case VIRTUAL_STIGMERGY_QUERY:
            {
                VirtualStigmergyQuery vsq;
                archive>>vsq;
                
                unsigned int id=vsq.getVirtualStigmergyID();
                std::string key_std=vsq.getVirtualStigmergyKey();
                std::string value_std=vsq.getVirtualStigmergyValue();
                time_t time_now=vsq.getVirtualStigmergyTimestamp();
                unsigned int robot_id=vsq.getRobotID();
                
                VstigTuple local=kh.getVirtualStigmergyTuple(id, key_std);
                
                //local tuple is not exist or the local timestamp is smaller
                if((local.getVstigTimestamp()==0)|| \
                   (local.getVstigTimestamp()<time_now))
                {
                    kh.createVirtualStigmergy(id);
                    kh.insertOrUpdateVirtualStigmergy(id, key_std, value_std, time_now, robot_id);
                    
                    VirtualStigmergyPut vsp(id, key_std, value_std, time_now, robot_id);
                    
                    std::ostringstream archiveStream;
                    boost::archive::text_oarchive archive(archiveStream);
                    archive<<vsp;
                    std::string vsp_string=archiveStream.str();
                    
                    micros_swarm_framework::MSFPPacket p;
                    p.packet_source=shm_rid;
                    p.packet_version=1;
                    p.packet_type=VIRTUAL_STIGMERGY_PUT;
                    p.packet_data=vsp_string;
                    p.package_check_sum=0;
                    
                    kh.publishPacket(p);
                }
                else if(local.getVstigTimestamp()>time_now)  //local timestamp is larger
                {
                    VirtualStigmergyPut vsp(id, key_std, local.getVstigValue(), local.getVstigTimestamp(), local.getRobotID());
                    std::ostringstream archiveStream;
                    boost::archive::text_oarchive archive(archiveStream);
                    archive<<vsp;
                    std::string vsp_string=archiveStream.str();
                    
                    micros_swarm_framework::MSFPPacket p;
                    p.packet_source=shm_rid;
                    p.packet_version=1;
                    p.packet_type=VIRTUAL_STIGMERGY_PUT;
                    p.packet_data=vsp_string;
                    p.package_check_sum=0;
                    
                    kh.publishPacket(p);
                }
                else if((local.getVstigTimestamp()==time_now)&& \
                        (local.getRobotID()!=robot_id))
                {
                    std::cout<<"query conflict"<<std::endl;
                }
                else
                {
                    //do nothing
                }
                
                break;
            }
            case VIRTUAL_STIGMERGY_PUT:
            {
                VirtualStigmergyPut vsp;
                archive>>vsp;
                
                unsigned int id=vsp.getVirtualStigmergyID();
                std::string key_std=vsp.getVirtualStigmergyKey();
                std::string value_std=vsp.getVirtualStigmergyValue();
                time_t time_now=vsp.getVirtualStigmergyTimestamp();
                unsigned int robot_id=vsp.getRobotID();
                
                VstigTuple local=kh.getVirtualStigmergyTuple(id, key_std);
                
                //local tuple is not exist or local timestamp is smaller
                if((local.getVstigTimestamp()==0)|| \
                   (local.getVstigTimestamp()<time_now))
                {
                    kh.createVirtualStigmergy(id);
                
                    kh.insertOrUpdateVirtualStigmergy(id, key_std, value_std, time_now, robot_id);
                    
                    VirtualStigmergyPut vsp(id, key_std, value_std, time_now, robot_id);
                    std::ostringstream archiveStream2;
                    boost::archive::text_oarchive archive2(archiveStream2);
                    archive2<<vsp;
                    std::string vsp_string=archiveStream.str();
                    
                    micros_swarm_framework::MSFPPacket p;
                    p.packet_source=shm_rid;
                    p.packet_version=1;
                    p.packet_type=VIRTUAL_STIGMERGY_PUT;
                    p.packet_data=vsp_string;
                    p.package_check_sum=0;
                    
                    kh.publishPacket(p);
                }
                else if((local.getVstigTimestamp()==time_now)&& \
                        (local.getRobotID()!=robot_id))
                {
                    std::cout<<"put conflict"<<std::endl;
                }
                else
                {
                    //do nothing
                }
                break;
            }
            
            default:
            {
                std::cout<<"UNDEFINED PACKET TYPE!"<<std::endl;
            }
        }
    }
    
    KernelInitializer::~KernelInitializer()
    {
        //std::cout<<" destruct KernelData"<<unique_robot_id_<<std::endl;
        
        std::string robot_id_string=boost::lexical_cast<std::string>(unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        
        boost::interprocess::shared_memory_object::remove(shm_object_name.data());
        boost::interprocess::named_mutex::remove(mutex_name.data());
    }
    
    unsigned int KernelHandle::getRobotID()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<unsigned int*, std::size_t> kernel_robot_id = segment.find<unsigned int>("shm_robot_id_"); 
        if(kernel_robot_id.first==0)
        {
            //return 0;
            return INT_MAX;
        }
        
        return *(kernel_robot_id.first);
    }
    
    void KernelHandle::setRobotID(unsigned int robot_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<unsigned int*, std::size_t> kernel_robot_id = segment.find<unsigned int>("shm_robot_id_"); 
        if(kernel_robot_id.first==0)
        {
            return;
        }
        
        named_kernel_mtx.lock();
        *(kernel_robot_id.first)=robot_id;
        named_kernel_mtx.unlock();
    }
    
    Location KernelHandle::getRobotLocation()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<Location*, std::size_t> kernel_robot_location = segment.find<Location>("shm_robot_location_"); 
        if(kernel_robot_location.first==0)
        {
            Location l(-1,-1,-1);
            return l;
        }
        
        return *(kernel_robot_location.first);
    }
    void KernelHandle::setRobotLocation(Location robot_location)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<Location*, std::size_t> kernel_robot_location = segment.find<Location>("shm_robot_location_"); 
        if(kernel_robot_location.first==0)
        {
            return;
        }
        
        named_kernel_mtx.lock();
        *(kernel_robot_location.first)=robot_location;
        named_kernel_mtx.unlock();
    }
     
    std::map<unsigned int, NeighborLocation> KernelHandle::getNeighbors()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        std::map<unsigned int, NeighborLocation> result;
    
        std::pair<shm_neighbors_type*, std::size_t> neighbors = segment.find<shm_neighbors_type>("shm_neighbors_"); 
        if(neighbors.first==0)
        {
            result.clear();
            return result;
        }
  
        shm_neighbors_type *n_pointer=neighbors.first;
        shm_neighbors_type::iterator n_it;
        
        for(n_it=n_pointer->begin(); n_it!=n_pointer->end(); n_it++)
        {
            NeighborLocation neighbor_location=n_it->second;
            result.insert(std::pair<unsigned int, NeighborLocation>(n_it->first,neighbor_location));
        }
    
        return result;
    }
     
    void KernelHandle::insertOrUpdateNeighbor(unsigned int robot_id, float distance, float azimuth, float elevation, float x, float y, float z)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_neighbors_type*, std::size_t> neighbors = segment.find<shm_neighbors_type>("shm_neighbors_"); 
        if(neighbors.first==0)
        {
            return;
        }
  
        shm_neighbors_type *n_pointer=neighbors.first;
        shm_neighbors_type::iterator n_it;
        n_it=n_pointer->find(robot_id);
    
        if(n_it!=n_pointer->end())
        {
            NeighborLocation new_neighbor_location(distance, azimuth, elevation, x, y, z);
            
            named_kernel_mtx.lock();
            n_it->second = new_neighbor_location;
            named_kernel_mtx.unlock();   
        }
        else
        {
            NeighborLocation new_neighbor_location(distance, azimuth, elevation, x, y, z);
            NeighborLocationType new_type(robot_id, new_neighbor_location);    
            
            named_kernel_mtx.lock();
            n_pointer->insert(new_type);
            named_kernel_mtx.unlock();
        }
    }
    
    void KernelHandle::deleteNeighbor(unsigned int robot_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());  
    
        std::pair<shm_neighbors_type*, std::size_t> neighbors = segment.find<shm_neighbors_type>("shm_neighbors_"); 
        if(neighbors.first==0)
        {
            return;
        }
  
        shm_neighbors_type *n_pointer=neighbors.first;
    
        named_kernel_mtx.lock();
        n_pointer->erase(robot_id);
        named_kernel_mtx.unlock();
    }
    
    void KernelHandle::printNeighbor()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        std::pair<shm_neighbors_type*, std::size_t> neighbors = segment.find<shm_neighbors_type>("shm_neighbors_"); 
        if(neighbors.first==0)
        {
            return;
        }
        
        shm_neighbors_type *n_pointer=neighbors.first;
        shm_neighbors_type::iterator n_it;
        
        for (n_it=n_pointer->begin(); n_it!=n_pointer->end(); n_it++)
        {
            std::cout<<n_it->first<<": ";
            
            std::cout<<n_it->second.getDistance()<<","<<n_it->second.getAzimuth()<<","<<n_it->second.getElevation()<<","<<\
                n_it->second.getX()<<","<<n_it->second.getY()<<","<<n_it->second.getZ();
            std::cout<<std::endl;
        }
    }
     
    void KernelHandle::insertOrUpdateSwarm(unsigned int swarm_id, bool value)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_swarms_type*, std::size_t> swarms = segment.find<shm_swarms_type>("shm_swarms_"); 
        if(swarms.first==0)
        {
            return;
        }
  
        shm_swarms_type *s_pointer=swarms.first;
        shm_swarms_type::iterator s_it;
        s_it=s_pointer->find(swarm_id);
    
        if(s_it!=s_pointer->end())
        {
            named_kernel_mtx.lock();
            s_it->second = value;
            named_kernel_mtx.unlock();
            return;    
        }
        else
        {
            SwarmsType new_type(swarm_id, value);
            
            named_kernel_mtx.lock();
            s_pointer->insert(new_type);
            named_kernel_mtx.unlock();
            return;
        }
    }
    
    bool KernelHandle::getSwarm(unsigned int swarm_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        //std::string mutex_name="named_kernel_mtx"+robot_id_string;
        //boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_swarms_type*, std::size_t> swarms = segment.find<shm_swarms_type>("shm_swarms_"); 
        if(swarms.first==0)
        {
            return false;
        }
  
        shm_swarms_type *s_pointer=swarms.first;
        shm_swarms_type::iterator s_it;
        s_it=s_pointer->find(swarm_id);
    
        if(s_it!=s_pointer->end())
        {
            return s_it->second;
        }
        else
        {
            return false;
        }
    
        return false;
    }
    
    std::vector<unsigned int> KernelHandle::getSwarmList()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        //std::string mutex_name="named_kernel_mtx"+robot_id_string;
        //boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::vector<unsigned int> result;
        result.clear();
        
        std::pair<shm_swarms_type*, std::size_t> swarms = segment.find<shm_swarms_type>("shm_swarms_"); 
        if(swarms.first==0)
        {
            return result;
        }
  
        shm_swarms_type *s_pointer=swarms.first;
        shm_swarms_type::iterator s_it;
        
        for(s_it=s_pointer->begin();s_it!=s_pointer->end();s_it++)
        {
            if(s_it->second)
                result.push_back(s_it->first);
        }
    
        return result;
    }
    
    void KernelHandle::deleteSwarm(unsigned int swarm_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());  
    
        std::pair<shm_swarms_type*, std::size_t> swarms = segment.find<shm_swarms_type>("shm_swarms_"); 
        if(swarms.first==0)
        {
            return;
        }
  
        shm_swarms_type *s_pointer=swarms.first;
    
        named_kernel_mtx.lock();
        s_pointer->erase(swarm_id);
        named_kernel_mtx.unlock();
    }
    
    void KernelHandle::printSwarm()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        std::pair<shm_swarms_type*, std::size_t> swarms = segment.find<shm_swarms_type>("shm_swarms_"); 
        if(swarms.first==0)
        {
            return;
        }
        
        shm_swarms_type *s_pointer=swarms.first;
        shm_swarms_type::iterator s_it;
        
        for (s_it=s_pointer->begin(); s_it!=s_pointer->end(); s_it++)
        {
            std::cout<<s_it->first<<": ";
            
            std::cout<<s_it->second;
            std::cout<<std::endl;
        }
    }
    
    void KernelHandle::joinNeighborSwarm(unsigned int robot_id, unsigned int swarm_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_neighbor_swarm_type*, std::size_t> neighbor_swarm = segment.find<shm_neighbor_swarm_type>("shm_neighbor_swarm_"); 
        if(neighbor_swarm.first==0)
        {
            return;
        }
  
        shm_neighbor_swarm_type *ns_pointer=neighbor_swarm.first;
        shm_neighbor_swarm_type::iterator ns_it;
        ns_it=ns_pointer->find(robot_id);
    
        if(ns_it!=ns_pointer->end())
        {
            if(ns_it->second.swarmIDExist(swarm_id))
            {
                named_kernel_mtx.lock();
                ns_it->second.setAge(0);
                named_kernel_mtx.unlock();
            }
            else
            {
                named_kernel_mtx.lock();
                ns_it->second.addSwarmID(swarm_id);
                ns_it->second.setAge(0);
                named_kernel_mtx.unlock();
            }
        }
        else
        {
            NeighborSwarm new_neighbor_swarm(alloc_inst, 0);
            new_neighbor_swarm.addSwarmID(swarm_id);
            NeighborSwarmType new_type(robot_id, new_neighbor_swarm);
            
            named_kernel_mtx.lock();
            ns_pointer->insert(new_type);
            named_kernel_mtx.unlock();
        }
    
    }
    
    void KernelHandle::leaveNeighborSwarm(unsigned int robot_id, unsigned int swarm_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_neighbor_swarm_type*, std::size_t> neighbor_swarm = segment.find<shm_neighbor_swarm_type>("shm_neighbor_swarm_"); 
        if(neighbor_swarm.first==0)
        {
            return;
        }
  
        shm_neighbor_swarm_type *ns_pointer=neighbor_swarm.first;
        shm_neighbor_swarm_type::iterator ns_it;
        ns_it=ns_pointer->find(robot_id);
    
        if(ns_it!=ns_pointer->end())
        {
            if(ns_it->second.swarmIDExist(swarm_id))
            {
                named_kernel_mtx.lock();
                ns_it->second.removeSwarmID(swarm_id);
                ns_it->second.setAge(0);
                named_kernel_mtx.unlock();
            }
            else
            {
                std::cout<<"robot"<<robot_id<<"is not in swarm"<<swarm_id<<"."<<std::endl;
            }
        }
        else  //不存在
        {
            std::cout<<"robot_id "<<robot_id<<" neighbor_swarm tuple is not exist."<<std::endl;
            return;
        }
    
    }
            
    void KernelHandle::insertOrRefreshNeighborSwarm(unsigned int robot_id, std::vector<unsigned int> swarm_list)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_neighbor_swarm_type*, std::size_t> neighbor_swarm = segment.find<shm_neighbor_swarm_type>("shm_neighbor_swarm_"); 
        if(neighbor_swarm.first==0)
        {
            return;
        }
  
        shm_neighbor_swarm_type *ns_pointer=neighbor_swarm.first;
        shm_neighbor_swarm_type::iterator ns_it;
        ns_it=ns_pointer->find(robot_id);
    
        if(ns_it!=ns_pointer->end())
        {
            named_kernel_mtx.lock();
            ns_it->second.clearSwarmIDVector();
            
            for(int i=0;i<swarm_list.size();i++)
            {
                ns_it->second.addSwarmID(swarm_list[i]);
            }
            ns_it->second.setAge(0);  //age置为0
            
            named_kernel_mtx.unlock();
        }
        else
        {
            NeighborSwarm new_neighbor_swarm(alloc_inst, 0);
            
            for(int i=0;i<swarm_list.size();i++)
            {
                new_neighbor_swarm.addSwarmID(swarm_list[i]);
            }
            
            NeighborSwarmType new_type(robot_id, new_neighbor_swarm);
            
            named_kernel_mtx.lock();
            ns_pointer->insert(new_type);
            named_kernel_mtx.unlock();
            return;
        }
    }
    
    std::set<unsigned int> KernelHandle::getSwarmMembers(unsigned int swarm_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        //std::string mutex_name="named_kernel_mtx"+robot_id_string;
        //boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::set<unsigned int> result;
        result.clear();
    
        std::pair<shm_neighbor_swarm_type*, std::size_t> neighbor_swarm = segment.find<shm_neighbor_swarm_type>("shm_neighbor_swarm_"); 
        if(neighbor_swarm.first==0)
        {
            return result;
        }
  
        shm_neighbor_swarm_type *ns_pointer=neighbor_swarm.first;
        shm_neighbor_swarm_type::iterator ns_it;
        
        for(ns_it=ns_pointer->begin(); ns_it!=ns_pointer->end(); ns_it++)
        {
            shm_int_vector tmp=ns_it->second.getSwarmIDVector();
            
            if (std::find(tmp.begin(), tmp.end(), swarm_id) != tmp.end())
                result.insert(ns_it->first);
        }
        
        return result;
    }
    
    void KernelHandle::deleteNeighborSwarm(unsigned int robot_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());  
    
        std::pair<shm_neighbor_swarm_type*, std::size_t> neighbor_swarm = segment.find<shm_neighbor_swarm_type>("shm_neighbor_swarm_"); 
        if(neighbor_swarm.first==0)
        {
            return;
        }
  
        shm_neighbor_swarm_type *ns_pointer=neighbor_swarm.first;
    
        named_kernel_mtx.lock();
        ns_pointer->erase(robot_id);
        named_kernel_mtx.unlock();
    }
    
    void KernelHandle::printNeighborSwarm()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        std::pair<shm_neighbor_swarm_type*, std::size_t> neighbor_swarm = segment.find<shm_neighbor_swarm_type>("shm_neighbor_swarm_"); 
        if(neighbor_swarm.first==0)
        {
            return;
        }
        
        shm_neighbor_swarm_type *nst_pointer=neighbor_swarm.first;
        shm_neighbor_swarm_type::iterator nst_it;
        
        for (nst_it=nst_pointer->begin(); nst_it!=nst_pointer->end(); nst_it++)
        {
            std::cout<<"neighbor swarm "<<nst_it->first<<": ";
            
            shm_int_vector temp=nst_it->second.getSwarmIDVector();
            for(int i=0;i<temp.size();i++)
            {
                std::cout<<temp[i]<<",";
            }
            std::cout<<"age: "<<nst_it->second.getAge();
            std::cout<<std::endl;
        }
    }
            
    void KernelHandle::createVirtualStigmergy(unsigned int id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        
        std::string id_string=boost::lexical_cast<std::string>(id);
        std::string virtual_stigmergy_name="shm_virtual_stigmergy_tuple_"+id_string;
        const char *p=virtual_stigmergy_name.data();
        
        VoidAllocator alloc_inst (segment.get_segment_manager());  
    
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            return;
        }
  
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
        shm_virtual_stigmergy_type::iterator vst_it;
        vst_it=vst_pointer->find(id);
    
        if(vst_it!=vst_pointer->end())
        {
            return;
        }
        else
        {
            shm_virtual_stigmergy_tuple_type *tmp=segment.construct<shm_virtual_stigmergy_tuple_type>(p)(std::less<shm_string>(), alloc_inst);
            VirtualStigmergyType vst(id, *tmp);
            
            named_kernel_mtx.lock();
            vst_pointer->insert(vst);
            named_kernel_mtx.unlock();
        }
    }
    
    
    void KernelHandle::insertOrUpdateVirtualStigmergy(unsigned int id, std::string key_std, std::string value_std, time_t time_now, unsigned int robot_id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        micros_swarm_framework::shm_string  key(key_std.data(), alloc_inst);
        micros_swarm_framework::shm_string  value(value_std.data(), alloc_inst);
    
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            return;
        }
  
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
        shm_virtual_stigmergy_type::iterator vst_it;
        vst_it=vst_pointer->find(id);
    
        if(vst_it!=vst_pointer->end())
        {
            shm_virtual_stigmergy_tuple_type::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                ShmVirtualStigmergyTuple new_tuple(value, time_now, robot_id, alloc_inst);
                
                named_kernel_mtx.lock();
                svstt_it->second = new_tuple;
                named_kernel_mtx.unlock();
            }
            else
            {
                ShmVirtualStigmergyTuple new_tuple(value, time_now, robot_id, alloc_inst);
                ShmVirtualStigmergyTupleType new_tuple_type(key,new_tuple);
                
                named_kernel_mtx.lock();
                vst_it->second.insert(new_tuple_type);
                named_kernel_mtx.unlock();
            }
        }
        else
        {       
            std::cout<<"ID "<<id<<" VirtualStigmergy is not exist."<<std::endl;
            return;
        }
    }
    
    VstigTuple KernelHandle::getVirtualStigmergyTuple(unsigned int id, std::string key_std)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        micros_swarm_framework::shm_string  key(key_std.data(), alloc_inst);
    
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            
        }
  
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
        shm_virtual_stigmergy_type::iterator vst_it;
        vst_it=vst_pointer->find(id);
    
        if(vst_it!=vst_pointer->end())
        {
            shm_virtual_stigmergy_tuple_type::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                shm_string value_shm=svstt_it->second.getVirtualStigmergyValue();
                std::string value(value_shm.data(), value_shm.size());
                time_t time_now=svstt_it->second.getVirtualStigmergyTimestamp();
                unsigned int robot_id=svstt_it->second.getRobotID();
                VstigTuple new_tuple(value, time_now, robot_id);
                
                return new_tuple;
            }
        }
        
        std::string value="";
        time_t time_now=0;
        unsigned int robot_id=0;
        VstigTuple tuple(value, time_now, robot_id);
            
        return tuple;
    }
    
    unsigned int KernelHandle::getVirtualStigmergySize(unsigned int id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        //std::string mutex_name="named_kernel_mtx"+robot_id_string;
        //boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
    
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            return 0;
        }
  
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
        shm_virtual_stigmergy_type::iterator vst_it;
        vst_it=vst_pointer->find(id);
    
        if(vst_it!=vst_pointer->end())
        {
            unsigned int size=vst_it->second.size();
            return vst_it->second.size();
        }
        
        return 0;
    }
    
    void KernelHandle::deleteVirtualStigmergy(unsigned int id)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());  
    
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            return;
        }
  
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
    
        named_kernel_mtx.lock();
        vst_pointer->erase(id);
        named_kernel_mtx.unlock();
    }
    
    void KernelHandle::deleteVirtualStigmergyValue(unsigned int id, std::string key_std)
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        std::string mutex_name="named_kernel_mtx"+robot_id_string;
        boost::interprocess::named_mutex named_kernel_mtx(boost::interprocess::open_or_create, mutex_name.data()); 
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
    
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        micros_swarm_framework::shm_string  key(key_std.data(), alloc_inst);
    
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            return;
        }
  
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
        shm_virtual_stigmergy_type::iterator vst_it;
        vst_it=vst_pointer->find(id);
    
        if(vst_it!=vst_pointer->end())
        {
            shm_virtual_stigmergy_tuple_type::iterator svstt_it=vst_it->second.find(key);
        
            if(svstt_it!=vst_it->second.end())
            {
                named_kernel_mtx.lock();
                vst_it->second.erase(key);
                named_kernel_mtx.unlock();
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }
    }
    
    void KernelHandle::printVirtualStigmergy()
    {
        std::string robot_id_string=boost::lexical_cast<std::string>(KernelInitializer::unique_robot_id_);
        std::string shm_object_name="KernelData"+robot_id_string;
        boost::interprocess::managed_shared_memory segment(boost::interprocess::open_or_create ,shm_object_name.data(), 102400);
        VoidAllocator alloc_inst (segment.get_segment_manager());
        
        std::pair<shm_virtual_stigmergy_type*, std::size_t> virtual_stigmergy = segment.find<shm_virtual_stigmergy_type>("shm_virtual_stigmergy_"); 
        if(virtual_stigmergy.first==0)
        {
            return;
        }
        
        shm_virtual_stigmergy_type *vst_pointer=virtual_stigmergy.first;
        shm_virtual_stigmergy_type::iterator vst_it;
        shm_virtual_stigmergy_tuple_type *svstt_pointer;
        shm_virtual_stigmergy_tuple_type::iterator svstt_it;
        
        for (vst_it=vst_pointer->begin(); vst_it!=vst_pointer->end(); vst_it++)
        {
            std::cout<<"["<<vst_it->first<<":"<<std::endl;
            svstt_pointer=&(vst_it->second);
            for (svstt_it=svstt_pointer->begin(); svstt_it!=svstt_pointer->end(); svstt_it++)
            {
                std::cout<<"("<<svstt_it->first<<","<< \
                    svstt_it->second.getVirtualStigmergyValue()<<","<<svstt_it->second.getVirtualStigmergyTimestamp()<<","<<\
                    svstt_it->second.getRobotID()<<")"<<std::endl;
            }
            std::cout<<"]"<<std::endl;
            std::cout<<std::endl;
        }
    }
    
    void KernelHandle::publishPacket(micros_swarm_framework::MSFPPacket msfp_packet)  //广播一条packet
    {
        ros::NodeHandle n;
        static ros::Publisher packet_publisher = n.advertise<micros_swarm_framework::MSFPPacket>("/micros_swarm_framework_topic", 1000, true);
        
        static bool flag=false;
        
        if(!flag)
        {
            ros::Duration(1).sleep();
            if(!packet_publisher)
            {
                ROS_INFO("packet_publisher could not initialize");
            }
            
            flag=true;
        }
        
        if(ros::ok())
        {
            packet_publisher.publish(msfp_packet);
        }
    }
};
