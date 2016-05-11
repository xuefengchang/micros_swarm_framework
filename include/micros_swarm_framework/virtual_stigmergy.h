/* 
 *  virtual_stigmergy.h - micros_swarm_framework virtual stigmergy
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

#ifndef VIRTUAL_STIGMERGY_H_
#define VIRTUAL_STIGMERGY_H_

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
    
    template<class Type>
    class VirtualStigmergy{
        private:
            unsigned int vstig_id_;
        public:
            VirtualStigmergy(){vstig_id_=-1;}
            
            VirtualStigmergy(unsigned int vstig_id)
            {
                vstig_id_=vstig_id;
                micros_swarm_framework::KernelHandle kh;
                kh.createVirtualStigmergy(vstig_id_);
            }
            
            void virtualStigmergyPut(std::string key, Type data)
            {
                std::ostringstream archiveStream;
                boost::archive::text_oarchive archive(archiveStream);
                archive<<data;
                std::string s=archiveStream.str();
                
                micros_swarm_framework::KernelHandle kh;
                unsigned int id=vstig_id_;
                std::string key_str=key;
                std::string value_str=s;
                time_t time_now=time(0);
                unsigned int robot_id=kh.getRobotID();
                kh.insertOrUpdateVirtualStigmergy(id, key_str, value_str, time_now, robot_id);
                
                VirtualStigmergyPut vsp(id, key_str, value_str, time_now, robot_id);
                
                std::ostringstream archiveStream2;
                boost::archive::text_oarchive archive2(archiveStream2);
                archive2<<vsp;
                std::string vsp_str=archiveStream2.str();   
                      
                micros_swarm_framework::MSFPPacket p;
                p.packet_source=robot_id;
                p.packet_version=1;
                p.packet_type=VIRTUAL_STIGMERGY_PUT;
                p.packet_data=vsp_str;
                p.packet_ttl=1;
                p.package_check_sum=0;
                
                kh.publishPacket(p);
                
                ros::Duration(0.1).sleep();   
            }
            
            Type virtualStigmergyGet(std::string key)
            {
                micros_swarm_framework::KernelHandle kh;
                VstigTuple vst=kh.getVirtualStigmergyTuple(vstig_id_, key);
                
                if(vst.getVstigTimestamp()==0)
                {
                    std::cout<<"ID"<<vstig_id_<<" virtual stigmergy, "<<key<<"is not exist."<<std::endl;
                }
                
                std::string data_str=vst.getVstigValue();
                Type data;
                std::istringstream archiveStream(data_str);
                boost::archive::text_iarchive archive(archiveStream);
                archive>>data;
                
                unsigned int id=vstig_id_;
                std::string key_std=key;
                std::string value_std=vst.getVstigValue();
                time_t time_now=vst.getVstigTimestamp();
                unsigned int robot_id=vst.getRobotID();
                VirtualStigmergyQuery vsq(id, key_std, value_std, time_now, robot_id);
                
                std::ostringstream archiveStream2;
                boost::archive::text_oarchive archive2(archiveStream2);
                archive2<<vsq;
                std::string vsg_str=archiveStream2.str();  
                
                micros_swarm_framework::MSFPPacket p;
                p.packet_source=kh.getRobotID();
                p.packet_version=1;
                p.packet_type=VIRTUAL_STIGMERGY_QUERY;
                p.packet_data=vsg_str;
                p.packet_ttl=1;
                p.package_check_sum=0;
                kh.publishPacket(p);
                
                ros::Duration(0.1).sleep();
                
                return data;  
            }
            
            unsigned int virtualStigmergySize()
            {
                micros_swarm_framework::KernelHandle kh;
                return kh.getVirtualStigmergySize(vstig_id_);
            }
    };
    
}
#endif
