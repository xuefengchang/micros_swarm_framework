/* 
 *  neighbors.h - micros_swarm_framework neighbors
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

#ifndef NEIGHBOR_H_
#define NEIGHBOR_H_

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

#include <ros/ros.h>

#include <micros_swarm_framework/kernel.h>

namespace micros_swarm_framework{
    
    template<class Type>
    class Neighbors{
        public:
            std::map<unsigned int, Type> data_;
            
            Neighbors()
            {
                data_.clear();
            }
            
            void printData()
            {
                typename std::map<unsigned int, Type>::iterator it;
                
                for(it=data_.begin();it!=data_.end();it++)
                {
                    std::cout<<"---"<<it->first<<","<<it->second<<"---"<<std::endl;
                }
            }
            
            void neighborsForeach(void(*foreach)(Type))
            {
                typename std::map<unsigned int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    (*foreach)(n_it->second);
                }
            }
            
            template<class T2>
            Neighbors<T2> neighborsMap( T2(*map)(Type) )
            {
                Neighbors<T2> n2;
                typename std::map<unsigned int, Type>::iterator n_it1;
    
                for(n_it1=data_.begin(); n_it1!=data_.end();n_it1++)
                {
                    T2 temp=(*map)(n_it1->second);
            
                    n2.data_.insert(std::pair<unsigned int, T2>(n_it1->first,temp));
                }        
                return n2;
            }
            
            
            template<class T2>
            T2 neighborsReduce( T2(*reduce)(Type, T2 &), T2 t2)
            {
                typename std::map<unsigned int, Type>::iterator n_it1;
    
                for(n_it1=data_.begin(); n_it1!=data_.end();n_it1++)
                {
                    t2=(*reduce)(n_it1->second, t2);
                }
            
                return t2;
            }
            
            Neighbors<Type> neighborsFilter(bool(*filter)(unsigned int, Type))
            {
                Neighbors<Type> result;
            
                typename std::map<unsigned int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if((*filter)(n_it->first, n_it->second))
                    {
                        result.data_.insert(std::pair<unsigned int, Type>(n_it->first,n_it->second));
                    }
                }
                return result;
            }
            
            Neighbors<Type> neighborsKin(unsigned int swarm_id)
            {
                Neighbors<Type> result;
            
                typename std::map<unsigned int, Type>::iterator n_it;
                
                micros_swarm_framework::KernelHandle kh;
                std::set<unsigned int> sm=kh.getSwarmMembers(swarm_id);
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    std::set<unsigned int>::iterator sm_it;
                    sm_it=sm.find(n_it->first);
                    if(sm_it!=sm.end())
                    {
                        result.data_.insert(std::pair<unsigned int, NeighborLocation>(n_it->first,n_it->second));
                    }
                }
                return result;
            }
            
            Neighbors<Type> neighborsNonKin(unsigned int swarm_id)
            {
                Neighbors<Type> result;
            
                typename std::map<unsigned int, Type>::iterator n_it;
                
                micros_swarm_framework::KernelHandle kh;
                std::set<unsigned int> sm=kh.getSwarmMembers(swarm_id);
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    std::set<unsigned int>::iterator sm_it;
                    sm_it=sm.find(n_it->first);
                    if(sm_it==sm.end())
                    {
                        result.data_.insert(std::pair<unsigned int, NeighborLocation>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
    };
    
    
    /*
     *  specialization for NeighborLocation type
     */
    template<>
    class Neighbors<NeighborLocation>{
        public:
            std::map<unsigned int, NeighborLocation> data_;
            
            Neighbors()
            {
                data_.clear();
            }
            
            Neighbors(bool get_data_now)
            {
                if(get_data_now)
                {
                    micros_swarm_framework::KernelHandle kh;
                    data_=kh.getNeighbors();
                }
                else
                {
                    data_.clear();
                }
            }
            
            void printData()
            {
                typename std::map<unsigned int, NeighborLocation>::iterator it;
                
                for(it=data_.begin();it!=data_.end();it++)
                {
                    std::cout<<"---"<<it->first<<":  "<<it->second.getDistance()<<","<< \
                        it->second.getAzimuth()<<","<<it->second.getElevation()<<","<< \
                        it->second.getX()<<","<<it->second.getY()<<","<<it->second.getZ()<< \
                        "---"<<std::endl;
                }
            }
            
            void neighborsForeach(void(*foreach)(NeighborLocation))
            {
                typename std::map<unsigned int, NeighborLocation>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    (*foreach)(n_it->second);
                }
            }
            
            template<class T>
            Neighbors<T> neighborsMap( T(*map)(NeighborLocation) )
            {
                Neighbors<T> n;
                typename std::map<unsigned int, NeighborLocation>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    T temp=(*map)(n_it->second);
            
                    n.data_.insert(std::pair<unsigned int, T>(n_it->first,temp));
                }
                
                return n;
            }
            
            
            template<class T>
            T neighborsReduce( T(*reduce)(NeighborLocation, T &), T t)
            {
                typename std::map<unsigned int, NeighborLocation>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    t=(*reduce)(n_it->second, t);
                }
            
                return t;
            }
            
            Neighbors<NeighborLocation> neighborsFilter(bool(*filter)(unsigned int, NeighborLocation))
            {
                Neighbors<NeighborLocation> result;
            
                typename std::map<unsigned int, NeighborLocation>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if((*filter)(n_it->first, n_it->second))
                    {
                        result.data_.insert(std::pair<unsigned int, NeighborLocation>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborLocation> neighborsKin(unsigned int swarm_id)
            {
                Neighbors<NeighborLocation> result;
            
                typename std::map<unsigned int, NeighborLocation>::iterator n_it;
                
                micros_swarm_framework::KernelHandle kh;
                std::set<unsigned int> sm=kh.getSwarmMembers(swarm_id);
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    std::set<unsigned int>::iterator sm_it;
                    sm_it=sm.find(n_it->first);
                    if(sm_it!=sm.end())
                    {
                        result.data_.insert(std::pair<unsigned int, NeighborLocation>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborLocation> neighborsNonKin(unsigned int swarm_id)
            {
                Neighbors<NeighborLocation> result;
            
                typename std::map<unsigned int, NeighborLocation>::iterator n_it;
                
                micros_swarm_framework::KernelHandle kh;
                std::set<unsigned int> sm=kh.getSwarmMembers(swarm_id);
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    std::set<unsigned int>::iterator sm_it;
                    sm_it=sm.find(n_it->first);
                    if(sm_it==sm.end())
                    {
                        result.data_.insert(std::pair<unsigned int, NeighborLocation>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
    };
};
#endif
