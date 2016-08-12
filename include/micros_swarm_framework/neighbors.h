/**
Software License Agreement (BSD)
\file      neighbors.h
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

#ifndef NEIGHBORS_H_
#define NEIGHBORS_H_

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

#include "micros_swarm_framework/singleton.h"
#include "micros_swarm_framework/runtime_platform.h"

namespace micros_swarm_framework{
    
    template<class Type>
    class Neighbors{
        private:
            boost::shared_ptr<RuntimePlatform> rtp_;
        public:
            std::map<int, Type> data_;
            
            Neighbors()
            {
                data_.clear();
                rtp_=Singleton<RuntimePlatform>::getSingleton();
            }
            
            void printData()
            {
                typename std::map<int, Type>::iterator it;
                
                for(it=data_.begin();it!=data_.end();it++)
                {
                    std::cout<<"---"<<it->first<<","<<it->second<<"---"<<std::endl;
                }
            }
            
            void neighborsForeach(void(*foreach)(Type))
            {
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    (*foreach)(n_it->second);
                }
            }
            
            void neighborsForeach(boost::function<void(Type)> foreach)  //for class member functions
            {
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    foreach(n_it->second);
                }
            }
            
            template<class T2>
            Neighbors<T2> neighborsMap( T2(*map)(Type) )
            {
                Neighbors<T2> n2;
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1=data_.begin(); n_it1!=data_.end();n_it1++)
                {
                    T2 temp=(*map)(n_it1->second);
            
                    n2.data_.insert(std::pair<int, T2>(n_it1->first,temp));
                }        
                return n2;
            }
            
            template<class T2>
            Neighbors<T2> neighborsMap(boost::function<T2(Type)> map)  //for class member functions
            {
                Neighbors<T2> n2;
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1=data_.begin(); n_it1!=data_.end();n_it1++)
                {
                    T2 temp=map(n_it1->second);
            
                    n2.data_.insert(std::pair<int, T2>(n_it1->first,temp));
                }        
                return n2;
            }
            
            template<class T2>
            T2 neighborsReduce(T2(*reduce)(Type, T2 &), T2 t2)
            {
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1=data_.begin(); n_it1!=data_.end();n_it1++)
                {
                    t2=(*reduce)(n_it1->second, t2);
                }
            
                return t2;
            }
            
            template<class T2>
            T2 neighborsReduce(boost::function<T2(Type, T2&)> reduce, T2 t2)  //for class member functions
            {
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1=data_.begin(); n_it1!=data_.end();n_it1++)
                {
                    t2=reduce(n_it1->second, t2);
                }
            
                return t2;
            }
            
            Neighbors<Type> neighborsFilter(bool(*filter)(int, Type))
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if((*filter)(n_it->first, n_it->second))
                    {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }
                return result;
            }
            
            Neighbors<Type> neighborsFilter(boost::function<bool(int, Type)> filter)  //for class member functions
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if(filter(n_it->first, n_it->second))
                    {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }
                return result;
            }
            
            Neighbors<Type> neighborsKin(int swarm_id)
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if(rtp_->inNeighborSwarm(n_it->first, swarm_id))
                    {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }

                return result;
            }
            
            Neighbors<Type> neighborsNonKin(int swarm_id)
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if(!rtp_->inNeighborSwarm(n_it->first, swarm_id))
                    {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
    };
    
    /*
     *  specialization for NeighborBase type
     */
    template<>
    class Neighbors<NeighborBase>{
        private:
            boost::shared_ptr<RuntimePlatform> rtp_;
        public:
            std::map<int, NeighborBase> data_;
            
            Neighbors()
            {
                data_.clear();
                rtp_=Singleton<RuntimePlatform>::getSingleton();
            }
            
            Neighbors(bool get_data_now)
            {
                if(get_data_now)
                {
                    data_.clear();
                    rtp_=Singleton<RuntimePlatform>::getSingleton();
                    data_=rtp_->getNeighbors();
                }
                else
                {
                    data_.clear();
                    rtp_=Singleton<RuntimePlatform>::getSingleton();
                }
            }
            
            void printData()
            {
                typename std::map<int, NeighborBase>::iterator it;
                
                for(it=data_.begin();it!=data_.end();it++)
                {
                    std::cout<<"---"<<it->first<<":  "<<it->second.getDistance()<<","<< \
                        it->second.getAzimuth()<<","<<it->second.getElevation()<<","<< \
                        it->second.getX()<<","<<it->second.getY()<<","<<it->second.getZ()<< \
                        it->second.getVX()<<","<<it->second.getVY()<<","<<it->second.getVZ()<< \
                        "---"<<std::endl;
                }
            }
            
            void neighborsForeach(void(*foreach)(NeighborBase))
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    (*foreach)(n_it->second);
                }
            }
            
            void neighborsForeach(boost::function<void(NeighborBase)> foreach)  //for class member functions
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    foreach(n_it->second);
                }
            }
            
            template<class T>
            Neighbors<T> neighborsMap(T(*map)(NeighborBase) )
            {
                Neighbors<T> n;
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    T temp=(*map)(n_it->second);
            
                    n.data_.insert(std::pair<int, T>(n_it->first,temp));
                }
                
                return n;
            }
            
            template<class T>
            Neighbors<T> neighborsMap(boost::function<T(NeighborBase)> map)  //for class member functions
            {
                Neighbors<T> n;
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    T temp=map(n_it->second);
            
                    n.data_.insert(std::pair<int, T>(n_it->first,temp));
                }
                
                return n;
            }
            
            template<class T>
            T neighborsReduce(T(*reduce)(NeighborBase, T &), T t)
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    t=(*reduce)(n_it->second, t);
                }
            
                return t;
            }
            
            template<class T>
            T neighborsReduce(boost::function<T(NeighborBase, T &)> reduce, T t)  //for class member functions
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    t=reduce(n_it->second, t);
                }
            
                return t;
            }
            
            Neighbors<NeighborBase> neighborsFilter(bool(*filter)(int, NeighborBase))
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if((*filter)(n_it->first, n_it->second))
                    {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborBase> neighborsFilter(boost::function<bool(int, NeighborBase)> filter)  //for class member functions
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if(filter(n_it->first, n_it->second))
                    {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborBase> neighborsKin(int swarm_id)
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if(rtp_->inNeighborSwarm(n_it->first, swarm_id))
                    {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborBase> neighborsNonKin(int swarm_id)
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it=data_.begin(); n_it!=data_.end();n_it++)
                {
                    if(!rtp_->inNeighborSwarm(n_it->first, swarm_id))
                    {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
    };
};
#endif
