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
#include <vector>
#include <map>
#include <algorithm>

#include "micros_swarm/singleton.h"
#include "micros_swarm/runtime_handle.h"

namespace micros_swarm{
    
    template<class Type>
    class Neighbors{
        public:
            Neighbors()
            {
                data_.clear();
                rth_ = Singleton<RuntimeHandle>::getSingleton();
            }
            
            Neighbors(const Neighbors<Type>& n)
            {
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                data_ = n.data_;
            }
            
            Neighbors& operator=(const Neighbors<Type>& n)
            {
                if(this == &n) {
                    return *this;
                }
                data_.clear();
                data_ = n.data_;
                return *this;
            }
            
            ~Neighbors()
            {
                rth_.reset();
            }
            
            std::map<int, Type>& data()
            {
                return data_;
            }
            
            void print()
            {
                typename std::map<int, Type>::iterator it;
                
                for(it = data_.begin(); it != data_.end(); it++) {
                    std::cout<<"---"<<it->first<<","<<it->second<<"---"<<std::endl;
                }
            }
            
            void foreach(void(*f)(Type))
            {
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    (*f)(n_it->second);
                }
            }
            
            void foreach(const boost::function<void(Type)>& f)  //for class member functions
            {
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    f(n_it->second);
                }
            }
            
            template<class T2>
            Neighbors<T2> map( T2(*f)(Type))
            {
                Neighbors<T2> n2;
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1 = data_.begin(); n_it1 != data_.end(); n_it1++) {
                    T2 temp = (*f)(n_it1->second);
                    n2.data_.insert(std::pair<int, T2>(n_it1->first,temp));
                }        
                return n2;
            }
            
            template<class T2>
            Neighbors<T2> map(const boost::function<T2(Type)>& f)  //for class member functions
            {
                Neighbors<T2> n2;
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1 = data_.begin(); n_it1 != data_.end(); n_it1++) {
                    T2 temp = f(n_it1->second);
                    n2.data_.insert(std::pair<int, T2>(n_it1->first,temp));
                }        
                return n2;
            }
            
            template<class T2>
            T2 reduce(T2(*f)(Type, T2 &), T2& t2)
            {
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1 = data_.begin(); n_it1 != data_.end(); n_it1++) {
                    t2 = (*f)(n_it1->second, t2);
                }
            
                return t2;
            }
            
            template<class T2>
            T2 reduce(const boost::function<T2(Type, T2&)>& f, T2& t2)  //for class member functions
            {
                typename std::map<int, Type>::iterator n_it1;
    
                for(n_it1 = data_.begin(); n_it1 != data_.end(); n_it1++) {
                    t2 = f(n_it1->second, t2);
                }
            
                return t2;
            }
            
            Neighbors<Type> filter(bool(*f)(int, Type))
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
                
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if((*f)(n_it->first, n_it->second)) {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }
                return result;
            }
            
            Neighbors<Type> filter(const boost::function<bool(int, Type)>& f)  //for class member functions
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if(f(n_it->first, n_it->second)) {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }
                return result;
            }
            
            Neighbors<Type> kin(int swarm_id)
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if(rth_->inNeighborSwarm(n_it->first, swarm_id)) {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }

                return result;
            }
            
            Neighbors<Type> nonkin(int swarm_id)
            {
                Neighbors<Type> result;
            
                typename std::map<int, Type>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if(!rth_->inNeighborSwarm(n_it->first, swarm_id)) {
                        result.data_.insert(std::pair<int, Type>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
        private:
            boost::shared_ptr<RuntimeHandle> rth_;
            std::map<int, Type> data_;
    };
    
    /*
     *  specialization for NeighborBase type
     */
    template<>
    class Neighbors<NeighborBase>{
        public:
            Neighbors()
            {
                data_.clear();
                rth_ = Singleton<RuntimeHandle>::getSingleton();
            }
            
            Neighbors(bool get_data_now)
            {
                if(get_data_now) {
                    data_.clear();
                    rth_ = Singleton<RuntimeHandle>::getSingleton();
                    rth_->getNeighbors(data_);
                }
                else {
                    data_.clear();
                    rth_ = Singleton<RuntimeHandle>::getSingleton();
                }
            }
            
            Neighbors(const Neighbors<NeighborBase>& n)
            {
                rth_ = Singleton<RuntimeHandle>::getSingleton();
                data_ = n.data_;
            }
            
            Neighbors& operator=(const Neighbors<NeighborBase>& n)
            {
                if(this == &n) {
                    return *this;
                }
                data_.clear();
                data_ = n.data_;
                return *this;
            }

            ~Neighbors()
            {
                rth_.reset();
            }
            
            std::map<int, NeighborBase>& data()
            {
                return data_;
            }
            
            void print()
            {
                typename std::map<int, NeighborBase>::iterator it;
                
                for(it = data_.begin(); it != data_.end(); it++) {
                    std::cout<<"---"<<it->first<<":  "<<it->second.distance<<","<< \
                        it->second.azimuth<<","<<it->second.elevation<<","<< \
                        it->second.x<<","<<it->second.y<<","<<it->second.z<< \
                        it->second.vx<<","<<it->second.vy<<","<<it->second.vz<< \
                        "---"<<std::endl;
                }
            }
            
            void foreach(void(*f)(NeighborBase))
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    (*f)(n_it->second);
                }
            }
            
            void foreach(const boost::function<void(NeighborBase)>& f)  //for class member functions
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    f(n_it->second);
                }
            }
            
            template<class T>
            Neighbors<T> map(T(*f)(NeighborBase))
            {
                Neighbors<T> n;
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    T temp = (*f)(n_it->second);
                    n.data_.insert(std::pair<int, T>(n_it->first,temp));
                }
                
                return n;
            }
            
            template<class T>
            Neighbors<T> map(const boost::function<T(NeighborBase)>& f)  //for class member functions
            {
                Neighbors<T> n;
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    T temp = f(n_it->second);
                    n.data_.insert(std::pair<int, T>(n_it->first,temp));
                }
                
                return n;
            }
            
            template<class T>
            T reduce(T(*f)(NeighborBase, T &), T& t)
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    t = (*f)(n_it->second, t);
                }
            
                return t;
            }
            
            template<class T>
            T reduce(const boost::function<T(NeighborBase, T &)>& f, T& t)  //for class member functions
            {
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    t = f(n_it->second, t);
                }
            
                return t;
            }
            
            Neighbors<NeighborBase> filter(bool(*f)(int, NeighborBase))
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if((*f)(n_it->first, n_it->second)) {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborBase> filter(const boost::function<bool(int, NeighborBase)>& f)  //for class member functions
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if(f(n_it->first, n_it->second)) {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborBase> kin(int swarm_id)
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if(rth_->inNeighborSwarm(n_it->first, swarm_id)) {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
            
            Neighbors<NeighborBase> nonkin(int swarm_id)
            {
                Neighbors<NeighborBase> result;
            
                typename std::map<int, NeighborBase>::iterator n_it;
    
                for(n_it = data_.begin(); n_it != data_.end(); n_it++) {
                    if(!rth_->inNeighborSwarm(n_it->first, swarm_id)) {
                        result.data_.insert(std::pair<int, NeighborBase>(n_it->first,n_it->second));
                    }
                }
                
                return result;
            }
        private:
            boost::shared_ptr<RuntimeHandle> rth_;
            std::map<int, NeighborBase> data_;
    };
};
#endif
