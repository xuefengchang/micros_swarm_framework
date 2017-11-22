/**
Software License Agreement (BSD)
\file      singleton.h
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

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <iostream>
#include <boost/smart_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace micros_swarm{
    
    template<class T>
    class Singleton
    {
        public:
            static void makeSingleton(boost::shared_ptr<T>& existed_ptr)  //make an esisted ptr to be a singleton
            {
                if(existed_ptr.use_count() == 0) {
                    std::cout<<"WRONG: using an empty ptr to get singleton, exit."<<std::endl;
                    exit(-1);
                }
                else {
                    singleton_mutex_.lock();
                    singleton_object_ = existed_ptr;
                    singleton_mutex_.unlock();
                }
            }

            static boost::shared_ptr<T> getExistedSingleton()  //get the existed singleton. it is in pairs with makeSingleton()
            {
                if(singleton_object_.use_count() == 0) {
                    std::cout<<"WRONG: try to get an not existed ptr, exit."<<std::endl;
                    exit(-1);
                }
                return singleton_object_;
            }

            static void deleteSingleton()  //delete the singleton
            {
                if(singleton_object_.use_count() != 0) {
                    singleton_mutex_.lock();
                    singleton_object_.reset();
                    singleton_mutex_.unlock();
                }
            }

            static boost::shared_ptr<T> getSingleton()  //none parameter contruction
            {
                if(singleton_object_.use_count() == 0) {
                    singleton_mutex_.lock();
                    if(singleton_object_.use_count() == 0) {
                        singleton_object_ = boost::shared_ptr<T>(new T());
                    }
                    singleton_mutex_.unlock();
                }
                return singleton_object_;
            }
            
            template<class P1>
            static boost::shared_ptr<T> getSingleton(P1 p1)  //one parameter construction
            {
                if(singleton_object_.use_count() == 0) {
                    singleton_mutex_.lock();
                    if(singleton_object_.use_count() == 0) {
                        singleton_object_ = boost::shared_ptr<T>(new T(p1));
                    }
                    singleton_mutex_.unlock();
                }
                return singleton_object_;
            }
            
            template<class P1, class P2>
            static boost::shared_ptr<T> getSingleton(P1 p1, P2 p2)  //two parameters construction
            {
                if(singleton_object_.use_count() == 0) {
                    singleton_mutex_.lock();
                    if(singleton_object_.use_count() == 0) {
                        singleton_object_ = boost::shared_ptr<T>(new T(p1, p2));
                    }
                    singleton_mutex_.unlock();
                }
                return singleton_object_;
            }
            
            template<class P1, class P2, class P3>
            static boost::shared_ptr<T> getSingleton(P1 p1, P2 p2, P3 p3)  //three parameter construction
            {
                if(singleton_object_.use_count() == 0) {
                    singleton_mutex_.lock();
                    if(singleton_object_.use_count() == 0) {
                        singleton_object_ = boost::shared_ptr<T>(new T(p1, p2, p3));
                    }
                    singleton_mutex_.unlock();
                }
                return singleton_object_;
            }
            
            static int use_count()
            {
                return singleton_object_.use_count();
            }
            
            ~Singleton(){}
        private:
            Singleton(){}
        private:
            static boost::shared_ptr<T> singleton_object_;
            static boost::mutex singleton_mutex_;  //TODO, is this ok for multi thread???
    };
    
    template<class T>
    boost::shared_ptr<T> Singleton<T>::singleton_object_;
    
    template<class T>
    boost::mutex Singleton<T>::singleton_mutex_;
};
    
#endif

