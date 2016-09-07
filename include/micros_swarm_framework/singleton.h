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
#include <string>
#include <boost/smart_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace micros_swarm_framework{
    
    template<class T>
    class Singleton
    {
        public:
            static boost::shared_ptr<T> getSingleton()  //none parameter contruction
            {
                if(object.use_count() == 0)
                {
                    micros_swarm_framework_mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T());
                    micros_swarm_framework_mut.unlock();
                }  
                return object;
            }
            
            template<class P1>
            static boost::shared_ptr<T> getSingleton(P1 p1)  //one parameter construction
            {
                if(object.use_count() == 0)
                {
                    micros_swarm_framework_mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T(p1));
                    micros_swarm_framework_mut.unlock();
                }
                return object;
            }
            
            template<class P1, class P2>
            static boost::shared_ptr<T> getSingleton(P1 p1, P2 p2)  //two parameters construction
            {
                if(object.use_count() == 0)
                {
                    micros_swarm_framework_mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T(p1, p2));
                    micros_swarm_framework_mut.unlock();
                }
                return object;
            }
            
            template<class P1, class P2, class P3>
            static boost::shared_ptr<T> getSingleton(P1 p1, P2 p2, P3 p3)  //three parameter construction
            {
                if(object.use_count() == 0)
                {
                    micros_swarm_framework_mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T(p1, p2, p3));
                    micros_swarm_framework_mut.unlock();
                }
                return object;
            }
            
            static int use_count()
            {
                return object.use_count();
            }
            
            ~Singleton(){}
        private:
            Singleton(){}
        private:
            static boost::shared_ptr<T> object;
            static boost::mutex micros_swarm_framework_mut;  //TODO, is this ok for multi thread???
    };
    
    template<class T>
    boost::shared_ptr<T> Singleton<T>::object;
    
    template<class T>
    boost::mutex Singleton<T>::micros_swarm_framework_mut;
};
    
#endif

