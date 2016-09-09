/**
Software License Agreement (BSD)
\file      check_neighbor.h
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

#ifndef CHECK_NEIGHBOR_H_
#define CHECK_NEIGHBOR_H_

#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/variant.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp> 

#include "ros/ros.h"

namespace micros_swarm_framework{
    
    class CheckNeighborInterface{
        public:
            virtual bool isNeighbor(const Base& self, const Base& neighbor)=0;
    };
    
    class CheckNeighbor : public CheckNeighborInterface{
        public:
            CheckNeighbor(float neighbor_distance)
            {
                neighbor_distance_ = neighbor_distance;
            }
        
            float getNeighborDistance()
            {
                return neighbor_distance_;
            }
        
            bool isNeighbor(const Base& self, const Base& neighbor)
            {
                float distance=sqrt((self.x-neighbor.x)*(self.x-neighbor.x)+(self.y-neighbor.y)*(self.y-neighbor.y)+
                    (self.z-neighbor.z)*(self.z-neighbor.z));
                    
                if(distance<neighbor_distance_)
                    return true;
                    
                return false;
            }
        private:
            float neighbor_distance_;
    };
};

#endif
