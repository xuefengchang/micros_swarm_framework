/**
Software License Agreement (BSD)
\file      testbb.cpp
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

#include "testbb/testbb.h"

// Register the application
PLUGINLIB_EXPORT_CLASS(testbb::TestBb, micros_swarm::Application)

namespace testbb{

    TestBb::TestBb()
    {
    }

    TestBb::~TestBb()
    {
    }
    
    void TestBb::loop(const ros::TimerEvent&)
    {   
        static int count=0;
        std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(robot_id());
        bb.put(robot_id_string, robot_id()+count);
        count++;
        //std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(robot_id());
        //bb.get(robot_id_string);
    }
    
    void TestBb::start()
    {
        ros::NodeHandle nh;
        
        bb=micros_swarm::BlackBoard<int>(0,0);
        //std::string robot_id_string="robot_"+boost::lexical_cast<std::string>(robot_id());
        //bb.put(robot_id_string, robot_id());

        timer = nh.createTimer(ros::Duration(0.1), &TestBb::loop, this);
    }
};

