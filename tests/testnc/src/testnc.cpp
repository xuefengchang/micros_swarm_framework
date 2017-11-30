/**
Software License Agreement (BSD)
\file      testnc.cpp 
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

#include "testnc/testnc.h"

// Register the application
PLUGINLIB_EXPORT_CLASS(testnc::TestNC, micros_swarm::Application)

namespace testnc{

    TestNC::TestNC() {}
    
    TestNC::~TestNC() {}

    void TestNC::init() {}

    void TestNC::stop() {}
    
    void TestNC::callback(const std_msgs::Float32& value)
    {
        //ROS_INFO("I received the value: %f.", value);
    }
    
    void TestNC::start()
    {
        std::cout<<"TestNC step into start..."<<std::endl;
        micros_swarm::Broadcaster<std_msgs::Float32> bc("testkey");
        boost::function<void(const std_msgs::Float32&)> cb=boost::bind(&TestNC::callback, this, _1);
        micros_swarm::Listener<std_msgs::Float32> ls("testkey", cb);
        //ls.ignore();
        
        for(int i = 0; i < 10; i++) {
            std_msgs::Float32 val;
            val.data = 3.141;
            bc.broadcast(val);
            ros::Duration(1).sleep();
        }
    }
};

