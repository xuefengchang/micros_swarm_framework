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

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "micros_swarm_framework/micros_swarm_framework.h"

namespace micros_swarm_framework{
    
    class TestNC : public nodelet::Nodelet
    {
        public:
            ros::NodeHandle node_handle_;
            boost::shared_ptr<RuntimePlatform> rtp_;
            boost::shared_ptr<CommunicationInterface> communicator_;
            ros::Timer timer_;
            ros::Publisher pub_;
            ros::Subscriber sub_;
            
            TestNC();
            ~TestNC();
            void callback(double value);
            virtual void onInit(); 
    };

    TestNC::TestNC()
    {
        
    }
    
    TestNC::~TestNC()
    {
    }
    
    void TestNC::callback(double value)
    {
        std::cout<<"I received the value: "<<value<<std::endl;
    }
    
    void TestNC::onInit()
    {
        node_handle_ = getNodeHandle();
        rtp_=Singleton<RuntimePlatform>::getSingleton();
        #ifdef ROS
        communicator_=Singleton<ROSCommunication>::getSingleton();
        #endif
        #ifdef OPENSPLICE_DDS
        communicator_=Singleton<OpenSpliceDDSCommunication>::getSingleton();
        #endif
    
        NeighborCommunication nc=NeighborCommunication("double");
        
        boost::function<void(double)> cb=boost::bind(&TestNC::callback, this, _1);
        nc.neighborListen("testkey", cb);
        
        //nc.neighborIgnore("testkey");
        
        for(int i=0;i<10;i++)
        {
            nc.neighborBroadcast("testkey", 3.14);
            ros::Duration(1).sleep();
        }
    }
};

// Register the nodelet
PLUGINLIB_EXPORT_CLASS(micros_swarm_framework::TestNC, nodelet::Nodelet)
