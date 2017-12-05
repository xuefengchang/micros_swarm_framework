/**
Software License Agreement (BSD)
\file      daemon_node.cpp
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

#include <signal.h>
#include <time.h>
#include "micros_swarm/runtime_core.h"

boost::shared_ptr<micros_swarm::RuntimeCore> rtp_core_;

void rtpManagerDestroySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    // All the default sigint handler does is call shutdown()
    rtp_core_->shutdown();
    rtp_core_.reset();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micros_swarm_framework_rtp_node");

    rtp_core_.reset(new micros_swarm::RuntimeCore());
    rtp_core_->initialize();

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, rtpManagerDestroySigintHandler);

    boost::thread t = boost::thread(boost::bind(&ros::spin));
    t.join();
    return 0;
}

