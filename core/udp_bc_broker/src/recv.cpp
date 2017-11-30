/**
Software License Agreement (BSD)
\file      recv.cpp
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

#include "udp_bc_broker/recv.h"

using namespace std;

namespace udp_bc_broker{

    UdpRecver::UdpRecver(int port)
    {
        setvbuf(stdout, NULL, _IONBF, 0);
        fflush(stdout);

        //bind address
        bzero(&addrto, sizeof(struct sockaddr_in));
        addrto.sin_family = AF_INET;
        addrto.sin_addr.s_addr = htonl(INADDR_ANY);
        addrto.sin_port = htons(port);

        //broadcast address
        bzero(&from, sizeof(struct sockaddr_in));
        from.sin_family = AF_INET;
        from.sin_addr.s_addr = htonl(INADDR_ANY);
        from.sin_port = htons(port);

        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
            cout<<"socket error!"<<endl;
            exit(-1);
        }

        const int opt = 1;
        int nb = 0;
        nb = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
        if(nb == -1) {
            cout<<"set socket error!"<<endl;
            exit(-1);
        }

        if(bind(sock,(struct sockaddr *)&(addrto), sizeof(struct sockaddr_in)) == -1) {
            cout<<"bind error!"<<endl;
            exit(-1);
        }

        len = sizeof(sockaddr_in);
        buf = new char[10000];
    }

    UdpRecver::~UdpRecver()
    {
        process_thread_->interrupt();
        process_thread_->join();
        delete process_thread_;

        delete[] buf;
    }

    void UdpRecver::process_msg()
    {
        while(true) {
            int ret = recvfrom(sock, buf, 10000, 0, (struct sockaddr*)&from, (socklen_t*)&len);
            if(ret <= 0) {
                cout<<"read error...."<<sock<<endl;
            }
            else {
                std::vector<uint8_t> msg_data;
                for(int i = 0; i < ret; i++) {
                    msg_data.push_back(buf[i]);
                }
                callback(msg_data);
            }
            sleep(0.1);
        }
    }

    void UdpRecver::receive(boost::function<void(const vector<uint8_t>&)> callBack)
    {
        callback = callBack;
        process_thread_ = new boost::thread(&UdpRecver::process_msg, this);
    }
};