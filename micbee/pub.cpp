#include <iostream>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

using namespace std;

namespace micros_swarm_framework{
    class UDPMICBEEPUB{
        public:
            void publish(const char *msg);
            void receive();

            UDPMICBEEPUB(int port);
        private:
            int sock;
            struct sockaddr_in addrto;
            int nlen;
    };

    UDPMICBEEPUB::UDPMICBEEPUB(int port)
    {
        setvbuf(stdout, NULL, _IONBF, 0);
        fflush(stdout);

        sock = -1;
        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        {
            cout<<"socket is error!"<<endl;
            exit(-1);
        }

        const int opt = 1;
        int nb = 0;
        nb = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
        if(nb == -1)
        {
            cout<<"set socket error!"<<endl;
            exit(-1);
        }

        bzero(&addrto, sizeof(struct sockaddr_in));
        addrto.sin_family=AF_INET;
        addrto.sin_addr.s_addr=htonl(INADDR_BROADCAST);
        addrto.sin_port=htons(port);
        nlen=sizeof(addrto);
    };

    void UDPMICBEEPUB::publish(const char* msg)
    {
        int ret=sendto(sock, msg, strlen(msg), 0, (sockaddr*)&addrto, nlen);
        if(ret<0)
        {
            cout<<"send error, ret = "<<ret<<endl;
        }
        else
        {
            cout<<msg<<endl;
        }
    }

    void UDPMICBEEPUB::receive()
    {

    }
};



int main()
{
	micros_swarm_framework::UDPMICBEEPUB pub(6000);
    while(1)
    {
        char msg[]={"test"};
        pub.publish(msg);
        sleep(1);
    }
	return 0;
}

