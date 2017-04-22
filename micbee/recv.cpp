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
    class UDPMICBEERECV{
    public:
        void receive();

        UDPMICBEERECV(int port);
    private:
        int sock;
        struct sockaddr_in addrto;
        struct sockaddr_in from;
        int nlen;
        int len;
    };

    UDPMICBEERECV::UDPMICBEERECV(int port)
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

        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        {
            cout<<"socket error!"<<endl;
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

        if(bind(sock,(struct sockaddr *)&(addrto), sizeof(struct sockaddr_in)) == -1)
        {
            cout<<"bind error!"<<endl;
            exit(-1);
        }

        len = sizeof(sockaddr_in);
    };

    void UDPMICBEERECV::receive()
    {
        char smsg[100000] = {0};
        while(1)
        {
            int ret=recvfrom(sock, smsg, 100, 0, (struct sockaddr*)&from,(socklen_t*)&len);
            if(ret<=0)
            {
                cout<<"read error...."<<sock<<endl;
            }
            else
            {
                printf("%s\t\n", smsg);
            }

            sleep(1);
        }
    }
};

int main()
{
    micros_swarm_framework::UDPMICBEERECV recv(6000);
    recv.receive();
	return 0;
}

