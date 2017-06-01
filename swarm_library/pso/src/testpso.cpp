/**
Software License Agreement (BSD)
\file      testpso.cpp 
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
#include <stdio.h>
#include <time.h>  
#include <string>
#include <fstream>  
#include <iostream>
#include "pso/pso.h"

using namespace std;
using namespace pso;

int main()
{
    Particle*p[40];  
    float w;
  
    Particle*temp;  
    float randX,randY;  
    srand((int)time(NULL));  
    for(int i=0;i<40;++i)  
    {
        randX=(rand()/(float)RAND_MAX)*30.0f;  
        randY=(rand()/(float)RAND_MAX)*30.0f;  
        cout<<"randX="<<randX<<endl;  
        cout<<"randY="<<randY<<endl;  
  
        p[i]=new Particle(randX,randY);  
        cout<<"The temp info is X:"<<p[i]->getX()<<" Y:"<<p[i]->getY()<<endl;  
      
    }  

    Coordinate gBest;
    int bestIndex=0;  
    float bestP;
    bestP=p[0]->getP();  
    gBest=p[0]->getPBest();  
    for(int i=1;i<40;++i)  
    {  
        if(p[i]->getP()<bestP)  
        {  
            bestP=p[i]->getP();  
            gBest=p[i]->getPBest();  
            bestIndex=i;  
        }  
    }  

    cout<<"Now the initial gBest is X:"<<gBest.x<<" Y:"<<gBest.y<<endl;  
    cout<<"And the p[0] is X:"<<p[0]->getX()<<" Y:"<<p[0]->getY()<<endl;  
    cout<<"And the p[39] is X:"<<p[39]->getX()<<" Y:"<<p[39]->getY()<<endl;  
    cout<<"Now p[0].p="<<p[0]->getP()<<endl;

    for(int k=0;k<100;++k)
    {
        w=0.9f-(0.9f-0.4f)*k/99.0f;

        for(int i=0;i<40;++i)  
        {  
            temp=p[i];  
            temp->setV(gBest,w);  
            temp->setCoordinate();  
            temp->setP();
        }  
        bestP=p[0]->getP();  
        gBest=p[0]->getPBest();  
        for(int i=1;i<40;++i)  
        {  
            temp=p[i];  
            if(temp->getP()<bestP)  
            {  
                bestP=temp->getP();  
                gBest=temp->getPBest();  
                bestIndex=i;  
            }
        }  
        cout<<"Now gBest is X:"<<gBest.x<<" Y:"<<gBest.y<<" and the minP="<<p[bestIndex]->getP()<<endl;  
        cout<<"bestIndex="<<bestIndex<<endl;  
    }
  
    getchar();
    return 0;
}
