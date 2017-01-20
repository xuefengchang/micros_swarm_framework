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
#include <stdio.h> //因为要用到sprintf函数。  
#include <time.h>  
#include <string>  
#include <math.h>  
#include <fstream>  
#include <iostream> 

#include "ros/ros.h"
#include "micros_swarm_framework/pso.h"

using namespace std;
using namespace micros_swarm_framework;

int main()
{
    Particle*p[40];  
    float w;//实际上是wmax=1.2f;此处设置wmin=0.5;  
  
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
    //至此，就完成了粒子群的初始化。  
    //////////////////////////////////////  
    Coordinate gBest;  //全局最优解。  
    int bestIndex=0;  
    float bestP; //最好的适应度。  
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
  
    ///////////////////////////////////   
    cout<<"Now the initial gBest is X:"<<gBest.x<<" Y:"<<gBest.y<<endl;  
    cout<<"And the p[0] is X:"<<p[0]->getX()<<" Y:"<<p[0]->getY()<<endl;  
    cout<<"And the p[39] is X:"<<p[39]->getX()<<" Y:"<<p[39]->getY()<<endl;  
    cout<<"Now p[0].p="<<p[0]->getP()<<endl;  
    ////////////////////////////////至此，已经寻找到初始时的种群最优。  
    /*char buf[20];  
    for(int i=0;i<40;++i)  
    {  
        sprintf(buf,"coordinate%d.dat",i);  
        ofstream out(buf,ios::out);  
        out.close();  
    }*/
    //////////////////////////这样做是为了每运行一次都重复添加。  
    for(int k=0;k<100;++k)   //k为迭代次数。  
    {
        w=0.9f-(0.9f-0.4f)*k/99.0f;  //这个因子很重要，既不能太大也不能太小。一开始自己就是设置得太大了导致出错。自己通过计算发现采用可变的惯性因子可以使得到的结果的精确度高一个数量级(达到1e-006)，而如果采用恒定的惯性因子，则只能得到1e-005的精度。  
        //////////////////////////一开始wmax=1.0,wmin=0.6,可以达到1e-006的精度，以为很高了，但是实际上wmax=0.9f,wmin=0.4f进可以很轻易地达到1e-11的水平，而这已经接近float的精度极限。  
  
        // w=0.85f;  
  
        for(int i=0;i<40;++i)  
        {  
            temp=p[i];  
            temp->setV(gBest,w);  
            temp->setCoordinate();  
            temp->setP();  
            //sprintf(buf,"coordinate%d.dat",i);  
            //temp->outputFile(buf);    
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
            /* 
            if((pow(gBest.x-10,2)+pow(gBest.y-20,2))<0.00001f) 
            { 
                cout<<"The gBest which is good enough has found!"<<endl; 
                cout<<"The index is "<<i<<endl; 
                cout<<"gBest is X:"<<gBest.x<<" Y:"<<gBest.y<<endl; 
                 exit(0); 
            } 
            */  
        }  
        cout<<"Now gBest is X:"<<gBest.x<<" Y:"<<gBest.y<<" and the minP="<<p[bestIndex]->getP()<<endl;  
        cout<<"bestIndex="<<bestIndex<<endl;  
    }
  
    getchar();  

    return 0;
}
