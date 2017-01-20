/**
Software License Agreement (BSD)
\file      pso.h
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

#ifndef PSO_H_
#define PSO_H_

#include <iostream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <vector>
#include <stack>
#include <map>
#include <set>
#include <queue>
#include <algorithm>

#include <fstream>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/serialization/vector.hpp>

#include "ros/ros.h"

namespace micros_swarm_framework{

    class Coordinate  {  
        public:  
            float x;  
            float y;  
            Coordinate()
            {
                x=0.0f;  
                y=.0f;  
            }
            
            Coordinate(float x,float y)
            {
                this->x=x;  
                this->y=y; 
            } 
    };
    
    class Particle{
        //friend: 
        //    std::ostream& operator<<(std::ostream &output,const Particle &right);  
  
        public:  
            Particle(float x,float y)
            {
                //////////////设置粒子属性参数
                Xmax=30.0f;   //自己通过测试证明，通过限制速度和位置坐标取得了良好的效果，可以很轻易地达到1e-006的精度以上，甚至于能达到1e-009的精度。原因就在于整个群体中的粒子都能更快地集中到适应度高的位置，自然群体最优解就更好。  
                Xmin=0.0f;  
                Ymax=30.0f;  
                Ymin=0.0f;  
                Vxmax=Xmax-Xmin;  //通常设置Vmax=Xmax-Xmin;    
                Vxmin=0-Vxmax;  
                Vymax=Ymax-Ymin;  
                Vymin=0-Vymax;  
  
                c1=2.0f;  
                c2=2.0f;
                //////////////////
    
                c.x=x;  
                c.y=y;  
                //p=100.0f; //先给它一个较大的适应度值（这里我们要得到的是一个较小的适应值）。  
                p=pow(c.x-10.0f,2)+pow(c.y-20.0f,2);  
                Vx=(Xmax-Xmin)/8.0f;  //////////////////////这里先采用第一种初始化方法，即给所有粒子一个相同的初始速度,为Vmax/8.0f,而Vmax=Xmax-Xmin=30-0=30;注意这个初始速度千万不能太大，自己测试发现它的大小对最终结果的精度影响也很大，有几个数量级。当然，也不能太小，自己测试发现在(Xmax-Xmin)/8.0f时可以得到比较高的精度。  
                Vy=(Xmax-Xmin)/8.0f;   
      
                ///初始时的pBest  
      
                pBest.x=x;  
                pBest.y=y;
            }
            
            void setP()
            {
                float temp=pow(c.x-10.0f,2)+pow(c.y-20.0f,2);  
                if(temp<p)  
                {  
                    p=temp;  
                    //pBest.x=c.x;  
                    //pBest.y=c.y;  
                    pBest=c;  
                }  
            }
            
            float getP()const
            {
                return p;  
            }
  
            //void setPBest();  //pBest的设置在setP()中就完成了。  
            Coordinate getPBest()const
            {
                return pBest;  
            }
      
            ///////////////////////这是第一种方法，即采用恒定的学习因子。但是实际上可变的学习因子c1,c2可以使种群更快地收敛。此处，将两个维度的速度设置放在同一个函数中。  
            void setV(Coordinate gBest,float w) //w为惯性因子
            {
                float r1,r2;  
                r1=rand()/(float)RAND_MAX;  
                r2=rand()/(float)RAND_MAX;  
                Vx=w*Vx+c1*r1*(pBest.x-c.x)+c2*r2*(gBest.x-c.x);  
                if(Vx>Vxmax)  
                    Vx=Vxmax;  
                else if(Vx<Vxmin)  
                    Vx=Vxmin;  
                Vy=w*Vy+c1*r1*(pBest.y-c.y)+c2*r2*(gBest.y-c.y);  
                if(Vy>Vxmax)  
                    Vy=Vxmax;  
                else if(Vy<Vxmin)  
                    Vy=Vxmin;  
            }
            
            float getVx()const
            {
                return Vx;
            }
            
            float getVy()const
            {
                return Vy;
            }
     
            void setCoordinate()
            {
                c.x=c.x+Vx;  
                if(c.x>Xmax)  
                    c.x=Xmax;  
                else if(c.x<Xmin)  
                    c.x=Xmin;  
                c.y=c.y+Vy;  
                if(c.y>Ymax)  
                    c.y=Ymax;  
                else if(c.y<Ymin)  
                    c.y=Ymin;
            }
            
            float getX()const
            {
                return c.x;  
            }
            
            float getY()const
            {
                return c.y;
            }
  
            //void outputFile(char Dir[])const;  
        private:  
            Coordinate c;  
            float p;  //p为适应度。  
            Coordinate pBest;  
            ////////////////////二维的话就要有两个速度。  
            float Vx;  
            float Vy;  
            float Xmax,Xmin;  
            float Ymax,Ymin;  
            float Vxmax,Vxmin; //它们是用来对坐标和速度进行限制的，限制它只能在一定的范围内。  
            float Vymax,Vymin;  
            float c1,c2; //c1,c2是学习因子。  
            //由于要对所有的对象进行比较之后才能得到群体最优，所以它还是不  
            //static Coordinate gBest; //这个是群体最优.整个群体共享一份就行，所以将它设置成static，但是注意千万不要以为static都是在初始化后就不能修改的，static const才是那样。     
    };
  
    /*void Particle::outputFile(char Dir[])const  
    {  
        ofstream out(Dir,ios::app);  //这是添加吧？  
        out<<this->getX()<<" "<<this->getY()<<" "<<pBest.x<<" "<<pBest.y<<endl;  
        out.close();  
    }  
  
    ostream& operator<<(ostream &output,const Particle &right)  
    {  
        output<<"Now the current coordinates is X:"<<right.getX()<<" Y:"<<right.getY()<<endl;  
        output<<"And the pBest is X:"<<right.getPBest().x<<" Y:"<<right.getPBest().y<<endl;  
        return output;  
    }*/
};
#endif
