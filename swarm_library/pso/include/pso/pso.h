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
#include <math.h>
#include <set>
#include <queue>
#include <algorithm>

#include <fstream>
#include <sstream>

namespace pso{

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
                Xmax=30.0f;
                Xmin=0.0f;  
                Ymax=30.0f;  
                Ymin=0.0f;  
                Vxmax=Xmax-Xmin;
                Vxmin=0-Vxmax;  
                Vymax=Ymax-Ymin;  
                Vymin=0-Vymax;  
  
                c1=2.0f;  
                c2=2.0f;
    
                c.x=x;  
                c.y=y;  
                //p=100.0f;
                p=pow(c.x-10.0f,2)+pow(c.y-20.0f,2);  
                Vx=(Xmax-Xmin)/8.0f;
                Vy=(Xmax-Xmin)/8.0f;
      
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

            Coordinate getPBest()const
            {
                return pBest;  
            }

            void setV(Coordinate gBest,float w)
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
        private:  
            Coordinate c;  
            float p;
            Coordinate pBest;
            float Vx;  
            float Vy;  
            float Xmax,Xmin;  
            float Ymax,Ymin;  
            float Vxmax,Vxmin;
            float Vymax,Vymin;  
            float c1,c2;
    };
};
#endif
