/**
Software License Agreement (BSD)
\file      singleton.h
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

#ifndef SINGLETON_H_
#define SINGLETON_H_

#include <iostream>
#include <string>
#include <boost/smart_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

boost::mutex mut;

namespace micros_swarm_framework{

    class Book
    {
        public:
            Book(std::string  bookName = std::string(),const int price = 0):bookName(bookName),price(price)
            {
            }
            ~Book(){}

            void setBookName(std::string  bookName)
            {
                mut.lock();
                this->bookName = bookName;
                mut.unlock();
            }
            void setPrice(const int price)
            {
                mut.lock();
                this->price = price;
                mut.unlock();
            }
            void display()
            {
                mut.lock();
                std::cout<<"book:"<<bookName<<" "<<"price:"<<price<<std::endl;
                mut.unlock();
            }
        private:
            std::string bookName;
            int price;
    };
    
    template<class T>
    class Singleton
    {
        public:
            static boost::shared_ptr<T> getSingleton()  //none parameter contruction
            {
                if(object.use_count() == 0)
                {
                    mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T());
                    mut.unlock();
                }  
                return object;
            }
            
            template<class P1>
            static boost::shared_ptr<T> getSingleton(P1 p1)  //one parameter construction
            {
                
                if(object.use_count() == 0)
                {
                    mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T(p1));
                    mut.unlock();
                }
                
                return object;
            }
            
            template<class P1, class P2>
            static boost::shared_ptr<T> getSingleton(P1 p1, P2 p2)  //two parameters construction
            {
                if(object.use_count() == 0)
                {
                    mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T(p1, p2));
                    mut.unlock();
                }
                return object;
            }
            
            template<class P1, class P2, class P3>
            static boost::shared_ptr<T> getSingleton(P1 p1, P2 p2, P3 p3)  //three parameter construction
            {
                if(object.use_count() == 0)
                {
                    mut.lock();
                    if(object.use_count()==0)
                        object = boost::shared_ptr<T>(new T(p1, p2, p3));
                    mut.unlock();
                }
                return object;
            }
            
            static int use_count()
            {
                return object.use_count();
            }
        private:
            Singleton(){}
        private:
            static boost::shared_ptr<T> object;
    };
    
    template<class T>
    boost::shared_ptr<T> Singleton<T>::object;
    
    class task
    {
        public:
            task(std::string bookName = std::string(),const int price = 0):bookName(bookName),price(price){}
            ~task(){}
            task(const task& tsk)
            {
                bookName = tsk.bookName;
                price = tsk.price;
            }
            task& operator = (const task& tsk)
            {
                bookName = tsk.bookName;
                price = tsk.price;
            }
            void operator()()const
            {
                Singleton<Book>::getSingleton()->setBookName(bookName);
                Singleton<Book>::getSingleton()->setPrice(price);
                Singleton<Book>::getSingleton()->display();
                std::cout<<Singleton<Book>::getSingleton().use_count()<<std::endl;
            }
        private:
            std::string bookName;
            int price;
    };
};
    
#endif


/*
int main()
{
    boost::thread thr1(task("C++",20));
    boost::thread thr2(task("Java",40));
    boost::thread thr3(task("Python",60));

    thr1.join();
    thr2.join();
    thr3.join();
    return 0;
}
*/

