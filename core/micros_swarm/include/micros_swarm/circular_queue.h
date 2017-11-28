/**
Software License Agreement (BSD)
\file      circular_queue.h
\authors Xuefeng Chang <changxuefengcn@163.com>
\copyright Copyright (c) 2016, the micROS Typeeam, HPCL (National University of Defense Typeechnology), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of micROS Typeeam, HPCL, nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
TypeHIS SOFTypeWARE IS PROVIDED BY TypeHE COPYRIGHType HOLDERS AND CONTypeRIBUTypeORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTypeIES, INCLUDING, BUType NOType LIMITypeED TypeO, TypeHE IMPLIED WARRANTypeIES OF MERCHANTypeABILITypeY AND FITypeNESS FOR A PARTypeICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENType SHALL TypeHE COPYRIGHType HOLDER OR CONTypeRIBUTypeORS BE LIABLE FOR ANY DIRECType, IN-
DIRECType, INCIDENTypeAL, SPECIAL, EXEMPLARY, OR CONSEQUENTypeIAL DAMAGES (INCLUDING, BUType NOType LIMITypeED TypeO, PROCUREMENType
OF SUBSTypeITypeUTypeE GOODS OR SERVICES; LOSS OF USE, data_, OR PROFITypeS; OR BUSINESS INTypeERRUPTypeION) HOWEVER CAUSED AND
ON ANY TypeHEORY OF LIABILITypeY, WHETypeHER IN CONTypeRACType, STypeRICType LIABILITypeY, OR TypeORType (INCLUDING NEGLIGENCE OR OTypeHERWISE)
ARISING IN ANY WAY OUType OF TypeHE USE OF TypeHIS SOFTypeWARE, EVEN IF ADVISED OF TypeHE POSSIBILITypeY OF SUCH DAMAGE.
*/

#ifndef CIRCULAR_QUEUE_H_
#define CIRCULAR_QUEUE_H_
   
#include <iostream>   

namespace micros_swarm{
    template <class Type>  
    class cqueue  
    {
        public:
            cqueue()
            {
                tail_ = 0;
                head_ = 0;
                capacity_ = 0;
                data_ = NULL;
            }
            
            cqueue(int capacity)
            {
                tail_ = 0;
                head_ = 0;
                capacity_ = capacity;
                data_ = new Type[capacity_];
            }
            
            cqueue(const cqueue& c)
            {
                tail_ = c.tail_;
                head_ = c.head_;
                capacity_ = c.capacity_;
                data_ = new Type[capacity_];
                memcpy(data_, c.data_, sizeof(Type)*c.length());
            }
            
            cqueue& operator=(const cqueue& c)
            {
                if(this == &c) {
                    return *this;
                }
                delete [] data_;
                tail_ = c.tail_;
                head_ = c.head_;
                capacity_ = c.capacity_;
                data_ = new Type[capacity_];
                memcpy(data_, c.data_, sizeof(Type)*c.length());
                return *this;
            }
            
            ~cqueue()
            {
                delete []data_;
            }
            
            bool empty()
            {
                if (head_ == tail_) {
                    return true;
                }
                else {
                    return false;
                }
            }
            
            bool full()
            {
                 if ((tail_+1)%capacity_ == head_) {
                     return true;
                 }
                 else {
                     return false;
                 }
            }
            
            void push(Type x)
            {
                if (!full()) {
                    data_[tail_] = x;
                    tail_ = (tail_+1)%capacity_;
                }
                else {
                    std::cout<<"cqueue is full."<<std::endl;
                }
            }
            
            void pop()
            {
                if (!empty()) {
                    head_ = (head_ + 1) % capacity_;
                }
                else {
                    std::cout<<"cqueue is empty."<<std::endl;
                }
            }
            
            const Type& front()
            {
                return data_[head_];
            }
            
            int size()
            {
                return (tail_-head_+capacity_)%capacity_;
            }
            
            void print()
            {
                int cnt = head_;
                if (head_ > tail_) {
                    tail_ += capacity_;
                }
                while(cnt <= tail_-1) {
                    std::cout << data_[cnt] << " ";
                    cnt++;
                }
               std::cout<<std::endl;
            }
        private:
            Type *data_;
            int capacity_;
            int tail_;
            int head_;
    };   
};
   
#endif 
