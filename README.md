## 1. micros_swarm_framework introduction

micros_swarm_framewrok is implemented using C++ in ROS Indigo version. It contains some robot swarm data structure, for example neighbors, swarm, virtual stigmergy. These data structure mainly referenced the Buzz.

<center>
![image1](doc/images/app1.png)
</center>

<center>
![image2](doc/images/app2.png)
</center>

## 2. Version

* version 1.0 implemented neighbors, swarm, virtual stigmergy, neighbor communication.

## 3. Instructions for use

#### 3.1 Swarm
##### 1. create a swarm
```cpp
Swarm s = Swarm(1);  //create a swarm s with id 1
```

##### 2. join in a swarm unconditionally
```cpp
s.joinSwarm();  //join in the swarm s
```

##### 3. leave a swarm unconditionally
```cpp
s.leaveSwarm();  //leave the swarm s
```

##### 4. join in a swarm according to a certain condition
```cpp
bool checkID(unsigned int id)
{
    if(id%2==0)
        return true;
    return false;
}

/*
 *BIND_FUNCTION_AND_PARAMETER_VALUES is a Macro definition. It bind the function with the parameter values and generate a function object.
 */
BoolFunction bf=BIND_FUNCTION_AND_PARAMETER_VALUES(&checkID, self_id);

s.selectSwarm(bf);  //the robot whose id is even join in the swarm s
```

##### 5. leave a swarm according to a certain condition
```cpp
bool checkID(unsigned int id)
{
    if(id%2==0)
        return true;
    return false;
}

/*
 *BIND_FUNCTION_AND_PARAMETER_VALUES is a Macro definition. It bind the function with the parameter values and generate a function object.
 */
BoolFunction bf=BIND_FUNCTION_AND_PARAMETER_VALUES(&checkID, self_id);

s.unselectSwarm(bf);  //the robot whose id is even leave the swarm s
```

##### 6. execute a swarm task
```cpp
void printID(unsigned int id)
{
    std::cout<<"id="<<id<<std::endl;
}

Function f = BIND_FUNCTION_AND_PARAMETER_VALUES(&printID, self_id);

s.execute(f);  //the robot in the swarm s print the id of themselves
```

##### 7. swarm intersection operation
```cpp
Swarm a = Swarm(1);
Swarm b = Swarm(2);
Swarm c = a.intersectionSwarm(b, 3);  //swarm a intersect with swarm b, generating a new swarm c with id 3
```

##### 8. swarm union operation
```cpp
Swarm a = Swarm(1);
Swarm b = Swarm(2);
Swarm c = a.unionSwarm(b, 3);  //union swarm a and swarm b, generating a new swarm c with id 3
```

##### 9. swarm difference operation
```cpp
Swarm a = Swarm(1);
Swarm b = Swarm(2);
Swarm c = a.differenceSwarm(b, 3);  //swarm a difference with swarm b, generating a new swarm c with id 3
```

##### 10. swarm negation operation
```cpp
Swarm a = Swarm(1);
Swarm b = a.negationSwarm(2);  //negate swarm a, generating a new swarm b with id 2
```

#### 3.2 Neighbors
##### 1. create a default neighbors structure
```cpp
Neighbors<NeighborLocation> n;  //NeighborLocation type
```

##### 2. create a user-defined data type neighbors structure
```cpp
Neighbors<int> n1;  //int
Neighbors<float> n2;  //float
Neighbors<string> n3;  //string
Neighbors<Type> n4;  //user-defined type
```

##### 3. neighborsForeach function
neighborsForeach function perform a function that has no return value on each robot's data
```cpp
Neighbors<int> n;

void testforeach(int a)
{
    std::cout<<"testforeach."<<std::endl;
}

n.neighborsForeach(testforeach);
```

##### 4. neighborsMap function
neighborsMap function perform a function that has a return value on each robot's data. generating a new Neighbors structure. Key is robot's id and value is the new data after transforming.
```cpp
Neighbors<int> n;

float testmap(int a)
{
    return a+3.14;
}

Neighbors<float> b = n.neighborsMap(testmap);
```

##### 5. neighborsReduce function
neighborsReduce function perform a function on the whole neighbors data structure and get a single return value.
```cpp
Neighbors<int> n;

float testreduce(int a, float& b) 
{
    b=b+a*2; return b; 
}

float t2=0;

t2 = n.neighborsReduce(testreduce, t2);
```

##### 6. neighborsFilter function
neighborsFilter function perform the filtering operation according to user-defined judging method on each tuple. generating a new neighbors structure.
```cpp
Neighbors<NeighborLocation> n;

bool testfilter(unsigned int a, NeighborLocation b)
{
    if(b.getX()>=5)
        return true;
    return false;
}

Neighbors<NeighborLocation> c = n.neighborsFilter(testfilter);
```

##### 7. neighborsKin function
neighborsFilter function is a special filter essentially. generating a new neighbors structure. every robot in the new neighbors structure is in a specified swarm.
```cpp
Neighbors<float> n;
Neighbors<float> c=n.neighborsKin(1);  //the memeber of the neighbors n which belong to the swarm with id 1 at the same time form a new neighbors c
```

##### 8. neighborsNonKin function
opposite of the neighborsKin
```cpp
Neighbors<float> n;
Neighbors<float> c=n.neighborsNonKin(1);  //the memeber of the neighbors n which don't belong to the swarm with id 1 form a new neighbors c
```

#### 3.3 VirtualStigmergy
##### 1. create a VirtualStigmergy
```cpp
VirtualStigmergy<float> v(1);  //data type is float，id is 1
```

##### 2. virtualStigmergyPut
```cpp
v.virtualStigmergyPut("test", 3.14);  //put <"test", 3.14> into the VirtualStigmergy v
```

##### 3. virtualStigmergyGet
```cpp
v.virtualStigmergyGet("test");  //query the value with the key "test" of the VirtualStigmergy v
```

##### 4. get the size of a VirtualStigmergy
```cpp
v.virtualStigmergySize();
```

note: we could define VirtualStigmergy structure using the simple type, for example int, float, string. For the user-defined data-type, we need to use the Macro definition defined in the micros_swarm_framework to serialize in order to store and transport data of this type. We provided two Macro definition:SERIALIZE, MEMBER:
```cpp
class TestVstigDataType{
        private:
            int a_;
            float b_;
            std::string c_;

            SERIALIZE
            {
                MEMBER a_;
                MEMBER b_;
                MEMBER c_;
            }
        public:

            TestVstigDataType(){}
            TestVstigDataType(int a, float b, std::string c)
            {
                a_=a;
                b_=b;
                c_=c;
            }

            void printTestVstigDataType()
            {
                std::cout<<"a_ = "<<a_<<std::endl;
                std::cout<<"b_ = "<<b_<<std::endl;
                std::cout<<"c_ = "<<c_<<std::endl;
            }
};
```

#### 3.4 NeighborCommunication
##### 1. create a NeighborCommunication 
##### 2. publish a <key, value> tuple
##### 3. subscribe a key, execute the corresponding callback

## 4. Simulation
#### 4.1 Motion and Spatial Coordination
* start the Stage with lots of robots
```xml
roslaunch micros_swarm_framework swarm_in_stage.launch
```

* start the micros_swarm_framework kernel of all the robots
```xml
roslaunch micros_swarm_framework kernel.launch
```

* wait a minute, in order that all the kernel node is started successfully, then start the application1 node on all the robots
```xml
roslaunch micros_swarm_framework app1.launch
```

#### 4.2 Separation into Multiple Swarms
* start the Stage with lots of robots
```xml
roslaunch micros_swarm_framework swarm_in_stage.launch
```

* start the micros_swarm_framework kernel of all the robots
```xml
roslaunch micros_swarm_framework kernel.launch
```

* wait a minute, in order that all the kernel node is started successfully, then start the application2 node on all the robots
```xml
roslaunch micros_swarm_framework app2.launch
```
