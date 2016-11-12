^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package micros_swarm_framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.15 (2016-11-12)
-----------
* release to kinetic rosdistro
* repair bugs in getParam
* repair bugs in opensplice dds publisher and subscriber
* repair bugs in MsgQueueManager
* complete MsgQueueManager class
* complete msg queue cache

0.0.14 (2016-09-09)
-----------
* complete neighbor communication
* simplify the user interface functions
* improve the security of multi-threading
* limit communication range
* add PacketParser class
* performance optimization
* optimize the RuntimePlatformKernel

0.0.13 (2016-08-16)
-----------
* improve neighbor communication mechanism, add Broadcaster and Listener class
* add Application class, which is convenient for users to develop app
* change cmake file
* change README file
* repair bugs in app2

0.0.12 (2016-07-18)
-----------
* reconstructe the whole architecture using nodelet to improve performance
* is extensible to Opensplice DDS
* add neighbor communication mechanism
* standard the interface definition
* repair bugs in cmake file

0.0.11 (2016-07-18)
-----------
* repair bugs in cmake file

0.0.10 (2016-07-18)
-----------
* reconstructe the whole architecture using nodelet to improve performance
* is extensible to Opensplice DDS
* add neighbor communication mechanism
* standard the interface definition

0.0.9 (2016-07-07)
-----------
* Add another experiment which is called "Flocking"
* Add linear velocity in the Base and NeighborBase classes
* Add the breakupSwarm function in the "swarm.h" header

0.0.8 (2016-06-14)
-----------
* No longer using the TCP protocol in ROS, UDP protocol is used instead
* Location class defined in the "data_type.h" header is renamed to Base
* NeighborLocation class defined in the "data_type.h" header is renamed to NeighborBase

0.0.7 (2016-05-30)
-----------
* optimize the installation

0.0.6 (2016-05-17)
-----------
* optimize the kernel code
* change the readme file

0.0.5 (2016-05-13)
-----------
* change the license to BSD
* perfect the package information
* optimize the kernel
* optimize code structure
* simplify synchronization protocol

0.0.4 (2016-05-11)
-----------
* repair the cmake bug

0.0.3 (2016-05-11)
-----------
* repair the headers bug

0.0.2 (2016-05-10)
------------------
* version 1.0
* Contributors: xuefengchang
