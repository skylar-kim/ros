# ROS: Robot Operating System

### ROS Workspace and ROS Package Notes
If you create a new workspace called `catkin_ws` in the path /home/user and use ROS Kinetic version, what is the command to execute to enable the workspace you created?  
`source /home/user/catkin_ws/devel/setup.bash`
setup.bash is located in the devel folder of the catkin workspace  

What does `roscd` command do?
Takes you to the last ROS workspace you have sourced its setup.bash. It does not take you to the default ROS package.  

It is a good practice to source your overlay workspace in the `.bashrc` rather than sourcing it every time when you open a new terminal.  

What is a `package.xml` file?  
<ul>- defines two types of dependencies:  
<ol>1. dependencies needed to build a package</ol>
<ol>2. dependencies needed to execute the package</ol>
</ul>
<ul>- used to describe the package and set its dependencies </ul>
<ul>- automatically generated when creating a new ROS package</ul>
<ul>- you can define a license of your package in the package.xml file</ul>  

What is the command used to create a new ROS package called my_package?  
```
cd ~/catkin_ws/src
catkin_create_pkg my_package std_msgs rospy roscpp
```
ROS workspaces:  
-A ROS workspace is a user directory that we will use to create the user ROS packages  
- A ROS workspace contains three main folders: a source, devel, and build folder  
-A ROS workspace is built using `catkin_make` command  

CMakelists.txt file:  
-The file `CMakeLists.txt` is the input to the CMake build system for building software packages  
-`CMakeLists.txt` file describes how to build the code and where to install it to  

## ROS Topics Overview
Publisher: nodes that produce information (ie. Odometry )  
Subscriber: nodes that consume information (ie. SLAM)  
For example: Odometry will publish Location information that contains (x,y,theta) to a particular topic so that the subscriber can get that information by subscribing to that particular topic.  
Note: can have multiple subscribers listening to one publisher  

### Message Communication
1. Run the Master ROS Node: The Master ROS Node maintains all the information about the ROS Network.  
2. Running the Subscriber: Node 2 is a subscriber for a particular topic. Node 2 will declare itself to the Master Node and provide relevant information (node name, topic name, message type). ROS Master Node will keep this information and wait for a publisher that meets the subscriber's criteria.  
3. Running the Publisher: Node 1 will declare itself to the Master Node, and declare relevant information about itself (name, topic name it will publish to, message type). The Master now has global knowledge of the network  
4. Provide Publisher Info: Master node tells Node 2 that there is a publisher node that Node 2 is looking for.  
5. Establish Connection Request: When Node 2 receives this information, Node 2 will send a request to connect to Node 1 using TCPROS protocol.  
6. Connection Response: Node 1 will send a response to accept the connection and start communication with Node 2. 
7. TCP Connection: Subscriber node creates a client for the publisher node using TCPROS and connects to the Publisher node. 
8. Message Transmission  

### Practical Tips to Write Publisher and Subscriber for ROS Topics
__Publisher__
1. Determine a __name__ for the topic to publish  
2. Determine the __type__ of the messages that the topic will publish  
3. Determine the __frequency__ of topic publication (how many messages per second)  
4. Create a publisher object with parameters chosen  
5. Keep publishing the topic message at the selected frequency  

__Subscriber__  
1. Identify the __name__ for the topic to listen to  
2. Identify the __type__ of the messages to be received  
3. Define a callback function that will be automatically executed when a new message is received on the topic  
4. Start listening for the topic messages  
5. Spin to listen forever (in C++)  

## Create Custom ROS Message
Defined by two thing: package_name/message_type (ie. std_msgs/String, geometry_msgs/Twist).  
Every ROS Message has some content which is defined by the type and field that has a value respect to its type. (ie. std_msgs/String message content is string data, with string being the type and field being the data.)  
__Steps to Create a New ROS Message__:  
1. create a msg folder in your package
2. create the message file with the file extension .msg
3. edit the message file by adding the elements (one per line)
4. Update the dependencies in package.xml and CMakeLists.txt
5. compile the package using catkin_make
6. make sure that your message is created using `rosmsg show`. 

Example from video: IoTSensor Message  
Message contents: id, name, temperature, humidity  
1. When creating the ros msg folder, make sure it's in the package folder and not anywhere else. (ie. ros_essentials_cpp)  
2. create the msg file IoTSensor.msg in the msg folder  
3. Updating dependencies: in package.xml, make sure the following are included: `<build_depend>message_generation</build_depend>` and `<exec_depend>message_runtime</exec_depend>`.  
In the CMakeLists.txt file, make sure the following is done: in find_package() add message_generation, in add_message_files() add IoTSensor.msg, in catkin_package() add message_runtime, and generate_messages is uncommented. 

## ROS Services
### What is a ROS Service?
1. ROS Server: the node that will provide the service  
2. ROS Client: the node that will consume the service  
Not like topic, service is __one-time__ communication. A client sends a request, then the server sends back a response.  

### When to use a ROS Service?
1. Request the robot to perform a _specific_ action (ie. path planning from point A to point B, spawn one robot in the simulator.)

### ROS Service line commands
`$ rosservice list`: lists all the services that are available
`$ rosservice info /name_of_service`: returns 
```
Node: name of the node that provides the service
URI: where the service is located on your system  
Type: the type of the message  
Args: the argument; when the client sends the request, it must send whatever arguments they must send.
```
`$ rossrv info Type`: ie. `$rossrv info turtlesim/Spawn`, returns the structure of the message. For example, `$rossrv info turtlesim/Spawn` will return:  
```
float32 x
float32 y
float32 theta
string name  
---  
string name
```
The arguments that the client needs to put down is on the upper half, and what the server will return is the lower half.  
__Example of using a service__:  
`$ rosservice call /spawn 7 7 180 t2`  
returns: `name: "t2"`  

### Steps to Write a ROS Service
The client will send a Request Message and the server will send back a Response Message.  
1. Define the service message (service file): will define the type of the message for the service request, and the type of the message for the service response. 
2. Create ROS Server node  
3. Create the ROS Client node: sends the request message and waits for the response message  
4. Execute the service  
5. Consume the service by the client  

For example:  
<ol> In ros_essentials_cpp, create a folder named 'srv'. In the srv folder, create a file called AddTwoInts.srv. In that file, we will define the type of message for the service request and response. It should look like the following:  
```
int64 a
int64 b
---
int64 sum
```
Go to package.xml and make sure the following is added:  
`<build_depend>message_generation</build_depend>` and `<exec_depend>message_runtime</exec_depend>`. These modules are responsible for reading the service file and converting it into source code for C++ and Python. 
Go to CMakeLists.txt and make sure you have:  
in `find_package` have message_generation  
in `add_service_files()` define the service file. ie. AddTwoInts.srv.

For that service file, we can find where it is created in: `~/catkin_ws/devel/include/ros_essentials_cpp`. 
__How to verify that your ROS service is working__: `rossrv list`  to see all the services created. 
</ol>
<ol>
	
</ol>


 









