# ROS: Robot Operating System
### How to add an existing project to GitHub using the command line:
(https://help.github.com/en/github/importing-your-projects-to-github/adding-an-existing-project-to-github-using-the-command-line)
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
1. In ros_essentials_cpp, create a folder named 'srv'. In the srv folder, create a file called AddTwoInts.srv. In that file, we will define the type of message for the service request and response. It should look like the following:  
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

2. Write ROS Service (Client/Server) in Python
__Server__: When writing a server, we will write something similar to a callback function for a subscriber node, which is known as a _handle_ function that is going to process the incomming message, formulate the response, and sends back the response to the client.  
__Primary function__ (in add_server.py): `add_two_ints_server('name of service', type of message to exchange, )`  
`rospy.Service`: creates a server that will be listening to incoming requests  
At the beginning of a python file, make sure to add the following so that we can access the service and messages defined for this service:
```python
from ros_essentials_cpp.srv import AddTwoInts
from ros_essentials_cpp.srv import AddTwoIntsRequest
from ros_essentials_cpp.srv import AddTwoIntsResponse
```
The `AddTwoIntsRequest` and `AddTwoIntsResponse` is in the devel folder when we compile the package.  
__Handle Function__: performs the service requested 
```python
def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    # returns the response, contains only 1 argument
    return AddTwoIntsResponse(req.a + req.b) 
```

__Client__: sends the request to the server for the service  
At the beginning of a python file, make sure to add the following so that we can access the service and messages defined for this service:
```python
from ros_essentials_cpp.srv import AddTwoInts
from ros_essentials_cpp.srv import AddTwoIntsRequest
from ros_essentials_cpp.srv import AddTwoIntsResponse
```
The `AddTwoIntsRequest` and `AddTwoIntsResponse` is in the devel folder when we compile the package.  
The rest of the comments are in the `add_client.py` program.  
__Note__: When you run `$ rosrun ros_essentials_cpp add_two_ints_server` (which is the cpp server) and `$ rosrun ros_essentials_cpp add_client.py 7 9` the client/server still works even though the programs are written in different languages. This is possible because they exchange serialized messages. 

### Writing the turtlesim cleaner

Cleaning Application Overview (need these methods):  
```c++
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message);
double setAbsoluteOrientation(double desired_angle);
// double clean();
double getDistance(double x1, double y1, double x2, double y2);
// void moveGoal(turtlesim::Post goal_pose, double distance_tolerate);
```
How all ROS nodes should start:  
```c++
#include "ros/ros.h"

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;

	return 0;
}
```
In order to implement speed, must publish the messages. But first, must find what what messages to publish and where:  
`$ rosrun turtlesim turtlesim_node`
`$ rostopic list`  
/turtle1/cmd_vel: this is the topic that makes the robot move  
`$ rostopic info /turtle1/cmd_vel` :   
Type: geometry_msgs/Twist (this is the type of message is published to the /cmd_vel topic)  
`$ rosmsg show geometry_msgs/Twist`: show what the message type contains  
```
geometry_msgs/Vector3 linear  
  float64 x
  float64 y  
  float64 z  
geometry_msgs/Vector3 angular  
  float64 x
  float64 y  
  float64 z  
```
So, we must publish Twist message in order to make the turtlesim move.  
The rest of the notes are on `robot_cleaner.cpp`. Here are some further, non-code, but still important notes:  
When you compile with `catkin_make`, you will not be able to run the program (or even compile successfully).  
In your __CMakeLists.txt__: 
```
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlesim_cleaner
  CATKIN_DEPENDS other_catkin_pkg roscpp rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)
...
## Declare a cpp executable
add_executable(robot_cleaner_node src/robot_cleaner.cpp)
target_link_libraries(robot_cleaner_node ${catkin_LIBARIES})
add_dependencies(robot_cleaner_node beginner_tutorials_gencpp)
```
In your package.xml file:  
```xml
<build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>message_generation</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
```

## Network Configuration in ROS
Although there is already a guide on Confluence, that is between two physically separate computers.  
This guide will show how to configure the ROS Network Configuration between 2 VMs, the __User Workstation__ and the __Robot Machine__  

Goal: control the Robot Machine remotely from the User Workstation  

Note: ROS must be installed on both the Robot Machine and the User Workstation  

#### Configuring the Robot Machine
In the Robot Machine, open the bashrc file: `$ gedit .bashrc`  

In the Robot Machine, we need to add some IP addresses. In order to find out which IP addresses to add, type `$ ifconfig` into the Robot Machine terminal window.  

Copy the IP address that is listed after `inet addr:` and paste it into the .bashrc file in the following format: `export ROS_HOSTNAME= ip address`, `export ROS_IP = ip address`, `export ROS_MASTER_URI=http://localhost:11311`    

Add the next three lines into the Robot Machine .bashrc file to display the environment variables that we have set when you open a new terminal:
```
echo "ROS_HOSTNAME: "$ROS_HOSTNAME
echo "ROS_IP: "$ROS_IP
echo "ROS_MASTER_URI: "$ROS_MASTER_URI
```
#### Configuring the User Workstation: 
In the User Workstation, open the bashrc file: `$ gedit .bashrc`  

Find the IP address of the user workstation by typing `$ ifconfig` into the User Workstation terminal window. Copy the ip address that's listed after `inet addr:`.  

Paste the ip address into the .bashrc file in the following format: `export ROS_HOSTNAME= ip address`, `export ROS_IP = ip address`. However, the ROS_MASTER_URI will be different.  

The IP address listed after ROS_MASTER_URI in the .bashrc file will be the ip address found on the Robot Machine. In the .bashrc file, it will look like the following: `export ROS_MASTER_URI = http://ip address of robot machine` except replace the words "ip address of robot machine" with the actual ip address.  

#### Communicating between User Workstation and Robot Machine
1. Run `$ roscore` on the Robot Machine  
2. Run `$ rosrun turtlesim turtlesim_node` on the Robot Machine  
3. Run `$ rostopic list` on the Robot Machine. A list of the available topics should print out.   
4. Run `$ rostopic list` on the User Workstation. The same list of the available topics should print out. Note: do NOT run `$ roscore` on the User Workstation.  

When the ROS master node is terminated on the Robot Machine, and you try to run `$ rostopic list` on the Robot Machine again, an error message saying `ERROR: Unable to communicate with master!` will pop up. This is because the workstation is no longer connected to the ROS Master of the robot. 

#### VM Configuration
If you did all those steps and the test communication above, and you get an error, try the following:  
[VMware PLayer Bridged Network not Working](https://www.youtube.com/watch?v=fheU2ER9tss)  
Do this configuration for both the User Workstation VM and the Robot Machine VM

## Launch File
What is a Launch file?  
- __launch file__: is a xml document, which specifies: which nodes to execute, their parameters, what other launch files to include. Has a .launch extension  
- __roslaunch__: is a program that easily launches multiple ROS nodes  

### How to write a launch file: 
First, create a launch directory in the package you are working in. For example, if you are working in the `turtlesim_cleaner` package, then you type the following into the terminal:  
` user@ubuntu:~/catkin_ws/src/turtlesim_cleaner$ mkdir launch` 
` user@ubuntu:~/catkin_ws/src/turtlesim_cleaner/launch$ touch clean_py.launch` 
```xml
<launch>
  <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>
  <node pkg="turtlesim" type="turtle_teleop_key" name="turtlesim_teleop_node" output="screen"/>
  <node pkg="turtlesim_cleaner" type="clean.py" name="clean" output="screen"/>
```
`pkg`: specifies the package name that the node is under  
`type`: name of the executable file. For c++ files, the formatting is like the first two nodes, for python scripts, the formatting is like the last node.  
`name`: a custom name that we can specify for that node  

To use the launch file, compile your workspace using `$ catkin_make` and launch it by `$ roslaunch package_name launch_file_name.launch`  

### Putting a Launch File Within a Launch File:  
Let's say we are creating another launch file called `launch_all.launch` and it will have the launch file we already made, `clean_py.launch` and another node. The format will look like the following
```xml
<launch>
  <include file="$(find turtlesim_cleaner)/launch/clean_py.launch"/>
  <node pkg="ros_essentials_cpp" type="turtlesim_cleaner.py" name="turtlesim_cleaner_node_py"/>
</launch>
```
The `find` command will return the absolute path of turtlesim_cleaner (or whwatever other package you put there) and then further specify the path of the launch file within the package.  

### Parameters in Launch Files:

If you want to have parameters in your nodes, then you must do the following:  
1. In the launch file, define the parameters _BEFORE_ the node is called so that they are registered in the parameter server. Also, make sure to add `output="screen"` to see the print statements. 
```xml
<launch>
  <include file="$(find turtlesim_cleaner)/launch/clean_py.launch"/>
  <param name="x_goal" value="2.0"/>
  <param name="y_goal" value="3.0"/>
  <node pkg="ros_essentials_cpp" type="turtlesim_cleaner.py" name="turtlesim_cleaner_node_py" output="screen"/>
</launch>
```
2. In the package src code, define the parameters. For example, if you wanted to use parameters in `turtlesim_cleaner.py`, define as follows:
```python
if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        #move(1.0, 2.0, False)
        #rotate(30, 90, True)

        x_goal = rospy.get_param("x_goal")
        y_goal = rospy.get_param("y_goal")

        print('x_goal = ', x_goal)
        print('y_goal = ', y_goal)

        go_to_goal(x_goal, y_goal) 
        #setDesiredOrientation(math.radians(90))
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
```

## OpenCV for ROS
- Open Source CV library  
- BSD License  
- free for both academic and commerical use  
- C++/Python/Java  
- Windows, MacOS, Linux, iOS, Android
- Strong focus on real time (written in C++ and optimized)

__Image Segmentation__: process of partioning a digital image into multiple segmentation, used to locate objects and boundaries (lines, curves, etc) in images  
__Image Thresholding__: from a grayscale image, thresholding can be used to create binary images, any color above the threshold is white, and any color below is black  
__Object Detection and Recognition__: detecting instances of semantic objects of a certain class in digital images and videos  
__Drawing__: draw different shapes (circle, lines, polyglons, etc)  
__Edge detection__: works by detecting discontinuities in brightness, used for image segmentation and data extraction  
__Video/Image Input Output__: openCV makes it simple to read/write images or videos  