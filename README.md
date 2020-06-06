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


 









