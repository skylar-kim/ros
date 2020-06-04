# ROS: Robot Operating System

### ROS Workspace and ROS Package Notes
If you create a new workspace called `catkin_ws` in the path /home/user and use ROS Kinetic version, what is the command to execute to enable the workspace you created?  
`source /home/user/catkin_ws/devel/setup.bash`
setup.bash is located in the devel folder of the catkin workspace  

What does `roscd` command do?
Takes you to the last ROS workspace you have sourced its setup.bash. It does not take you to the default ROS package.  

It is a good practice to source your overlay workspace in the `.bashrc` rather than sourcing it every time when you open a new terminal.  

What is a `package.xml` file?  
<ul>defines two types of dependencies:  
<ol>dependencies needed to build a package</ol>
<ol>dependencies needed to execute the package</ol>
</ul>
 
