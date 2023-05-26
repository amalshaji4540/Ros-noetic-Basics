

## Making a workspace for ROS

```bash
cd
mkdir catkin_ws
cd catkin_ws 
mkdir src
catkin_make

```

>For executing our own packages written by ourself we need to repeat catkin_make


>To automatically execute commands type the below command a window will open add the commond on that window that is to be executed automaticaly
```bash
gedit ~/.bashrc
```

### Creating a Package inside workspace

```bash
catkin_create_pkg package_name dependency_libraries_nodes_etc
```
eg,
```bash
cd catkin_ws/src/
catkin_create_pkg my_robot_controller rospy turtlesim
```

Here we can see that workspace is created inside src file.
A workspace can have many number of packages.A package will contain nodes.

## ROS Nodes

### Creating Node inside Package

1. First we need to enter inside package folder

```bash
cd catkin_ws/src/my_robot_controller/
```

2. Make a folder for storing python codes

```bash
mkdir scripts
```

3. Enter inside scripts and make a python file 

```bash
cd scripts/
touch my_first_node.py
```

4. Make the python file executable

```bash
chmod +x my_first_node.py
```

5. Open src folder on vs code and edit the python file

```bash
cd ../../
code .
```


### Running a Node

1. Rosrun allows you to use the package name to directly run a node within a package (without having to know the package path).
Note : [[Ros imported things to note]] 

```bash
rosrun [package_name] [node_name]
```

```bash
rosrun my_robot_controller my_first_node.py
```

### Killing  a node

```bash
rosnode kill /test_node
```

### Reference Links

**1. [[Ros imported things to note]]**
2. [Reference note for nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
3. [Reference video for nodes](https://youtu.be/jWtkzDbez9M)

## ROS Topic

### Reference Links.
**1. [[Ros imported things to note]]**
2. [Reference notes for topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
3. [Reference video for topic](https://youtu.be/GAJ3c5XmJSA)

## Ros publisher with Python

### Python code
This is the code for drawing circle on turtlesim

```python
#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist

if __name__== '__main__':

rospy.init_node("draw_circle_node")

rospy.loginfo("Draw circle node has been started")

  

pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)

  

rate = rospy.Rate(2)

  

while not rospy.is_shutdown():

msg = Twist ()

msg.linear.x = 2.0

msg.angular.z = 1.0

pub.publish(msg)

rate.sleep()
```
code explanation : [[Python code explanation for drawing circle on turtlesim]]
### Reference Links.
1. [Reference video for publisher](https://youtu.be/jLKexhgAu4w)

## Ros Subscriber with Python

### Python code
This code subscribes and print values from the topic /turtle1/pose

```python
#!/usr/bin/env python3

import rospy

from turtlesim.msg import Pose

  

def pos_callback(msg : Pose):

rospy.loginfo("("+str(msg.x)+ "," +str(msg.y)+")")

  
  

if __name__=='__main__':

rospy.init_node("turtle_pose_subscriber")

sub = rospy.Subscriber("/turtle1/pose",Pose,callback=pos_callback)

  

rospy.loginfo("turtle_pose_subscriber has started")

rospy.spin()
```

### Reference Links
1. [Reference video for publisher](https://youtu.be/eciuB0p8bK0)

## Ros Subscriber publisher with python

### Python code 

```python
#!/usr/bin/env python3

import rospy

from turtlesim.msg import Pose

from geometry_msgs.msg import Twist

  

def callback_pose(pose: Pose):

cmd = Twist()

if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0 :

cmd.linear.x = 1.0

cmd.angular.z = 1.4

else:

cmd.linear.x = 5.0

cmd.angular.z = 0.0

pub.publish(cmd)

  

if __name__ == '__main__':

rospy.init_node("turtle_controller")

pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)

sub = rospy.Subscriber("/turtle1/pose",Pose,callback=callback_pose)

rospy.loginfo("Node has been started")

  

rospy.spin()
```

### Reference Links
1. [Video for publisher and subscriber ](**https://youtu.be/rtcnHzpjArM**)
