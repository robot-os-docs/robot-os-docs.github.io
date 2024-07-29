---
layout: post
title:  "Creating a Simple ROS2 Publisher using Python"
date:   2024-07-28 23:50:19 +0530
categories: ros2
---

### The main packages used here are:
- rclpy
- std_msgs

### Where to create the node?
The node is created in the folder with the package name created inside the package. This is relatively different from creating a scripts folder as we do in ROS1.

From the src folder, if we have a package named talker-listener, to create the file pub.py, we use
```shell
touch talker-listener/talker-listener/pub.py
```
This file is the place where we will be creating the publisher node.

For the package hello_world, we would have the folder structure as such
```shell
.  
├── hello_world  
│   ├── __init__.py  
│   ├── log  
│   │   ├── COLCON_IGNORE  
│   │   ├── latest -> latest_list  
│   │   ├── latest_list -> list_2024-07-28_21-15-24  
│   │   └── list_2024-07-28_21-15-24  
│   │       └── logger_all.log  
│   ├── pub.py  
│   └── sub.py  
├── package.xml  
├── resource  
│   └── hello_world  
├── setup.cfg  
├── setup.py  
└── test  
   ├── test_copyright.py  
   ├── test_flake8.py  
   └── test_pep257.py  
  
8 directories, 12 files
```
This is the final structure after building the package

## To create the publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('publisher')
pub = node.create_publisher(String,'publisher_topic',10)
while rclpy.ok():
	msg = String()
	msg.data = "Hello World"
	node.get_logger().info('Publishing: "%s"' % msg.data)
	pub.publish(msg)
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

### Line-By-Line explanation

```python
import rclpy
```
This imports the main ROS2 package which is used throughout the code

```python
from rclpy.node import Node
```
The Node object helps us create the node. Its initialization is synonymous to the **rospy.init_node()** function.

```python
from std_msgs.msg import String
```
This imports the String object from the standard messages in-built into ros2.

```python
rclpy.init()
```
This function initializes the file for ROScompatibility. Component of the init_node function from ROS1.

```python
node = Node('publisher')
```
The node variable is where the Node object is initialized with the node name

An alternative to this method is:
```python
node = rclpy.create_node('publisher')
```
This is the one and same as Node initialization from object.

```python
pub = node.create_publisher(String,'publisher_topic',10)
```
The function **rospy.Publisher** now works in this form. The only difference in the arguments of the function is that the **type of data** sent comes first, then the **topic** and at last the queue_size, which has been renamed to **qos_profile**, but function in the same way.

```python
while rclpy.ok():
```
The conditional function rclpy.ok is the equivalent to rospy.is_shutdown. It tells us if the node has been shutdown or not.

```python
msg = String()
```
Here, we simply initialize the String object.

```python
msg.data = "Hello World"
```
Assigning the string 'Hello World' to the data field of the object completes the initialization of the message.

```python
node.get_logger().info('Publishsing: "%s"'%msg.data)
```
Instead of using **rospy.loginfo** as we use in ROS1, here we have to use the node itself to implement the get_logger function, which can in turn provide the info function, working in the same way as the prior version.

```python
pub.publish(msg)
```
This function publishes the msg to the topic we defined earlier. It remains the same as in ROS1.

```python
rclpy.spin(node)
```
Now, instead of simply using rospy.spin, we use this spin function to continuously run a just single node.

```python
node.destroy_node()
```
We have the option to kill the node once the work is over. This helps us free up memory.

```python
rclpy.shutdown()
```
In the final line of the code, we close the connection of the node, This is a mandatory line in all of the packages you create.

## To use the publisher

We need to make some edits to files.

### setup.py

```python
entry_points = {
	'console_scripts':[
		'talker = talker-listener.pub:main',	
	]
}
```
In the file, edit the entry_points variable to look like this. For each node you create, you have to create this change to the setup.py file. This gives you the option to name your node and the right side of the assign operation provides the main function path of our pub.py file.

## Build the package

Go back to your ros2_ws directory (will be mentioned as root directory henceforth).

```shell
cd ~/ros2_ws/
```

Run the following command to build the package

```shell
colcon build
```

## Run the node
After the above steps, you can run the code using the run statement

```shell
ros2 run talker-listener talker
```

This starts our node execution!

# There ya go! We have created our first traditional hello world publisher and run it!