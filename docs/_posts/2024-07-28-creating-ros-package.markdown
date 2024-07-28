---
layout: post
title:  "Creating ROS2 Package"
date:   2024-07-28 23:50:19 +0530
categories: ros2
---
## We will be using Python for our packages!

### Note: 
##### We will be using **package_name** to denote the package name. It can be changed to the name you wish 

###  Step 1: Go to the src folder of your workspace

```shell
cd ~/ros2_ws/src
```

### Step2: Creating the package!

```shell
ros2 pkg create --build-type ament_python package_name
```

After running this command you will get a warning

```shell
[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identifiers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0
```

This is because we are not setting the licence type. To fix this, modify the previous command with:

```shell
ros2 pkg create --build-type ament_python --licence Apache-2.0 package_name
```

The '**Apache-2.0**' can be replaced with the following:

 - **BSL-1.0**
 - **BSD-2.0**
 - **BSD-2-Clause**
 - **BSD-3-Clause**
 - **GPL-3.0-only**
 - **LGPL-3.0-only**
 - **MIT**
 - **MIT-0**

### Step 3: Building the package

#### 1. Move to the main workspace folder

```shell
cd ~/ros2_ws
```

#### 2. Build all packages using the command below

```shell
colcon build
```

#### To build our package only

```shell
colcon build --packages-select package_name
```

### Step 4: Use the package!

To use the package, after creating a node, say **node_name**, we run the command

```shell
ros2 run package_name node_name
```


# And Its Done! Now you know how to create a package in ROS2.