---
layout: post
title:  "Creating ROS2 Workspace"
date:   2024-07-28 23:50:19 +0530
categories: ros2
---

## Install colcon extenstions

```shell
sudo apt install python3-colcon-common-extensions
```

## Create Workspace directory

```shell
mkdir ~/ros2_ws
cd ~/ros2_ws
```

## Initializing the package

### Step 1: Create src folder inside the Workspace directory

```shell
mkdir ~/ros_ws/src
```

#### The output tree should be as such

```shell
.  
└── src  
  
2 directories, 0 files
```

### Step 2: Run the initialization command

```shell
colcon build --symlink-install
```

#### Tree output
```shell
.  
├── build  
│   └── COLCON_IGNORE  
├── install  
│   ├── COLCON_IGNORE  
│   ├── local_setup.bash  
│   ├── local_setup.ps1  
│   ├── local_setup.sh  
│   ├── _local_setup_util_ps1.py  
│   ├── _local_setup_util_sh.py  
│   ├── local_setup.zsh  
│   ├── setup.bash  
│   ├── setup.ps1  
│   ├── setup.sh  
│   └── setup.zsh  
├── log  
│   ├── build_2024-07-28_18-48-39  
│   │   ├── events.log  
│   │   └── logger_all.log  
│   ├── COLCON_IGNORE  
│   ├── latest -> latest_test  
│   ├── latest_build -> build_2024-07-28_18-48-39  
│   ├── latest_test -> test_2024-07-28_18-48-55  
│   └── test_2024-07-28_18-48-55  
│       ├── events.log  
│       └── logger_all.log  
└── src  
  
10 directories, 17 files
```

### Step 3: Test the build

```shell
colcon test
```

### Step 4: Source the setup.bash file to use the workspace

#### In the workspace folder itself, run the command below

```shell
source install/setup.bash
```

#### To use this workspace on each run, run the below command

```shell
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

Now, the terminal on opening will default to this workspace! 😀

# Voila! Now you can start building your ROS packages
