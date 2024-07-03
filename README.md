# rescue_vision_24
**Author** : <a href="https://github.com/mkdir-sweetiepie"><img src="https://img.shields.io/badge/Ji Hyeon Hong-white?style=flat&logo=github&logoColor=red"/></a>

## Description
Victim Box mission. Used realsense_camera. 
## Table of Contents
- [Build & Usage](#build--usage)

## Build & Usage
#### Build from source code
```shell
$ cd ~/${workspace_name}_ws/src
$ git clone https://github.com/RO-BIT-Intelligence-Robot-Team/rescue_vision_24.git
$ cd ..
$ catkin_make
```

## rescue_vision_ui는 테스트 용이라서 삭제 후 사용하시면 됩니다.
rosrun rescue_vision_ui rescue_vision_ui 
대회에서는 사용 X

#### How to use (test code) 
```
$ roslaunch rescue_vision_24 rescue_vision_24.launch 
$ rosrun rescue_vision_ui rescue_vision_ui
```
