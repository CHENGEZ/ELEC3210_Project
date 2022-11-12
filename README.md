# ELEC3210_Project
* Final Project of ELEC3210 for 2022 Fall

* Each folder in the repo serves as one package for one of the tasks.

#### The file structure of one package should be as follows:
```
package_name
│   CMakelist.txt    
│   package.xml
|
└───include
│   └───package_name
|       | xxxx.h
│       |...
|  
└───src
    │   xxxx.cpp
    │   ...
```
And `package_name` refers to one of `build_map`, `key_ctrl`, `img_detection`, `follow_yellow_ball` and `judge_area`.

#### The directory structure overall (After you run `catkin_make` under `catkin_ws`) should look like the followings:
```
catkin_ws
|
└───build
│   │...  
|
└───devel
|   |...
|  
└───src
    │   CMakelist.txt
    │
    └───build_map
    |
    └───key_ctrl
    |
    └───img_detection
    |
    └───follow_yellow_ball
    |
    └───judge_area
```
Here, the `src` folder is essentially this git repo; and the `catkin_ws` should be the catkin workspace located under your `$HOME`. But the name of your workspace may or may not be `catkin_ws`, I simply followed the convention and chose this name.
