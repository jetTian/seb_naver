# seb_naver
SEB-Naver: A SE(2)-based Local Navigation Framework for Car-like Robots on Uneven Terrain 

Paper: https://arxiv.org/abs/2503.02412

Video: https://www.bilibili.com/video/BV1yBRpYmEmX/

## Dependence 
### ackermann-msgs
```
sudo apt-get install ros-noetic-ackermann-msgs
```
### casadi
install ipopt first

```bash
git clone https://github.com/casadi/casadi.git
cd casadi 
mkdir build & cd build
cmake .. & make
sudo make install
```

## Run 

```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

```
source devel/setup.zsh
```

```
roslaunch plan_manage multi_sim.launch
```

