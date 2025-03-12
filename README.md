# seb_naver
SEB-Naver: A SE(2)-based Local Navigation Framework for Car-like Robots on Uneven Terrain

## dependence install 
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

## run 

```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

```
source devel/setup.zsh
```

```
roslaunch plan_manage multi_sim.launch
```

