# Installation
clone this repository and build the pkg
```
git clone git@github.com:yuki-shark/jsk_path_planner.git
catkin build
```

## Dependency [WIP]
ROS packages
* [safe_footstep_planner](https://github.com/yuki-shark/safe_footstep_planner)
* [navigation](https://github.com/yuki-shark/navigation)
* [navigation_layers](https://github.com/yuki-shark/navigation_layers)

# Usage
## Launch
### simple path planning
```
roslaunch jsk_path_planner JAXON_RED_path_planner.launch
```
### path planning with a push cart
```
roslaunch jsk_path_planner JAXON_RED_push_cart.launch
```

### path planning with a wheelbarrow
```
roslaunch jsk_path_planner JAXON_RED_wheelbarrow.launch
```

## Costmap layers
files named `cosmap_{target}_{suffix}.yaml`
#### obstacles_pointcloud
set lethal cost at obstacles
#### inflation
inflate obstacle layer's grid
#### gradient
set cost depending on inclination of the ground
#### object
set cost for known object
each object cost can be set in `config/database.json`
#### grad_inflation
inflate gradient layer's grid
