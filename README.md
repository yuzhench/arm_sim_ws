# Project Title

Arm control ros2 packages 
---

## Features
- real arm control 
- simulation mode 

---

## Installation
```bash
# Clone the repository
git git@github.com:yuzhench/arm_sim_ws.git

```

### launch the whole system 
```bash 
ros2 launch robot_bringup bringup.launch.py simulation_mode:=false 
```



# Hardware Interface Code Structure

## Lowest Level: Motor Controller
- **Header:** `src/arm_sim_pkg/include/motor_controller.hpp`  
- **Implementation:** `src/arm_sim_pkg/src/motor_controller.cpp`  
- **Description:**  
  Provides low-level motor control logic, including sending commands and handling feedback directly from the simulated or physical motors.

## ROS 2 Hardware Interface
- **Header:** `src/arm_sim_pkg/include/arm_hw.hpp`  
- **Implementation:** `src/arm_sim_pkg/src/arm_hw.cpp`  
- **Description:**  
  Bridges the low-level motor controller with the ROS 2 control framework.  
  Implements the standard hardware interface functions (`read()`, `write()`, etc.),  
  enabling ROS 2 controllers to interact with the motors through the defined interfaces.

 