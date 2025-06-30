# microAutoware: An Autoware vehicle interface for real-time embedded systems

microAutoware is a package based in micro-ROS to bring the Autoware Core/Universe inside a microcontroller with Hardware-In-the-Loop (HIL) validation support.

<div align="center">

[![Linux](https://img.shields.io/badge/os-ubuntu22.04-blue.svg)](https://www.linux.org/) [![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html) [![STM32](https://img.shields.io/badge/microcontroller-STM32-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)

</div>


<p align="center">
  <img width="80%" height="80%" src="docs/figures/testbed.png">
</p>

<p align="center">
  <img width="50%" height="40%" src="docs/figures/components_dark.png#gh-dark-mode-only">
</p>
<p align="center">
  <img width="50%" height="40%" src="docs/figures/components.png#gh-light-mode-only">
</p>

---

## Introduction

Looking to bring standardization to the Autoware Core/Universe interface with the vehicle's low-level modules, microAutoware utilizes micro-ROS to embed the vehicle interface module inside the microcontroller, using the default ROS2 topics and services to control the vehicle. Another advantage of this package is its independence from the physical layer, where data between Autoware and microAutoware can be transmitted through UART (as validated in this repository), Ethernet, or other micro-ROS-compatible protocols.

As of now, microAutoware is available only for STM32 microcontrollers using FreeRTOS, but there are future plans to expand compatibility to other hardware families and RTOSes.


<p align="center">
  <img src="docs/figures/architecture_dark.png#gh-dark-mode-only">
</p>

<p align="center">
  <img src="docs/figures/architecture.png#gh-light-mode-only">
</p>

In FreeRTOS, microAutoware is implemented as a task that communicates and synchronizes with other tasks using event flags and global variables protected by a mutex.

<p align="center">
  <img width="50%" height="40%" src="docs/figures/RTOS_blockdiagram_dark.png#gh-dark-mode-only">
</p>

<p align="center">
  <img width="50%" height="40%" src="docs/figures/RTOS_blockdiagram.png#gh-light-mode-only">
</p>

## Dependencies

- ROS 2 Humble
- Autoware Core/Universe 2024.01
- STM32 microcontroller that supports micro-ROS
- For HIL testbed:
  - CARLA Simulator 0.9.15


## microAutoware lib

- `microAutoware.h`
  - Header file including: micro-ROS libraries, Autoware libraries, defining flags, declaring structs typedefs and function prototypes.
- `microAutoware.c`
  - microAutoware task, that implement the micro-ROS node and declare package's global variables. 
- `microAutoware_config.h`
  - Header file to configure microAutoware parameters, as timeouts, timing, transport layer and node name.
- `executorCallbacks.c`
  - Declaration of micro-ROS callback functions for timers, topics and services.


## [Creating project in STM32CubeIDE](docs/CREATE_PROJECT.md)

## [HIL Mode](docs/HIL.md)


## Citation

```
@INPROCEEDINGS{darocha2025microautoware,
      author    = {{da Rocha}, Gabriel Toffanetto França and Bacurau, Rodrigo Moreira and Ferreira, Janito Vaqueiro},
      booktitle = {2025 IEEE Intelligent Vehicles Symposium (IV)},
      title     = {microAutoware: An Autoware Vehicle Interface Designed for Real-Time Embedded Systemswith Hardware-In-the-Loop (HIL) Support},
      year      = {2025},
    }
```

## References


<a id="ref1"></a> [1] micro-ROS for STM32CubeMX/IDE. Available in: [github.com/micro-ROS/micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils).


<a id="ref2"></a> [2] G. Kaljavesi, T. Kerbl, T. Betz, K. Mitkovskii and F. Diermeyer, "CARLA-Autoware-Bridge: Facilitating Autonomous Driving Research with a Unified Framework for Simulation and Module Development," 2024 IEEE Intelligent Vehicles Symposium (IV), Jeju Island, Korea, Republic of, 2024, pp. 224-229, doi: 10.1109/IV55156.2024.10588623. Available in: [github.com/TUMFTM/Carla-Autoware-Bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge).
