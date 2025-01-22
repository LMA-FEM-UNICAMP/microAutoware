/**
  ******************************************************************************
  * @file    microAutoware_config.h
  * @brief   This file contains the definitions of parameters that configures
  * 		 microAutoware.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  December 10, 2024
  *          Modified: January 22, 2025
  ******************************************************************************
  */#ifndef MICROAUTOWARE_CONFIG_H_
#define MICROAUTOWARE_CONFIG_H_

  #define UART 1
  #define UDP 2
  #define USB 3
  #define OTHER 0

  // microAutoware node name
  #define NODE_NAME "vehicle_interface"

  // microAutoware transport layer
  #define TRANSPORT UART

  // microAutoware UART Handle (for TRANSPORT == UART)
  #define HUART huart3

  // Use simulation time (used for HIL simulation)
  #define USE_SIM_TIME 1

  // Timeout for sync timestamp with ROS
  #define TIMEOUT_TS_SYNC 60

  // Timeout ping to micro-ros agent
  #define WATCHDOG_AGENT_TIMEOUT 1000

  // Executor spin once timeout in ms
  #define EXECUTOR_SPIN_TIME 20

#endif
