/**
  ******************************************************************************
  * @file    microAutoware.h
  * @brief   This file contains the declarations for FreeRTOS task microAutoware, 
  *          for implementing a vehicle interface for Autoware using micro-ros.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha 
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 9, 2024
  *          Modified: 
  ******************************************************************************
  */
#ifndef MICROAUTOWARE_H_
#define MICROAUTOWARE_H_

  // Includes -- START
  #include "cmsis_os.h"
  #include "microAutoware_config.h"

  // micro-ros includes
  #include <rcl/rcl.h>
  #include <rcl/error_handling.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <uxr/client/transport.h>
  #include <rmw_microxrcedds_c/config.h>
  #include <rmw_microros/rmw_microros.h>

  // Clock msg type
  #include <rosgraph_msgs/msg/clock.h>

  // Autoware msg types
  #include <autoware_auto_control_msgs/msg/ackermann_control_command.h>
  #include <autoware_auto_vehicle_msgs/msg/control_mode_report.h>
  #include <autoware_auto_vehicle_msgs/msg/engage.h>
  #include <autoware_auto_vehicle_msgs/msg/gear_command.h>
  #include <autoware_auto_vehicle_msgs/msg/gear_report.h>
  #include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.h>
  #include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.h>
  #include <autoware_auto_vehicle_msgs/msg/steering_report.h>
  #include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.h>
  #include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.h>
  #include <autoware_auto_vehicle_msgs/msg/velocity_report.h>
  #include <autoware_auto_vehicle_msgs/srv/control_mode_command.h>
  #include <tier4_vehicle_msgs/msg/actuation_command_stamped.h>
  #include <tier4_vehicle_msgs/msg/actuation_status_stamped.h>
  #include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.h>
  #include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.h>
  // Includes -- END

#if USE_SIM_TIME == 1
  #define SIMULATION_TIME
#endif

  // Driving mode
  #define EMERGENCY 0
  #define AUTOWARE 1
  #define MANUAL 4

  // ThreadFlags
  #define TO_AUTOWARE_MODE_FLAG 0x01
  #define TO_MANUAL_MODE_FLAG 0x10
  #define TO_EMERGENCY_MODE_FLAG 0x1000000
  #define DATA_UPDATED_FLAG 0x100
  #define MICRO_ROS_AGENT_ONLINE_FLAG 0x100000 

  // Check flag macro for bitwise comparison of ThreadFlags
  #define CHECK_FLAG(flag, input) ((flag & input) == flag && !(input >> 31))

#if TRANSPORT == UART
  #include "usart.h"
  #include "dma.h"
#endif  /* TRANSPORT == UART */

  /**
  * @name   float_bytes
  * @brief  Union to send float through UART.
  */
  typedef union{
    float fFloat;
    unsigned char ucBytes[4];
  } float_bytes;

  /**
    * @name   control_action
    * @brief  Struct with Autoware recieved data compressed for TaskControle.
    */
  typedef struct{
    float_bytes xSteeringAngle;
    float_bytes xSteeringVelocity;
    float_bytes xSpeed;
    float_bytes xAcceleration;
    float_bytes xJerk;
    unsigned char ucControlMode;
  } control_action;


  /**
    * @name   vehicle_status
    * @brief  Struct with vehicle status information recieved from CARLA.
    */
  typedef struct{
    float_bytes xLongSpeed;
    float_bytes xLatSpeed;
    float_bytes xHeadingRate;
    float_bytes xSteeringStatus;
  } vehicle_status;


  /**
    * @name   control_signal
    * @brief  Struct with vehicle control data compressed for CARLA.
    */
  typedef struct{
    float fThrottle;
    float fBrake;
    float fSteeringAngle;
    unsigned char ucHandBrake;
    unsigned char ucReverse;
    unsigned char ucManualGearShift;
    unsigned char ucGear;
    unsigned char ucControlMode;
    float fLongSpeed;
    float fLatSpeed;
    float fHeadingRate;
    float fSteeringStatus;
  } control_signal;



  // Autoware Quality of Service
  static const rmw_qos_profile_t rmw_qos_profile_autoware =
    {
      RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };

  // Function Prototypes -- START

  // Task prototype
  void StartMicroAutoware(void *argument);

  // MicroROS prototypes
  bool cubemx_transport_open(struct uxrCustomTransport * transport);
  bool cubemx_transport_close(struct uxrCustomTransport * transport);
  size_t cubemx_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err);
  size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t* err);

  void * microros_allocate(size_t size, void * state);
  void microros_deallocate(void * pointer, void * state);
  void * microros_reallocate(void * pointer, size_t size, void * state);
  void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

  // Executor callbacks (implemented in executorCallbacks.c)
  void timer_watchdog_agent_callback(rcl_timer_t * timer, int64_t last_call_time);
  void clock_callback(const void * xMsgIn);
  void control_cmd_callback(const void * xMsgIn);
  void gear_cmd_callback(const void * xMsgIn);
  void turn_indicators_cmd_callback(const void * xMsgIn);
  void hazard_lights_cmd_callback(const void * xMsgIn);
  void actuation_cmd_callback(const void * xMsgIn);
  void emergency_callback(const void * xMsgIn);
  void control_mode_cmd_callback(const void * xRequestMsg, autoware_auto_vehicle_msgs__srv__ControlModeCommand_Response * xResponseMsg);

  // Function Prototypes -- END

#endif /* MICROAUTOWARE_H_ */
