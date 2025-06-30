## Creating project in STM32CubeIDE

STM32CubeIDE 1.15.1

1. After create your project in STM32CubeIDE, the first step is configure the *code generation*:

    - In your `.ioc` file go to: `Project Manager > Code Generator`;
    - Check `Generate peripheral initialization as a pair of 'c/.h' files per peripheral`.

2. Once you configured the FreeRTOS using CMSIS_V2, is necessary to create the microAutoware tasks and RTOS entities:

    - Again in the `.ioc` file, go to: `Pinout & Configuration > Middleware and Software > FREERTOS > Configuration > Tasks and Queues`;
      - Create microAutoware task as below:
    
          | Field                  | Parameter          |
          | ---------------------- | ------------------ |
          | Task Name              | TaskMicroAutowa    |
          | Priority               | osPriorityNormal   |
          | Stack Size (Words)     | 4500               |
          | Entry Function         | StartMicroAutoware |
          | Code Generation Option | As external        |
          | Parameter              | NULL               |
          | Allocation             | Dynamic            |
          | Buffer Name            | NULL               |
          | Control Block Name     | NULL               |

          > It's important that microAutoware task don't stay blocked for long once the executor runs periodically. Thus, is important to evaluate `TaskMicroAutowa` priority to avoid starvation.

    - Now, the mutexes are created in `Pinout & Configuration > Middleware and Software > FREERTOS > Configuration > Mutexes`;
      - Create microAutoware mutexes as below:

        1. MutexVehicleStatus

            | Field              | Parameter          |
            | ------------------ | ------------------ |
            | Mutex Name         | MutexVehicleStatus |
            | Allocation         | Dynamic            |
            | Control Block Name | NULL               |

        2. MutexControlAction

            | Field              | Parameter          |
            | ------------------ | ------------------ |
            | Mutex Name         | MutexControlAction |
            | Allocation         | Dynamic            |
            | Control Block Name | NULL               |

    - Finally, is necessary to create the event flag object, going to `Pinout & Configuration > Middleware and Software > FREERTOS > Configuration > Events`;
      - Create microAutoware event flags as below:

        | Field              | Parameter           |
        | ------------------ | ------------------- |
        | Event flags Name   | EventsMicroAutoware |
        | Allocation         | Dynamic             |
        | Control Block Name | NULL                |



3. Copy `microAutoware_config.h` and `microAutoware.h` to `Core/Inc` project folder;

4. Copy `microAutoware.c` and `executorCallbacks.c` to `Core/Src` project folder;

5. Clone micro-ROS for microAutoware repository:

```shell
git clone -b humble git@github.com:LMA-FEM-UNICAMP/microautoware_micro-ROS_stm32.git
```


6. Configure micro-ROS for microAutoware [[1]](#ref1): [microautoware_micro-ROS_stm32 project configuration](https://github.com/LMA-FEM-UNICAMP/microautoware_micro-ROS_stm32?tab=readme-ov-file#using-this-package-with-stm32cubeide).


### Global variables

#### `xVehicleStatus`

- Collect vehicle information to publish in Autoware.

- Type: `vehicle_status`
  
  | Variable          | Type    | Description               |
  | ----------------- | ------- | ------------------------- |
  | `xLongSpeed`      | `float` | Longitudinal speed (m/s)  |
  | `xLatSpeed`       | `float` | Lateral speed (m/s)       |
  | `xHeadingRate`    | `float` | Heading rate (rad/s)      |
  | `xSteeringStatus` | `float` | Steering tire angle (rad) |

#### `xControlAction`

- Compact Autoware's high-level control references to embedded system low-level control tasks.

- Type: `control_action`
  
  | Variable            | Type            | Description                     |
  | ------------------- | --------------- | ------------------------------- |
  | `xSteeringAngle`    | `float`         | Steering tire angle (rad)       |
  | `xSteeringVelocity` | `float`         | Steering tire speed (rad/s)     |
  | `xSpeed`            | `float`         | Speed (m/s)                     |
  | `xAcceleration`     | `float`         | Acceleration (m/s^2 )           |
  | `xJerk`             | `float`         | Jerk (m/s^3)                    |
  | `ucControlMode`     | `unsigned char` | Vehicle control mode (HIL only) |

### Flags

#### `VEHICLE_NEW_DATA_FLAG`

Once the vehicle information to the Autoware are ready, the `VEHICLE_NEW_DATA_FLAG` is set to microAutoware and sends that data through ROS.

- Pooling flag to microAutoware task.

#### `AUTOWARE_NEW_DATA_FLAG`

When microAutoware receives Autoware's data and updates `xControlAction`, the `AUTOWARE_NEW_DATA_FLAG` is set to system tasks to process that information.

- Blocks control task.

#### Change control mode flags


A command to change the vehicle's control mode can be executed from either Autoware or the vehicle, i.e., there is a two-way control mode change. Therefore, the flags to activate the mode change were split into two flags for each function, with the prefix `MA_` when the command comes from the vehicle and `SYS_` when it comes from Autoware.

Both the `TO_AUTOWARE_MODE`, `TO_MANUAL_MODE`, and `EMERGENCY_MODE` flags are non-blocking and are polled at the start of the execution of the microAutoware task, as well as in the system task designed in the HIL package. The last two flags are used for fail-safe capabilities, and the last one is used exclusively for this purpose.




#### `MICRO_ROS_AGENT_ONLINE_FLAG`

When the embedded system is powered on, microAutoware tries to connect to the micro-ROS agent to establish communication with Autoware, and the system task loops are blocked. When this occurs, the `MICRO_ROS_AGENT_ONLINE_FLAG` flag is set, unblocking the system tasks. This flag has a timeout, and if the high-level system does not connect to the low-level system, the embedded system powers up in manual mode only.