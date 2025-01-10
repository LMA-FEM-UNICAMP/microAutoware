# microAutoware_stm32

microAutoware pic



# Creating project in STM32CubeIDE

- Activate separated .c/.h in code generator

## Tasks

TaskmicroAutoware

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

## Flags



## Mutex

MutexVehicleStatus

| Field              | Parameter          |
| ------------------ | ------------------ |
| Mutex Name         | MutexVehicleStatus |
| Allocation         | Dynamic            |
| Control Block Name | NULL               |

MutexControlAction

| Field              | Parameter          |
| ------------------ | ------------------ |
| Mutex Name         | MutexControlAction |
| Allocation         | Dynamic            |
| Control Block Name | NULL               |

# Integrating microAutoware

- Script that copies libs to project

# HIL Mode 

- Use sim time
- TaskControle