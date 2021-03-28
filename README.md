# injector_machine_v2.0
This repository contain source files for injector molding machine controller based on STM32F072RB microcontroller.

## project purpose

  The purpose of this project was to create molding injector machine controller. The main requirements are reliable operation of device and safety of operators.
  
## functionalities

  - driving MOSFET power switches to control solenoid valves to control machine actuators,
  - implement machine cycle algorithm based on setups given by device operator in auto or semi-auto mode,
  - handling and supervision of actions taken by operator in manual mode,
  - counting number of fabricated elements,
  - displaying basic informations about machine state,
  - command line interface for debugging and servicing purposes,
  - reliable operation in presence of electromagnetic interferences.
  
## software implementation
  
   To achieve high reliability, the user interface code runs in parallel with finite state machine responsible for maintaining machine workflow.
   All the non time critical tasks related with handling: cycle counter, LCD display and user menu are based on DMA and FIFO queues. Finite state machine runs alternately
   with short and concise tasks pop one-by-one from queue to achieve fsm response time shorter than 1 milisecond without use of external interrupts for handling limit switches
   transitions. Polling strategy has been chosen for  reading position of machine parts to minimize probability of false reading due to electromagnetic interferences.
   
   Command line interface based on UART transmission have been developed for debugging and servicing purposes. One can connect computer
   to device using USB interface to see informations about error codes, time elapsed from reset and current execition line. For example:
   
   ![alt text](https://github.com/leszqq/injector_machine_v2.0/blob/master/resources/cli_scr.png "CLI")

  To achieve cohesion of source files, they are organized in similiar way. Every module is based on "base" static structure and set of access functions.
  
  Sketchy finite state machine work flow is shown below:
  
   ![alt text](https://github.com/leszqq/injector_machine_v2.0/blob/master/resources/flow_diagram.png "flow")
  
## hardware implementation 

  Hardware part of project is based set of on custom printed circuit boards tailored to application requirements.
  The main pcb consists of set of MOSFET solenoid valve drivers, power supply circuit and stm32f072RB evaluation board.
  Evaluation board have been used to speed up pcb design process. Top pcb layer is used mainly for ground shielding, with SMD
  components located on bottom layer. Ground plane is connected with metal case through MLCC capacitors to improve shielding and 
  prevent DC current flow in case as well.
  
  ![alt text](https://github.com/leszqq/injector_machine_v2.0/blob/master/resources/main_pcb.png "Main pcb")
  
  Main PCB is connected to limit switches and control buttons using screw connectors. ISP wires have been used for connections
  with LCD, seven segment display and PCB containing LEDs indicating state of given solenoid valves and limit switches.
  All the MOSFET drivers are fused seperately.
    
  
