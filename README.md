# TK_CommHub V00.02.00
## Main communication hub for the foosball table - IMS TU Darmstadt
### Temporary Software for control of the Power Manager Units

The TK_CommHub is a temporary implementation of the communication hub that acts as the main interface between the PC and the table's power supply modules and motor controllers. The software was designed to control a power module and allow users to test and debug software with a single pole. The software consists of a state machine looping five states that allows the system to enable the power modules. Then disconnet the power supply to the motor controllers, disconnect the main power supply and discharge the capacitors before returning to an idle state. The communication between the PowerManager (PM) and the COMM_HUB runs over an I2C bus.


    State 0. && Switch-GND  -->  State 1.
    State 1. && Switch-VCC  -->  State 2.
    State 2. && Switch-GND  -->  State 3.
    State 3. && (Switch-VCC || V_PM<3.3V)  -->  State 4.
    State 4. && Switch-VCC  -->  State 0.


#### State 0:

The initial state of the state machine is an idle state. This means that the connection to charge the capacitors over a R100 resistor is disconnected as well as the connection to the motor controllers. Once the Switch is enabled (GND) the COMM_HUB performs an initiation of the system. The module reades the control register of the PM to ensure this module is connected and operational. Then it request the PM to enter Mode1 - Precharge mode - to load the capacitors. The COMM_HUB supervises the charging process and awaits for the valid flag to be raised (Supply Voltage stable) to set the Enable flag high and connect the main power supply while disconnecting the precharge line. If the process occurs without issues the system enters State 1.

-> LED - Off


#### State 1:

The second state of the state machine is the operational mode. The system remains connected to the power supply until the switch is disabled (VCC). Once this happens the COMM_HUB request the PM to enter Mode0 - Idle mode - to disconnect the main power supply from the motor controllers. This mode disables the table but does not enable the discharge of the capacitors since the only two lines available are still conected to V48+/V40+. Then the COMM_HUB enters State 2.

-> LED - On

#### State 2:

The state 2 is a hold state that allows the user to turn the V48+/V40+ supply off to disconnect the equipment. Once the buck converters are off the state is exited by enabling the switch (GND). This initiates the discharge mode by requesting the PM to enter Mode2 - Discharge mode - and reconnect the precharge line, now connected to ground over the R100 resistor. Then the state machine enters the next state

-> LED - Blink slow


#### State 3:

The State 3 is a observation mode. In this state the COMM_HUB periodically request the measurement voltage from the PM and once the voltage is below V3.3 (digital voltage supply) or the switch has been disabled (VCC), it request the PM to enter Idle mode and finish operating.

-> LED - Blink fast


#### State 4:

The last state is a reset state. If the switch has been disabled (VCC) in the previous state, state 4 is imediately exited and the state machine returns to the initial state. If the user has allowed the power supply to discharge then the user can return the switch to the inital state and reset the state machine. 

-> LED - Off



- Recomendation 1: Let the state 3 exit through the measurement of the voltage to prevent any possible discharges. The exit through the switch is reserved for debuging this software.

- Recomendation 2: If the Safety Line is pressed, the software malfunctions or the current state of the machine is not clear. Reset the PCB and run imediately through states 0 and 1. The PM has internal safety functions that disconnect the power if it is in operational mode and a drop in power is detected. By entering the Mode0 again the error flags are reset and the system can be operated normaly again.






