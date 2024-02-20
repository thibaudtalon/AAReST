This code is to control ultra-thin deformable mirrors for the AAReST Telescope

There are two primary functions that this code performs:
- Move the mirror mounts: 3 picomotor linear actuators provide tip/tilt/piston actuations to perform rigid-body motions of the mirrors
- Deform the mirror: A total of 23 piezoletric actuators create off-neutral-axis in-plane strains which yield local bending moments and curvature on the mirror.

Many auxiliary functions provide low-level actuations and debugging capabilities

A register saves all critical values needed by the code to function. It is saved dynamically in the RAM and periodically in the EEPROM (non-volatile memory)

**main.c**
- Parsing function to read and acknoledge incoming messages
- Watchdog timer interupt to save the Register to non-volatile memory
- main function initializing the electronics and containing a round-robin loop to check for incoming messages and actuate electrodes

**Memory.h**
- Defines the register list that defines all variables used by the code
- Defines function to read/write from/to the EERPOM

**Interfaces.h**
- Controls the low-level communication interfaces (I2C, UART, SPI, ADC)
- Defines function to initialize, read, write on all the different interfaces

**Driver.h**
- Define component-level functions to interface with the electrical hardware on the Mirror electronic boards
- Defines functions to communicate wirelessly from an onboard XBee module
- Defines functions to control the High Voltage board (Initialize, activate, set voltages of variables converters, enable and check current limiter, etc.)
- Defines functions to activate the mirror separation device (mirror is constrained during launch)
- Defines functions to move picomotors
- Defines functions to read the mirror temperature from thermopiles
- Defines functions to read/write to external EEPROM (this EEPROM can store mutliple copies of the code)
- Defines functions to operate the watchdog timer

**Algorithms.h**
- Define high-level algorithms to operate the mirror
- Defines functions to accurately position picomotors using a stochastic approach
- Defines functions to actuate the electrodes on the deformable mirror
  
