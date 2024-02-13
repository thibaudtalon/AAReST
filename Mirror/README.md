This code is to control ultra-thin deformable mirrors for the AAReST Telescope

There are two primary functions that this code performs:
- Move the mirror mounts: 3 picomotor linear actuators provide tip/tilt/piston actuations to perform rigid-body motions of the mirrors
- Deform the mirror: A total of 23 piezoletric actuators create off-neutral-axis in-plane strains which yield local bending moments and curvature on the mirror.

Many auxiliary functions provide low-level actuations and debugging capabilities
