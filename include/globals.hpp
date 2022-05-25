#ifndef _GLOBALS_HPP_
#define _GLOBALS_HPP_

// Port Definitions
#define FLPORT 21 // Front-Left drivetrain motor port
#define FRPORT 11 // Front-Right drivetrain motor port
#define MLPORT 9 // Middle-Left drivetrain motor port
#define MRPORT 1 // Middle-Right drivetrain motor port
#define BLPORT 8 // Back-Left drivetrain motor port
#define BRPORT 2 // Back-Right drivetrain motor port
#define LIPORT 4 // Lift motor port
#define RIPORT 5 // Ring Intake motor port

#define IMUPORT 19 // Port for Interial Sensor

#define LCPISTON 'A' // Lift claw piston port
#define T1PISTON 'D' // Tilter 1 piston port
#define T2PISTON 'C' // Tilter 2 piston port
#define TCPISTON 'B' // Tilter claw piston

// Constants for Subsystems
#define LIFTUP 580 // Up position of lift
#define LIFTMID 370 // Middle position of lift
#define LIFTDOWN 55 // Down position of lift
#define LIFTKP 1 // Kp value for the lift motor

#define RINGSPEED 115 // Speed of Ring Intake

#define ROTATE_BASE 0 // Rotate the base
#define MOVE_BASE 1 // Move the base
#define BRAKE_BASE 2 // Brake the base

#define BASEKP 0.4 // Kp value for the base
#define BASEKD 0.1 // Kd value for the base

#define ROTATEKP 1 // Kp value for the base rotation
#define ROTATEKD 1 // Kd value for the base rotation

#endif
