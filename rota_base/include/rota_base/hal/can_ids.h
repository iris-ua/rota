
#pragma once

// ****************************************************************************
// IDs CAN
// Producer: Gateway
// Data length = 6
// Data[0..1] = SP_motor1 (int/short, little endian)
// Data[2..3] = SP_motor2 (int/short, little endian)
// Data[4..5] = SP_motor3 (int/short, little endian)
#define CANID_MOTOR	0x010

// Data length = 3
// Data[0..1] = Steering position (int/short little endian)
// Data[2] = speed (byte)
#define	CANID_STEERING	0x011

// Data length = 3
// Data[0] = command (0: direction, 1: Stop light, 2: front lights)
// Data[1] = dependent on command
// Data[2] = time
#define CANID_LIGHTS	0x012

// Data length = 6
// Data[0..1] = Pan (int/short little endian)
// Data[2..3] = Tilt (int/short little endian)
// Data[4] = pan velocity (byte)
// Data[5] = tilt speed (byte)
#define CANID_PAN_TILT  0x013

// Producer: Motors
// Data length = 2
// Data[0..1] = Encoder value, motor1 (int/short, little endian)
#define CANID_ENC	0x020

// Producer: Line sensors board
// Data length = 2
// Data[1..0] = oldest sensing, data[3..2], data[5..4], data[7..6]
// Data[0] = MS3 MS3 MS2 MS1 MS0 RS2 RS1 RS0
// Data[1] = LS2 LS1 LS0
// little endian
#define CANID_LINESENS  0x030

// Producer: Steering control board
// Data length = 4
// data[0] = groundSensor(SENS_ESQ)
// data[1] = groundSensor(SENS_DIR)
// data[2] = sens_obst(SENS_ESQ)
// data[3] = sens_obst(SENS_DIR)
#define CANID_SENSORS	0x040

// Producer: Battery monitor board
// Data length = 2
// data[0] = CONTROL battery voltage value
// data[1] = POWER battery voltage value
#define CANID_BATS	0x050

// Producer: Dynamixel control board
// Data lenght = 4
// data[0..1] = pan position feedback (int/short little endian)
// data[2..3] = tilt position feedback (int/short little endian)
#define CANID_RD_PANTILT 0x060

// Producer: Dynamixel control board
// Data lenght = 2
// data[0..1] = steering position feedback (int/short, little endian)
#define CANID_RD_STEERING 0x061

#define ERROR_CODE_FEEDBACK 0x0FFF
