#ifndef _IIS_ROBOT_DEP
#define _IIS_ROBOT_DEP

#include <math.h>

#define SLOW_SPEED 0
#define NORMAL_SPEED 1
#define FAST_SPEED 2

#define CYCLE_TIME_IN_SECONDS 0.001

// rightR2WTM = [[-0.72281, 0.60291, 0.33770, -0.49622]; [0.00929, 0.48016, -0.87713, 0.59918]; [-0.69098, -0.63714, -0.34147, 0.63349]; [0.00000, 0.00000, 0.00000, 1.00000]];
// rightW2RTM = [[-0.72281, -0.00929, -0.69098, 0.08462]; [0.60291, 0.48016, -0.63714, 0.41509]; [0.33770, -0.87713, -0.34147, 0.90945]; [0.00000, 0.00000, 0.00000, 1.00000]];
// leftR2WTM = [[-0.879651, 0.336431, 0.336196, -0.492322]; [0.426848,  0.246623, 0.870044, 0.754155]; [0.209797,  0.90884,  -0.360547,  0.629509]; [0.00000, 0.00000, 0.00000, 1.00000]];
// leftW2RTM = [[-0.87965, 0.42685, 0.20980, -0.88705]; [0.33643, 0.24662, 0.90884, -0.59248]; [0.33620, 0.87005, -0.36055, -0.26366]; [0.00000, 0.00000, 0.00000, 1.00000]];

#define RR2WTM static double (*rightR2WTM)[4] = new double[4][4]{{-0.72281, 0.60291, 0.33770, -0.49622}, {-0.00929, 0.48016, -0.87713, 0.59918}, {-0.69098, -0.63714, -0.34147, 0.63349}, {0.00000, 0.00000, 0.00000, 1.00000}}
#define RW2RTM static double (*rightW2RTM)[4] = new double[4][4]{{-0.72281, -0.00929, -0.69098, 0.08462}, {0.60291, 0.48016, -0.63714, 0.41509}, {0.33770, -0.87713, -0.34147, 0.90945}, {0.00000, 0.00000, 0.00000, 1.00000}}
#define LR2WTM static double (*leftR2WTM)[4] = new double[4][4]{{-0.879651, 0.336431, 0.336196, -0.492322}, {0.426848,  0.246623, 0.870044, 0.754155}, {0.209797,  0.90884,  -0.360547,  0.629509}, {0.00000, 0.00000, 0.00000, 1.00000}}
#define LW2RTM static double (*leftW2RTM)[4] = new double[4][4]{{-0.87965, 0.42685, 0.20980, -0.88705}, {0.33643, 0.24662, 0.90884, -0.59248}, {0.33620, 0.87005, -0.36055, -0.26366}, {0.00000, 0.00000, 0.00000, 1.00000}}

inline double to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}

inline double to_radians(double degrees) {
    return degrees * M_PI / 180;
}

/* error codes (taken from iis_kukie!) */
#define OROCOS_OK_CODE 0
#define OROCOS_OK_MSG "ok"

#define OROCOS_MAX_JNT_LIMIT_CODE 1
#define OROCOS_MAX_JNT_LIMIT_MSG "maximum joint movement limit exceeded"

#define OROCOS_MAX_CART_LIMIT_CODE 2
#define OROCOS_MAX_CART_LIMIT_MSG "maximum cartesian movement limit exceeded"

#define OROCOS_WRONG_COMMAND_MODE_CODE 3
#define OROCOS_WRONG_COMMAND_MODE_MSG "robot is operated in wrong command mode"

#define OROCOS_WRONG_JOINT_SIZE_CODE 4
#define OROCOS_WRONG_JOINT_SIZE_MSG "Size of joint array not equal to 7"

#define OROCOS_PORT_BINDING_FAILED_CODE 5
#define OROCOS_PORT_BINDING_FAILED_MSG "port binding failed"

#define OROCOS_BAD_PACKET_LENGTH_CODE 6
#define OROCOS_BAD_PACKET_LENGTH_MSG "bad packet length fom FRI"

#define OROCOS_SEND_DATAGRAM_FAILED_CODE 7
#define OROCOS_SEND_DATAGRAM_FAILED_MSG "bad packet length fom FRI"

#define OROCOS_PADDING_NOT_OK_CODE 8
#define OROCOS_PADDING_NOT_OK_MSG "padding on this platform not ok"

#define OROCOS_BYTE_ORDER_NOT_OK_CODE 9
#define OROCOS_BYTE_ORDER_NOT_OK_MSG "byte order and float representations are not OK on this platform"

#define OROCOS_NO_RAD_PROVIDED_CODE 10
#define OROCOS_NO_RAD_PROVIDED_MSG "ptp expects data in radians (define label)"

#define OROCOS_MAX_JOINT_LIMIT_CODE 11
#define OROCOS_MAX_JOINT_LIMIT_MSG "number of given joints is not correct"

#define IIS_ROBOT_DEP_MAX_JNT_VEL 0.1;
#define IIS_ROBOT_DEP_MAX_CART_VEL 1.0;

#define OROCOS_ROBOT_STOPPED_DANGER_CODE 12
#define OROCOS_ROBOT_STOPPED_DANGER_MSG "robot stopped due to possible collision"

#endif
