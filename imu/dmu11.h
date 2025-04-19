#ifndef IMU_DMU11_H_
#define IMU_DMU11_H_

#define DMU11_MSG_SIZE 68
#define DMU11_MSG_HEADER_1 0x55
#define DMU11_MSG_HEADER_2 0xAA

#include "ch.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum {
	UART_PORT_COMM_HEADER = 0,
	UART_PORT_BUILTIN,
	UART_PORT_EXTRA_HEADER
} UART_PORT;


/**
 * @brief DMU11State
 */
typedef struct {
    uint16_t header;
    uint16_t messageCount;
    float axisXRate;
    float axisXAcceleration;
    float axisYRate;
    float axisYAcceleration;
    float axisZRate;
    float axisZAcceleration;
    float avgTemperature;
    float axisXDeltaTheta;
    float axisXDeltaVel;
    float axisYDeltaTheta;
    float axisYDeltaVel;
    float axisZDeltaTheta;
    float axisZDeltaVel;
    uint16_t startupFlags;
    uint16_t operationFlags;
    uint16_t checksum;

    // read callback
    void(*read_callback)(float *accel, float *gyro, float *mag);
    UART_PORT uart_port;
} DMU11State;


void dmu11_set_read_callback(DMU11State *state, void(*func)(float *accel, float *gyro, float *mag));
void dmu11_init(DMU11State *state, stkalign_t *thread_work_area, size_t thread_work_area_size);


#endif /* IMU_DMU11_H_ */