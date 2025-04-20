#include "dmu11.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "commands.h"
#include "terminal.h"
#include "hal.h"
#include "hw.h"

#define BAUDRATE					460800
#ifndef UART_NUMBER
#ifdef HW_UART_3_DEV
#define UART_NUMBER 3
#else
#ifdef HW_UART_P_DEV
#define UART_NUMBER 2
#else
#define UART_NUMBER 1
#endif
#endif
#endif

// Threads
static THD_FUNCTION(dmu11_thread, arg);

// Variables
static volatile bool thread_is_running = false;
static volatile bool uart_is_running[UART_NUMBER] = {false};
static SerialConfig uart_cfg[UART_NUMBER] = {{
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
}};

// Different for Rx and Tx because it is possible for hardware to use different UART driver
// for Rx and Tx, feature not a bug XD
static SerialDriver *serialPortDriverTx[UART_NUMBER];
static SerialDriver *serialPortDriverRx[UART_NUMBER];
static stm32_gpio_t * TxGpioPort[UART_NUMBER];
static stm32_gpio_t * RxGpioPort[UART_NUMBER];
static uint8_t TxGpioPin[UART_NUMBER], RxGpioPin[UART_NUMBER], gpioAF[UART_NUMBER];
static bool pins_enabled[UART_NUMBER];


/**
 * @brief DMUMessageBuffer struct to hold the message buffer and state.
 */
typedef struct {
    uint8_t buffer[DMU11_MSG_SIZE];
    int size;
} DMU11_Message_Buffer;

static DMU11State* m_terminal_state = NULL;  // Static pointer to hold state for terminal
static DMU11_Message_Buffer dmu11_message_buffer = {0};

/**
 * @brief parse_dmu11_buffer_data_into_state parses a buffer of data from a DMU11 message into a DMU11State struct.
 * @param msgBuffer
 * @param msgBufferSize
 * @param dmu11State
 * @return true if parsing was successful, false otherwise
 */
bool parse_dmu11_buffer_data_into_state(const uint8_t* msgBuffer, size_t msgBufferSize, DMU11State* dmu11State) {

    if (!msgBuffer || !dmu11State || msgBufferSize < DMU11_MSG_SIZE) {
        commands_printf("Buffer too small to contain complete message: %zu bytes\n", msgBufferSize);
        return false;
    }

    // Parse header (0x55AA)
    dmu11State->header = (uint16_t)(msgBuffer[0] << 8) | msgBuffer[1];
    dmu11State->messageCount = (uint16_t)(msgBuffer[2] << 8) | msgBuffer[3];

    size_t offset = 4;

    // Helper function to convert big-endian bytes to float
    union {
        uint32_t i;
        float f;
    } converter;

    // Parse float values
    #define PARSE_FLOAT(dest) do { \
        converter.i = ((uint32_t)msgBuffer[offset] << 24) | \
                        ((uint32_t)msgBuffer[offset + 1] << 16) | \
                        ((uint32_t)msgBuffer[offset + 2] << 8) | \
                        msgBuffer[offset + 3]; \
        dest = converter.f; \
        offset += 4; \
    } while(0)

    PARSE_FLOAT(dmu11State->axisXRate);
    PARSE_FLOAT(dmu11State->axisXAcceleration);
    PARSE_FLOAT(dmu11State->axisYRate);
    PARSE_FLOAT(dmu11State->axisYAcceleration);
    PARSE_FLOAT(dmu11State->axisZRate);
    PARSE_FLOAT(dmu11State->axisZAcceleration);

    offset += 4; // Skip reserved words

    PARSE_FLOAT(dmu11State->avgTemperature);
    PARSE_FLOAT(dmu11State->axisXDeltaTheta);
    PARSE_FLOAT(dmu11State->axisXDeltaVel);
    PARSE_FLOAT(dmu11State->axisYDeltaTheta);
    PARSE_FLOAT(dmu11State->axisYDeltaVel);
    PARSE_FLOAT(dmu11State->axisZDeltaTheta);
    PARSE_FLOAT(dmu11State->axisZDeltaVel);

    #undef PARSE_FLOAT

    // Parse status flags
    dmu11State->startupFlags = (uint16_t)(msgBuffer[offset] << 8) | msgBuffer[offset + 1];
    offset += 2;
    dmu11State->operationFlags = (uint16_t)(msgBuffer[offset] << 8) | msgBuffer[offset + 1];
    offset += 2;

    // Parse checksum
    dmu11State->checksum = (uint16_t)(msgBuffer[66] << 8) | msgBuffer[67];


    float accel[3] = {dmu11State->axisXAcceleration, dmu11State->axisYAcceleration, dmu11State->axisZAcceleration};
    float gyro[3] = {dmu11State->axisXRate, dmu11State->axisYRate, dmu11State->axisZRate};
    float mag[3] = {0,0,0};
    dmu11State->read_callback(accel, gyro, mag);

    return true;
}

/**
 * @brief validates the checksum of a message from DMU11 with the computed checksum.
 * @param buffer
 * @return true if checksum is valid, false otherwise
 */
bool validate_dmu11_checksum(const uint8_t* buffer) {
    uint16_t sum = 0;

    // Process words 0-32
    for (size_t i = 0; i < 33; i++) {
        size_t offset = (i * 2);
        uint16_t word = (uint16_t)(buffer[offset] << 8) | buffer[offset + 1];
        uint16_t oldSum = sum;
        sum += word;
    }

    // Take 2's complement
    uint16_t checksum = ~sum + 1;

    // Get provided checksum from message
    uint16_t providedSum = (uint16_t)(buffer[66] << 8) | buffer[67];

    return checksum == providedSum;
}

void parse_dmu11_byte_into_state(uint8_t byte, DMU11State* dmu11State) {
    // Look for DMU11 message header bytes (0x55 0xAA)
    if (!dmu11_message_buffer.size) {
        if (byte == DMU11_MSG_HEADER_1) {
            dmu11_message_buffer.buffer[0] = byte;
            dmu11_message_buffer.size = 1;
        }
        else if (dmu11_message_buffer.size == 1 && byte == DMU11_MSG_HEADER_2) {
            dmu11_message_buffer.buffer[1] = byte;
            dmu11_message_buffer.size = 2;
        }
        else {
            dmu11_message_buffer.size = 0;
        }
        return;
    }

    // add the byte to the message buffer
    dmu11_message_buffer.buffer[dmu11_message_buffer.size++] = byte;


    // see if we have a complete message in the buffer
    if (dmu11_message_buffer.size == DMU11_MSG_SIZE) {
        // Validate checksum
        if (validate_dmu11_checksum(dmu11_message_buffer.buffer)) {
            if (parse_dmu11_buffer_data_into_state(dmu11_message_buffer.buffer, DMU11_MSG_SIZE, dmu11State)) {
                // Use the data - maybe callback or queue
                // print_dmu11_data(dmu11State);
                m_terminal_state = dmu11State;
            }
        }
        
        // Reset buffer for next message
        dmu11_message_buffer.size = 0;
    }
}

void print_dmu11_data(const DMU11State* data) {
    if (!data) {
        commands_printf("No IMU data to print\n");
        return;
    };

    commands_printf("=== DMU11 IMU Data ===\n");
    commands_printf("Header: 0x%04X\n", data->header);
    commands_printf("Message Count: %u\n", data->messageCount);
    commands_printf("Axis X Rate: %.6f °/s\n", data->axisXRate);
    commands_printf("Axis X Acceleration: %.6f g\n", data->axisXAcceleration);
    commands_printf("Axis Y Rate: %.6f °/s\n", data->axisYRate);
    commands_printf("Axis Y Acceleration: %.6f g\n", data->axisYAcceleration);
    commands_printf("Axis Z Rate: %.6f °/s\n", data->axisZRate);
    commands_printf("Axis Z Acceleration: %.6f g\n", data->axisZAcceleration);
    commands_printf("Average Temperature: %.6f °C\n", data->avgTemperature);
    commands_printf("Axis X Delta Theta: %.6f °\n", data->axisXDeltaTheta);
    commands_printf("Axis X Delta Vel: %.6f m/s\n", data->axisXDeltaVel);
    commands_printf("Axis Y Delta Theta: %.6f °\n", data->axisYDeltaTheta);
    commands_printf("Axis Y Delta Vel: %.6f m/s\n", data->axisYDeltaVel);
    commands_printf("Axis Z Delta Theta: %.6f °\n", data->axisZDeltaTheta);
    commands_printf("Axis Z Delta Vel: %.6f m/s\n", data->axisZDeltaVel);
    commands_printf("System Startup Flags: 0x%04X\n", data->startupFlags);
    commands_printf("System Operation Flags: 0x%04X\n", data->operationFlags);
    commands_printf("Checksum: 0x%04X\n", data->checksum);
    commands_printf("=====================\n");
}

static void terminal_print_command(int argc, const char **argv) {
    (void)argc;(void)argv;

    if (m_terminal_state != NULL) {
        print_dmu11_data(m_terminal_state);
    }
}

/** UART INITIALIZATION STUFF */
void dmu_uartcomm_initialize(void) {
	serialPortDriverTx[0] = &HW_UART_DEV;
	serialPortDriverRx[0] = &HW_UART_DEV;
	uart_cfg[0].speed =  BAUDRATE;
	RxGpioPort[0] = HW_UART_RX_PORT; RxGpioPin[0] = HW_UART_RX_PIN;
	TxGpioPort[0] = HW_UART_TX_PORT; TxGpioPin[0] = HW_UART_TX_PIN;
	gpioAF[0] = HW_UART_GPIO_AF;
}

void dmu_uartcomm_start(UART_PORT port_number) {
	if(port_number >= UART_NUMBER){
		return;
	}

	sdStart(serialPortDriverRx[port_number], &uart_cfg[port_number]);
	sdStart(serialPortDriverTx[port_number], &uart_cfg[port_number]);
	uart_is_running[port_number] = true;

	palSetPadMode(TxGpioPort[port_number], TxGpioPin[port_number], PAL_MODE_ALTERNATE(gpioAF[port_number]) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(RxGpioPort[port_number], RxGpioPin[port_number], PAL_MODE_ALTERNATE(gpioAF[port_number]) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	pins_enabled[port_number] = true;
}

void dmu_uartcomm_stop(UART_PORT port_number) {
	if(port_number >= UART_NUMBER) {
		return;
	}

	if (uart_is_running[port_number]) {
		sdStop(serialPortDriverRx[port_number]);
		sdStop(serialPortDriverTx[port_number]);
		palSetPadMode(TxGpioPort[port_number], TxGpioPin[port_number], PAL_MODE_INPUT_PULLUP);
		palSetPadMode(RxGpioPort[port_number], RxGpioPin[port_number], PAL_MODE_INPUT_PULLUP);
		uart_is_running[port_number] = false;
	}
	// Notice that the processing thread is kept running in case this call is made from it.
}

void dmu11_init(DMU11State *state, stkalign_t *work_area, size_t work_area_size) {
    dmu_uartcomm_initialize();
    dmu_uartcomm_start(state->uart_port);

    chThdSleep(1);
    chThdCreateStatic(work_area, work_area_size, NORMALPRIO, dmu11_thread, state);

    // register terminal command for the first instance of this driver.
	if (m_terminal_state == 0) {
        terminal_register_command_callback(
            "dmu11",
            "Print DMU11 data",
            0,    
            terminal_print_command);
	}
}

void dmu11_set_read_callback(DMU11State *state, void(*func)(float *accel, float *gyro, float *mag)){
    state->read_callback = func;
}


static THD_FUNCTION(dmu11_thread, arg) {
	DMU11State *state = (DMU11State*)arg;


	chRegSetThreadName("dmu11_thread");

    event_listener_t el[UART_NUMBER];
	for(int port_number = 0; port_number < UART_NUMBER; port_number++) {
		chEvtRegisterMaskWithFlags(&(*serialPortDriverRx[port_number]).event, &el[port_number], EVENT_MASK(0), CHN_INPUT_AVAILABLE);
	}

	for(;;) {
		chEvtWaitAnyTimeout(ALL_EVENTS, ST2MS(10));
		
		bool rx = true;
        
		while (rx) {
			rx = false;
			if (uart_is_running[state->uart_port]  || true) {
				msg_t res = sdGetTimeout(serialPortDriverRx[0], TIME_IMMEDIATE);
				if (res != MSG_TIMEOUT) {
					parse_dmu11_byte_into_state((uint8_t)res, state);
					rx = true;
				}
			}
		}
	}

    
}