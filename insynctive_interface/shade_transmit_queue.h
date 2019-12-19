
#ifndef SHADE_TRANSMIT_QUEUE_H
#define         SHADE_TRANSMIT_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define WINDOW_SHADE_CMD_B2S_SET_SHADE_POS          0x01
#define WINDOW_SHADE_CMD_B2S_GET_SHADE_POS          0x81
#define WINDOW_SHADE_CMD_B2S_SET_VENETIAN_ANGLE     0x02
#define WINDOW_SHADE_CMD_B2S_GET_VENETIAN_ANGLE     0x82
#define WINDOW_SHADE_CMD_B2S_GET_BATTERY_LEVEL      0x83
#define WINDOW_SHADE_CMD_B2S_GET_LIMIT              0x84
#define WINDOW_SHADE_CMD_B2S_ACK                    0xFF

#define WINDOW_SHADE_CMD_S2B_REPORT_SHADE_POSITION  0x81
#define WINDOW_SHADE_CMD_S2B_REPORT_VENETIAN_ANGLE  0x82
#define WINDOW_SHADE_CMD_S2B_REPORT_BATTERY_LEVEL   0x83
#define WINDOW_SHADE_CMD_S2B_REPORT_LIMIT_POSITION  0x84
#define WINDOW_SHADE_CMD_S2B_LEARN_REQ              0x7E
#define WINDOW_SHADE_CMD_S2B_ACK                    0xFF


#define BRIDGE_TO_SHADE_PARAM_100_PERCENT      100
#define BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_LOWER 101
#define BRIDGE_TO_SHADE_PARAM_SOFT_LIMIT_UPPER 102
#define BRIDGE_TO_SHADE_PARAM_SAVED_POSITION   103
#define BRIDGE_TO_SHADE_PARAM_HALT_MOVEMENT    104

#define MI_CMD_VENITIAN_TILT_INCREASE   0x17
#define MI_CMD_VENITIAN_TILT_DECREASE   0x18

#define BRIDGE_TO_SHADE_COMMAND_RETRIES 12

typedef struct pending_shade_command_t
{
    uint32_t shade_serial;
    uint8_t shade_command, param, retries_left;

}pending_shade_command;


#define MAX_QUEUED_COMMANDS 30


void erase_all_of_same_commands_for_shade(uint32_t serial, uint8_t command);

void erase_remaining_sent_command_queued_for_shade( uint32_t serial );

void erase_remaining_sent_nonack_command_queued_for_shade( uint32_t serial );

bool enqueue_shade_command(uint32_t serial, uint8_t command, uint8_t parameter);

void getNext_shade_command(pending_shade_command* pending_command);

void shade_command_queue_init(void);

#endif  /* SHADE_TRANSMIT_QUEUE_H */
