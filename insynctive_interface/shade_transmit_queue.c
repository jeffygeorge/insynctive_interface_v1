#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "shade_transmit_queue.h"
#include "ESW1032P-01.h"

uint8_t shade_queue_index;
pending_shade_command shade_tx_queue[MAX_QUEUED_COMMANDS];

static void clear_tx_queue_entry(uint8_t index)
{
    if ((index >= 0) && (index < MAX_QUEUED_COMMANDS)) {
        shade_tx_queue[index].shade_serial = 0;
        shade_tx_queue[index].shade_command = 0;
        shade_tx_queue[index].param = 0;
        shade_tx_queue[index].retries_left = 0;
    }
}

static int find_first_empty_entry(void)
{
    int index = 0;

    for (index = 0; index < MAX_QUEUED_COMMANDS; ++index) {
        if (shade_tx_queue[index].shade_serial == 0)
            return index;
    }

    return index;
}


void erase_all_of_same_commands_for_shade(uint32_t serial, uint8_t command)
{
    for (int i = 0; i < MAX_QUEUED_COMMANDS; ++i) {
        // Erase duplicate commands queued for this shade

        if (shade_tx_queue[i].shade_serial == serial)
            if (shade_tx_queue[i].shade_command == command)
                clear_tx_queue_entry(i);
    }
}

void erase_first_found_sent_command_for_shade(uint32_t serial)
{
    for (int i = 0; i < MAX_QUEUED_COMMANDS; ++i ) {
        // Erase all of the same commands queued for this shade

        if (shade_tx_queue[i].shade_serial == serial)
            clear_tx_queue_entry(i);
    }

}

void erase_remaining_sent_command_queued_for_shade( uint32_t serial ){
    for (int i = 0; i < MAX_QUEUED_COMMANDS; ++i) {
        // Erase all of the same commands queued for this shade

        if ((shade_tx_queue[i].shade_serial == serial) &&
            (shade_tx_queue[i].retries_left > 0))
            clear_tx_queue_entry(i);
    }
}


void erase_remaining_sent_nonack_command_queued_for_shade(uint32_t serial)
{
    /* Erase all of the same commands queued for this shade */

    for (int i = 0; i < MAX_QUEUED_COMMANDS; ++i)

        if ((shade_tx_queue[i].shade_serial == serial) &&
            (shade_tx_queue[i].shade_command != WINDOW_SHADE_CMD_B2S_ACK) &&
            (shade_tx_queue[i].retries_left > 0))

            clear_tx_queue_entry(i);
}

bool enqueue_shade_command(uint32_t serial, uint8_t command, uint8_t parameter)
{
    save_command_and_param(serial, command, parameter);
    erase_all_of_same_commands_for_shade(serial, command);

    int index = find_first_empty_entry();

    if (index < MAX_QUEUED_COMMANDS) {
        // If we found an empty spot

        shade_tx_queue[index].shade_serial = serial;
        shade_tx_queue[index].shade_command = command;
        shade_tx_queue[index].param = parameter;
        shade_tx_queue[index].retries_left = BRIDGE_TO_SHADE_COMMAND_RETRIES;

        return true;
    }

    return false;
}

void getNext_shade_command(pending_shade_command* pending_command)
{
    uint8_t i = shade_queue_index;

    uint8_t slots_checked;

    for (slots_checked = 0; slots_checked < MAX_QUEUED_COMMANDS; ++slots_checked) {
        // Find the next non-empty command starting at the shade_queue_index

        if (++i >= MAX_QUEUED_COMMANDS)
            i = 0;

        if (shade_tx_queue[i].shade_serial != 0) {
            shade_queue_index = i;

            pending_command->shade_serial  = shade_tx_queue[i].shade_serial;
            pending_command->shade_command = shade_tx_queue[i].shade_command;
            pending_command->param         = shade_tx_queue[i].param;
            pending_command->retries_left  = shade_tx_queue[i].retries_left;

            if (shade_tx_queue[i].retries_left != 0)
                shade_tx_queue[i].retries_left--;

            if (shade_tx_queue[i].retries_left == 0)
                clear_tx_queue_entry(i);

            return;
        }
    }
}

void shade_command_queue_init(void)
{
    uint8_t i = 0;
    for (; i< MAX_QUEUED_COMMANDS; ++i)
    {
        shade_tx_queue[i].shade_serial = 0;
        shade_tx_queue[i].shade_command = 0;
        shade_tx_queue[i].param = 0;
        shade_tx_queue[i].retries_left = 0;
    }

    shade_queue_index = 0;
}
