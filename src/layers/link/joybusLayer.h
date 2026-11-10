/* The GameCube's side of a JOY Bus, for a GBA in JOY Bus mode (module/joybus.cpp): the adapter
 * sends a command as the GameCube's controller port does and reads the GBA's reply.
 *
 * It takes pio0's state machine 0, as the GB and GBA link layers do; only one of them is active at
 * a time (link_disable() and gb_link_deinit() first). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Take the PIO and load the program, with the GameCube's bits `bit_us` long (4 or 5). The lines
 * are left released until joybus_set_pins. */
bool joybus_start(unsigned bit_us);
void joybus_stop(void);

/* Which link pins to use (GPIO numbers): the adapter pulls tx_pin low to send and reads rx_pin;
 * the same pin for a GBA that talks on the line it listens on. Every other link pin is left
 * released, pulled up. */
void joybus_set_pins(unsigned tx_pin, unsigned rx_pin);

/* One command (tx_len bytes, 1..5) and its reply (rx_len bytes, 1..5). Returns rx_len, or -1
 * when the reply didn't all come within the time a reply takes plus `slack_us`. */
int joybus_transfer(const uint8_t* tx, unsigned tx_len, uint8_t* rx, unsigned rx_len, uint32_t slack_us);

#ifdef __cplusplus
}
#endif
