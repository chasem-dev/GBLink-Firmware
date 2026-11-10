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

/* Take the PIO, with the GameCube's bits `bit_us` long (4 to 7). The lines are left released
 * until joybus_set_pins. */
bool joybus_start(unsigned bit_us);
void joybus_stop(void);

/* Which link pins to use (GPIO numbers): the adapter sends on tx_pin and reads rx_pin; the same
 * pin for a GBA that talks on the line it listens on (then, and only then, the line is open
 * drain: pulled low or released, never driven high). `invert_tx`, `invert_rx`: that line the
 * other way up, resting low (two lines only). Every other link pin is left released, pulled up. */
void joybus_set_pins(unsigned tx_pin, unsigned rx_pin, bool invert_tx, bool invert_rx);

/* One command (tx_len bytes, 1..5) and its reply (rx_len bytes, 1..5). Returns rx_len, or -1
 * when the reply didn't all come within the time a reply takes plus `slack_us`. Commands are kept
 * JOYBUS_GAP_US apart: a GBA misses one that follows another sooner (64 us measured). */
#define JOYBUS_GAP_US 150u
int joybus_transfer(const uint8_t* tx, unsigned tx_len, uint8_t* rx, unsigned rx_len, uint32_t slack_us);

/* Why the last joybus_transfer returned -1: where the program was waiting (JOYBUS_AT_* in
 * joybusProgram.h), and how many of the reply's bits had come. */
void joybus_last_failure(uint8_t* stage, uint8_t* bits);
/* The bits that had come (the first received highest of the `bits` low ones). */
uint32_t joybus_last_failure_raw(void);

/* The link pins' levels over about 2 ms (bit n: GPIO n): which were seen high, and which low. */
void joybus_sample_pins(uint8_t* seen_high, uint8_t* seen_low);

/* How long `pin` takes to rise once released after being pulled low for 20 us, in microseconds
 * (0: under one; 0xFF: it didn't within 250, so something holds it low). The bus must be idle. */
uint8_t joybus_rise_time(unsigned pin);

#ifdef __cplusplus
}
#endif
