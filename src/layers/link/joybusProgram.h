/* The PIO program for the GameCube's side of a JOY Bus (joybusLayer_pio.c), built at run time so
 * the length of the bits it sends can be set. Plain C with no SDK headers, so the host tests
 * (tests/host/joybusPioTest.py) build and run the same program in a PIO emulator.
 *
 * The line is open drain: the pin's output latch is held at 0 and the program switches its
 * direction, so pindir 1 pulls the line low and pindir 0 lets the pull-up take it high. Nothing
 * here ever drives a line high, so a pin that turns out to be one of the GBA's outputs is never
 * fought.
 *
 * A JOY Bus bit starts with the line low: low for a quarter and high for three quarters is a 1,
 * low for three quarters and high for a quarter is a 0. The GameCube ends a command with a stop
 * bit (low for a quarter). The GBA then replies the same way, with bits 4 us long.
 *
 * Clock: JOYBUS_CYCLES_PER_US PIO cycles a microsecond.
 *
 * In (TX FIFO), per transaction:
 *   word 0  (tx bits - 1) << 24 | (rx bits - 1) << 16
 *   word 1  the command's first 32 bits, INVERTED (a 1 bit releases the line), first bit at bit 31
 *   word 2  bits 32..39 the same way, only when there are more than 32
 * Out (RX FIFO): the reply's bits, 32 a word, the first bit received highest; the last word holds
 * what is left in its low bits (autopush at 32, then a push).
 *
 * Pins: SET and OUT drive the TX pin (set_base = out_base, 1 pin); WAIT and IN read the RX pin
 * (in_base). They are the same pin when the GBA talks on the line it listens on. */
#ifndef JOYBUS_PROGRAM_H
#define JOYBUS_PROGRAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JOYBUS_CYCLES_PER_US 4u
#define JOYBUS_PROGRAM_MAX 18u

/* After the falling edge that starts a reply bit, when to sample it (cycles): about 2.25 us,
 * between a 1's 1 us low and a 0's 3 us low. */
#define JOYBUS_SAMPLE_DELAY 8u

/* instruction encodings (RP2040 datasheet, 3.4), with no side-set */
#define JB_DELAY(d) ((uint16_t)(((d) & 0x1fu) << 8))
#define JB_JMP(cond, addr) ((uint16_t)(0x0000u | ((cond) << 5) | ((addr) & 0x1fu)))
#define JB_JMP_ALWAYS 0u
#define JB_JMP_X_DEC 2u
#define JB_JMP_Y_DEC 4u
#define JB_WAIT_PIN(pol, index) ((uint16_t)(0x2000u | ((pol) << 7) | (1u << 5) | ((index) & 0x1fu)))
#define JB_IN_PINS(n) ((uint16_t)(0x4000u | ((n) & 0x1fu)))
#define JB_OUT(dest, n) ((uint16_t)(0x6000u | ((dest) << 5) | ((n) & 0x1fu)))
#define JB_OUT_X 1u
#define JB_OUT_Y 2u
#define JB_OUT_PINDIRS 4u
#define JB_PUSH(iffull, block) ((uint16_t)(0x8000u | ((iffull) << 6) | ((block) << 5)))
#define JB_PULL(ifempty, block) ((uint16_t)(0x8080u | ((ifempty) << 6) | ((block) << 5)))
#define JB_SET_PINDIRS(v) ((uint16_t)(0xe000u | (4u << 5) | ((v) & 0x1fu)))

/* Where a transaction starts (the wrap goes back here), from the program's first instruction. */
#define JOYBUS_OFFSET_START 0u

/* Build the program into out (JOYBUS_PROGRAM_MAX words); `bit_us` is the length of the bits the
 * GameCube sends (4 or 5; the GBA's replies are always read as 4 us bits). Returns the length, or
 * 0 for a length it can't make. *wrap gets the last instruction (it wraps to the start). */
static inline unsigned joybus_build_program(uint16_t* out, unsigned bit_us, unsigned* wrap)
{
    if (bit_us < 4 || bit_us > 7) return 0;
    const unsigned q = bit_us;   /* a quarter of a bit, in cycles: 4 cycles a microsecond */
    unsigned n = 0;
    out[n++] = JB_PULL(0, 1);                                   /*  0: the transaction's lengths */
    out[n++] = JB_OUT(JB_OUT_X, 8);                             /*  1: x = tx bits - 1 */
    out[n++] = JB_OUT(JB_OUT_Y, 8);                             /*  2: y = rx bits - 1 */
    out[n++] = JB_PULL(0, 1);                                   /*  3: the first 32 bits */
    const unsigned tx_bit = n;
    out[n++] = JB_SET_PINDIRS(1) | JB_DELAY(q - 1);             /*  4: low, a quarter */
    out[n++] = JB_OUT(JB_OUT_PINDIRS, 1) | JB_DELAY(2 * q - 2); /*  5: the bit, half a bit... */
    out[n++] = JB_PULL(1, 0);                                   /*  6: ...(the next 32 bits after 32) */
    out[n++] = JB_SET_PINDIRS(0) | JB_DELAY(q - 2);             /*  7: high, a quarter... */
    out[n++] = JB_JMP(JB_JMP_X_DEC, tx_bit);                    /*  8: ...and the next bit */
    out[n++] = JB_SET_PINDIRS(1) | JB_DELAY(q - 1);             /*  9: the stop bit: low, a quarter */
    out[n++] = JB_SET_PINDIRS(0);                               /* 10: let go */
    out[n++] = JB_WAIT_PIN(1, 0);                               /* 11: the line back up (a shared line rises slowly) */
    const unsigned rx_bit = n;
    out[n++] = JB_WAIT_PIN(0, 0) | JB_DELAY(JOYBUS_SAMPLE_DELAY); /* 12: a bit starts; wait... */
    out[n++] = JB_IN_PINS(1);                                   /* 13: ...and read it */
    out[n++] = JB_WAIT_PIN(1, 0);                               /* 14: its end */
    out[n++] = JB_JMP(JB_JMP_Y_DEC, rx_bit);                    /* 15 */
    out[n++] = JB_PUSH(0, 1);                                   /* 16: what is left of the reply */
    *wrap = n - 1;
    return n;
}

/* The TX FIFO words for a command of tx_len bytes (1..5) and a reply of rx_len bytes (1..5) into
 * words (3); returns how many to send. */
static inline unsigned joybus_pack_command(const uint8_t* tx, unsigned tx_len, unsigned rx_len, uint32_t* words)
{
    uint8_t b[8] = {0};
    for (unsigned i = 0; i < tx_len && i < 5; i++) b[i] = tx[i];
    words[0] = ((uint32_t)(tx_len * 8 - 1) << 24) | ((uint32_t)(rx_len * 8 - 1) << 16);
    words[1] = ~(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]);
    words[2] = ~((uint32_t)b[4] << 24);
    return tx_len > 4 ? 3 : 2;
}

/* How many RX FIFO words a reply of rx_len bytes comes in. */
static inline unsigned joybus_reply_words(unsigned rx_len) { return rx_len * 8 > 32 ? 2 : 1; }

/* The reply's bytes from its RX FIFO words. */
static inline void joybus_unpack_reply(const uint32_t* words, unsigned rx_len, uint8_t* rx)
{
    const unsigned bits = rx_len * 8;
    if (bits <= 32) {
        for (unsigned i = 0; i < rx_len; i++) rx[i] = (uint8_t)(words[0] >> (bits - 8 - 8 * i));
        return;
    }
    for (unsigned i = 0; i < 4; i++) rx[i] = (uint8_t)(words[0] >> (24 - 8 * i));
    const unsigned rest = bits - 32;
    for (unsigned i = 0; i < rx_len - 4; i++) rx[4 + i] = (uint8_t)(words[1] >> (rest - 8 - 8 * i));
}

#ifdef __cplusplus
}
#endif

#endif
