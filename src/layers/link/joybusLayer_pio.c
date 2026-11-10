#include "joybusLayer.h"
#include "joybusProgram.h"

#include <zephyr/kernel.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/sio.h"
#include "hardware/timer.h"
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

/* the link pins: SC, SI, SO, SD (GBA cable), SD (GBC cable) */
#define LINK_PIN_FIRST 0u
#define LINK_PIN_COUNT 5u

static PIO g_pio = NULL;
static const uint g_sm = 0;   /* allocated by the GBA link layer at boot, as gbLinkLayer.c uses it */
static bool g_started = false;
static unsigned g_bit_us = 5;
static uint g_offset = 0;
static uint g_wrap = 0;
static unsigned g_tx_pin = 2, g_rx_pin = 1;
static bool g_invert_tx = false, g_invert_rx = false;
static uint32_t g_fail_raw = 0;
static bool g_push_pull = true;    /* what the loaded program does (two lines) */
static uint32_t g_last_end = 0;    /* when the last command ended (time_us_32) */
static uint8_t g_fail_stage = 0, g_fail_bits = 0;
static uint16_t g_instructions[JOYBUS_PROGRAM_MAX];
static struct pio_program g_program = { .instructions = g_instructions, .length = 0, .origin = -1 };

static void release_link_pins(void)
{
    for (unsigned pin = LINK_PIN_FIRST; pin < LINK_PIN_FIRST + LINK_PIN_COUNT; pin++) {
        gpio_init(pin);   /* also clears any output or input override */
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
}

/* the TX pin at rest: released (open drain), or driven to the line's idle level (push-pull) */
static void idle_tx(void)
{
    if (g_push_pull) {
        pio_sm_set_pins_with_mask(g_pio, g_sm, 1u << g_tx_pin, 1u << g_tx_pin);
        pio_sm_set_pindirs_with_mask(g_pio, g_sm, 1u << g_tx_pin, 1u << g_tx_pin);
    } else {
        pio_sm_set_pins_with_mask(g_pio, g_sm, 0, 1u << g_tx_pin);
        pio_sm_set_pindirs_with_mask(g_pio, g_sm, 0, 1u << g_tx_pin);
    }
}

/* the state machine stopped, emptied and back at the start, with the TX pin at rest */
static void reset_sm(void)
{
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_sm_clear_fifos(g_pio, g_sm);
    pio_sm_restart(g_pio, g_sm);
    idle_tx();
    pio_sm_exec(g_pio, g_sm, pio_encode_jmp(g_offset + JOYBUS_OFFSET_START));
    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void load_program(bool push_pull)
{
    unsigned wrap = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);
    g_program.length = (uint8_t)joybus_build_program(g_instructions, g_bit_us, push_pull, &wrap);
    g_wrap = wrap;
    g_offset = pio_add_program(g_pio, &g_program);
    g_push_pull = push_pull;
}

static void configure(void)
{
    pio_sm_set_enabled(g_pio, g_sm, false);
    release_link_pins();
    const bool push_pull = g_tx_pin != g_rx_pin;
    if (push_pull != g_push_pull || !g_program.length) load_program(push_pull);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, g_offset, g_offset + g_wrap);
    sm_config_set_set_pins(&c, g_tx_pin, 1);
    sm_config_set_out_pins(&c, g_tx_pin, 1);
    sm_config_set_in_pins(&c, g_rx_pin);
    /* both shift left (first bit highest); pull ifempty uses the 32-bit threshold; replies are
     * pushed 32 bits at a time and the rest by the program */
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (float)(JOYBUS_CYCLES_PER_US * 1000000u));

    pio_gpio_init(g_pio, g_tx_pin);
    gpio_pull_up(g_tx_pin);
    if (push_pull && g_invert_tx) gpio_set_outover(g_tx_pin, GPIO_OVERRIDE_INVERT);
    if (push_pull && g_invert_rx) gpio_set_inover(g_rx_pin, GPIO_OVERRIDE_INVERT);
    idle_tx();

    pio_sm_init(g_pio, g_sm, g_offset, &c);
    pio_sm_set_enabled(g_pio, g_sm, true);
    g_last_end = time_us_32();   /* the lines have just changed: the first command waits the gap out */
}

bool joybus_start(unsigned bit_us)
{
    if (bit_us < 4 || bit_us > 7) return false;
    g_bit_us = bit_us;
    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));
    g_pio = pio_rpi_pico_get_pio(dev);
    g_program.length = 0;   /* built for these bits when the pins are chosen */
    g_started = true;
    configure();
    return true;
}

void joybus_stop(void)
{
    if (!g_started) return;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_sm_clear_fifos(g_pio, g_sm);
    pio_clear_instruction_memory(g_pio);
    release_link_pins();
    g_program.length = 0;
    g_started = false;
}

void joybus_set_pins(unsigned tx_pin, unsigned rx_pin, bool invert_tx, bool invert_rx)
{
    g_tx_pin = tx_pin;
    g_rx_pin = rx_pin;
    g_invert_tx = invert_tx && tx_pin != rx_pin;
    g_invert_rx = invert_rx && tx_pin != rx_pin;
    if (g_started) configure();
}

int joybus_transfer(const uint8_t* tx, unsigned tx_len, uint8_t* rx, unsigned rx_len, uint32_t slack_us)
{
    if (!g_started || tx_len < 1 || tx_len > 5 || rx_len < 1 || rx_len > 5) return -1;

    uint32_t words[3];
    const unsigned n = joybus_pack_command(tx, tx_len, rx_len, g_push_pull, words);
    while (time_us_32() - g_last_end < JOYBUS_GAP_US) {}
    /* the state machine waits at its first pull: the words all fit in the FIFO */
    pio_sm_clear_fifos(g_pio, g_sm);
    for (unsigned i = 0; i < n; i++) pio_sm_put(g_pio, g_sm, words[i]);

    /* the command and its stop bit, then the reply's bits (under 4 us each, 5 allowed) */
    const uint32_t budget = (tx_len * 8 + 1) * g_bit_us + rx_len * 8 * 5 + slack_us;
    const uint32_t start = time_us_32();
    uint32_t got[2];
    const unsigned want = joybus_reply_words(rx_len);
    for (unsigned i = 0; i < want; i++) {
        while (pio_sm_is_rx_fifo_empty(g_pio, g_sm)) {
            if (time_us_32() - start > budget) {
                /* no reply, or only part of one: where the program waits, and the bits it still wants (y) */
                pio_sm_set_enabled(g_pio, g_sm, false);
                const unsigned pc = pio_sm_get_pc(g_pio, g_sm);
                pio_sm_clear_fifos(g_pio, g_sm);
                pio_sm_exec(g_pio, g_sm, JB_PUSH_NOBLOCK);   /* the bits that came */
                g_fail_raw = pio_sm_is_rx_fifo_empty(g_pio, g_sm) ? 0 : pio_sm_get(g_pio, g_sm);
                pio_sm_exec(g_pio, g_sm, JB_MOV_ISR_Y);
                pio_sm_exec(g_pio, g_sm, JB_PUSH_NOBLOCK);
                const uint32_t left = pio_sm_is_rx_fifo_empty(g_pio, g_sm) ? rx_len * 8 - 1 : pio_sm_get(g_pio, g_sm);
                g_fail_stage = (uint8_t)(pc >= g_offset ? pc - g_offset : 0xFF);
                g_fail_bits = g_fail_stage < JOYBUS_AT_BIT_START || left > rx_len * 8 - 1 ? 0 : (uint8_t)(rx_len * 8 - 1 - left);
                reset_sm();
                g_last_end = time_us_32();
                return -1;
            }
        }
        got[i] = pio_sm_get(g_pio, g_sm);
    }
    /* the GBA's stop bit follows the last bit read */
    g_last_end = time_us_32() + 8;
    joybus_unpack_reply(got, rx_len, rx);
    return (int)rx_len;
}

void joybus_last_failure(uint8_t* stage, uint8_t* bits)
{
    *stage = g_fail_stage;
    *bits = g_fail_bits;
}

uint32_t joybus_last_failure_raw(void) { return g_fail_raw; }

void joybus_sample_pins(uint8_t* seen_high, uint8_t* seen_low)
{
    const uint32_t mask = ((1u << LINK_PIN_COUNT) - 1u) << LINK_PIN_FIRST;
    uint32_t high = 0, low = 0;
    const uint32_t start = time_us_32();
    while (time_us_32() - start < 2000) {
        const uint32_t in = sio_hw->gpio_in & mask;
        high |= in;
        low |= ~in & mask;
    }
    *seen_high = (uint8_t)(high >> LINK_PIN_FIRST);
    *seen_low = (uint8_t)(low >> LINK_PIN_FIRST);
}

uint8_t joybus_rise_time(unsigned pin)
{
    if (!g_started || pin < LINK_PIN_FIRST || pin >= LINK_PIN_FIRST + LINK_PIN_COUNT) return 0xFF;
    pio_sm_set_enabled(g_pio, g_sm, false);
    /* by hand: low for 20 us (open drain: output latch 0, direction out), then released */
    gpio_init(pin);
    gpio_pull_up(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
    busy_wait_us_32(20);
    const uint32_t start = time_us_32();
    gpio_set_dir(pin, GPIO_IN);
    uint32_t took = 0xFF;
    while (time_us_32() - start < 250) {
        if (gpio_get(pin)) { took = time_us_32() - start; break; }
    }
    configure();   /* the pins and the state machine back as they were */
    return (uint8_t)took;
}
