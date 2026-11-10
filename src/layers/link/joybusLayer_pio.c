#include "joybusLayer.h"
#include "joybusProgram.h"

#include <zephyr/kernel.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

/* the link pins: SC, SI, SO, SD (GBA cable), SD (GBC cable) */
#define LINK_PIN_FIRST 0u
#define LINK_PIN_COUNT 5u

static PIO g_pio = NULL;
static const uint g_sm = 0;   /* allocated by the GBA link layer at boot, as gbLinkLayer.c uses it */
static bool g_started = false;
static unsigned g_bit_us = 4;
static uint g_offset = 0;
static uint g_wrap = 0;
static unsigned g_tx_pin = 2, g_rx_pin = 1;
static uint16_t g_instructions[JOYBUS_PROGRAM_MAX];
static struct pio_program g_program = { .instructions = g_instructions, .length = 0, .origin = -1 };

static void release_link_pins(void)
{
    for (unsigned pin = LINK_PIN_FIRST; pin < LINK_PIN_FIRST + LINK_PIN_COUNT; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }
}

/* the state machine stopped, emptied and back at the start, with the TX pin let go */
static void reset_sm(void)
{
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_sm_clear_fifos(g_pio, g_sm);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_set_pindirs_with_mask(g_pio, g_sm, 0, 1u << g_tx_pin);
    pio_sm_exec(g_pio, g_sm, pio_encode_jmp(g_offset + JOYBUS_OFFSET_START));
    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void configure(void)
{
    pio_sm_set_enabled(g_pio, g_sm, false);
    release_link_pins();

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

    /* open drain: the TX pin's output latch is 0 and the program only switches its direction */
    pio_gpio_init(g_pio, g_tx_pin);
    gpio_pull_up(g_tx_pin);
    pio_sm_set_pins_with_mask(g_pio, g_sm, 0, 1u << g_tx_pin);
    pio_sm_set_pindirs_with_mask(g_pio, g_sm, 0, 1u << g_tx_pin);

    pio_sm_init(g_pio, g_sm, g_offset, &c);
    pio_sm_set_enabled(g_pio, g_sm, true);
}

bool joybus_start(unsigned bit_us)
{
    unsigned wrap = 0;
    const unsigned n = joybus_build_program(g_instructions, bit_us, &wrap);
    if (!n) return false;
    g_program.length = (uint8_t)n;
    g_wrap = wrap;
    g_bit_us = bit_us;

    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));
    g_pio = pio_rpi_pico_get_pio(dev);

    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);
    g_offset = pio_add_program(g_pio, &g_program);
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
    g_started = false;
}

void joybus_set_pins(unsigned tx_pin, unsigned rx_pin)
{
    g_tx_pin = tx_pin;
    g_rx_pin = rx_pin;
    if (g_started) configure();
}

int joybus_transfer(const uint8_t* tx, unsigned tx_len, uint8_t* rx, unsigned rx_len, uint32_t slack_us)
{
    if (!g_started || tx_len < 1 || tx_len > 5 || rx_len < 1 || rx_len > 5) return -1;

    uint32_t words[3];
    const unsigned n = joybus_pack_command(tx, tx_len, rx_len, words);
    /* the state machine waits at its first pull: the words all fit in the FIFO */
    pio_sm_clear_fifos(g_pio, g_sm);
    for (unsigned i = 0; i < n; i++) pio_sm_put(g_pio, g_sm, words[i]);

    /* the command and its stop bit, then the reply's bits (4 us, a little longer allowed) */
    const uint32_t budget = (tx_len * 8 + 1) * g_bit_us + rx_len * 8 * 5 + slack_us;
    const uint32_t start = time_us_32();
    uint32_t got[2];
    const unsigned want = joybus_reply_words(rx_len);
    for (unsigned i = 0; i < want; i++) {
        while (pio_sm_is_rx_fifo_empty(g_pio, g_sm)) {
            if (time_us_32() - start > budget) {
                reset_sm();   /* no reply, or only part of one: the program is still waiting for bits */
                return -1;
            }
        }
        got[i] = pio_sm_get(g_pio, g_sm);
    }
    joybus_unpack_reply(got, rx_len, rx);
    return (int)rx_len;
}
