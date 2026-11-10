#pragma once

#include <zephyr/kernel.h>
#include <cstdint>
#include <span>

#include "moduleInterface.hpp"
#include "joybusWiring.hpp"

/* GameCube JOY Bus mode (SetMode 0x08): the adapter is a GameCube's controller port with a GBA on
 * it. The host (a GameCube emulator or port) sends each JOY Bus command it would put on the cable;
 * the adapter sends it to the GBA, which must be in JOY Bus mode (as the Pokémon games are at
 * their copyright screen, and the programs a GameCube sends a GBA), and returns the reply.
 *
 * Commands (command channel):
 *   0x50 Transfer     [0x50, seq, txLen, rxLen, tx bytes...]   txLen, rxLen 1..5
 *        reply (data) [0x50, seq, result, wiring, rxLen, rx bytes...]
 *                     result 0: replied; 1: no reply; 2: bad request
 *        rxLen | 0x80: then a status command (0x00) at once, its reply after the first:
 *                     [..., rx bytes..., statusResult, 3 status bytes]. A GameCube asks for the status after
 *                     each write it sends a GBA, to see when the GBA has taken the word; this saves the host a
 *                     round trip for each (a status changes nothing on the GBA).
 *   0x51 SetWiring    [0x51, wiring]   0xFF: find it (the default); else one of joybusWiring.hpp's
 *        reply (data) [0x53, ...] as GetState
 *   0x52 SetBitLength [0x52, us]       the GameCube's bits: 4 (the default) or 5 us
 *        reply (data) [0x53, ...] as GetState
 *   0x53 GetState     [0x53]
 *        reply (data) [0x53, protocol 2, wiring (0xFF none yet), fixed, bitUs, wiringCount]
 *   0x54 Diagnose     [0x54, flags]   flags bit 0: also measure how fast the lines the wirings send on rise
 *                                     (each is pulled low for 20 us: only while the GBA isn't answering)
 *        reply (data) [0x54, 1, wiringCount, wiring in use, found | fixed << 1, bitUs,
 *                      pins seen high, pins seen low (bit n: GPIO n; 0 SC, 1 SI, 2 SO, 3 SD GBA cable, 4 SD GBC cable),
 *                      rise time of GPIO 0..4 in us (0xFE not measured, 0xFF held low),
 *                      then for each wiring: tries (16 bits, low byte first), replies (16 bits),
 *                      where the last unanswered one stopped (joybusProgram.h's JOYBUS_AT_*), the most
 *                      reply bits an unanswered one got]
 *
 * Which pins carry the bus depends on the cable and isn't documented for sure, so by default the
 * adapter finds them (joybusWiring.hpp): when a status (0x00) or reset (0xFF) command gets no
 * answer, it tries the same command on each wiring that only ever pulls a GBA input or a shared
 * line low, and keeps the first that a GBA answers (type 0x0004). Only those two commands are
 * retried, as they change nothing on the GBA; a write is never sent twice. */
class JoybusModule : public IModule
{
public:
    static constexpr uint8_t protocolVersion = 2;

    enum class Command : uint8_t
    {
        Transfer = 0x50,
        SetWiring = 0x51,
        SetBitLength = 0x52,
        GetState = 0x53,
        Diagnose = 0x54,
    };

    JoybusModule();

    void execute();
    void cancel() override;
    bool canHandle(uint8_t command) override { return (command & 0xF0) == 0x50; }
    void receiveCommand(std::span<const uint8_t> command) override;

private:
    struct Request
    {
        uint8_t length;
        uint8_t data[15];
    };

    void handle(const Request& request);
    void transfer(const Request& request);
    void usePins(uint8_t wiring);
    void sendState();
    void sendDiagnosis(bool riseTimes);
    int sendOn(uint8_t wiring, const uint8_t* tx, unsigned txLen, uint8_t* rx, unsigned rxLen);

    struct WiringStats
    {
        uint16_t tries = 0;
        uint16_t replies = 0;
        uint8_t lastStage = 0;
        uint8_t mostBits = 0;
    };
    WiringStats m_stats[joybus::wiringCount];

    char m_queueBuffer[8 * sizeof(Request)] __aligned(4);
    struct k_msgq m_queue;
    volatile bool m_cancel = false;

    joybus::WiringSearch m_search;
    uint8_t m_pins = 0xFF;         // the wiring the pins are set up for
    uint8_t m_bitUs = 5;           // a GameCube's bits are 4.94 us
};
