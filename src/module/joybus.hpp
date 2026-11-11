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
 *   0x52 SetBitLength [0x52, us]       the GameCube's bits, 4 to 16 us; 0: find it (the default): until a GBA
 *                                     answers, status and reset commands go out at 5 us (a console's speed)
 *                                     and 16 us (for an adapter whose lines are slow) in turn
 *        reply (data) [0x53, ...] as GetState
 *   0x53 GetState     [0x53]
 *        reply (data) [0x53, protocol 5, wiring (0xFF none yet), fixed, bitUs, wiringCount]
 *   0x54 Diagnose     [0x54, flags]   flags bit 0: also measure how fast the lines the wirings send on rise
 *                                     (each is pulled low for 20 us: only while the GBA isn't answering)
 *        reply (data) [0x54, 1, wiringCount, wiring in use, found | fixed << 1, bitUs,
 *                      pins seen high, pins seen low (bit n: GPIO n; 0 SC, 1 SI, 2 SO, 3 SD GBA cable, 4 SD GBC cable),
 *                      rise time of GPIO 0..4 in us (0xFE not measured, 0xFF held low),
 *                      then for each wiring: tries (16 bits, low byte first), replies (16 bits),
 *                      where the last unanswered one stopped (joybusProgram.h's JOYBUS_AT_*), the most
 *                      reply bits an unanswered one got]
 *   0x56 WriteMany    [0x56, seq, count (1..14), ack, count words of 4 bytes]   for sending a program to a
 *                     GBA faster than a USB round trip a word. For each word, as a GameCube game does: the
 *                     write, then a status command until the GBA has taken the word (its receive flag clear;
 *                     asked again for up to 100 ms). `ack`: the status bits the GBA flips with each word it
 *                     takes (0x10 for a multiboot; 0: none). With it, a write whose reply is lost is sent
 *                     again only if the status shows the word didn't arrive. A GBA that says nothing is asked
 *                     again for 50 ms before the batch is given up.
 *        reply (data) [0x56, seq, result, words written, the last status byte, the step that failed
 *                      (0 a write, 1 a status), writes sent again, statuses asked again]
 *                     result 0: all written and taken; 1: no reply; 4: a word was never taken
 *   0x57 Drop         [0x57, tenths of a second (1..50)]   every link line held low that long, as when the
 *                     adapter is unplugged: a GBA running a program a GameCube sent it restarts
 *        reply (data) [0x57, 1]
 *   0x55 LastReplies  [0x55]          what came back that wasn't a whole GBA reply, for seeing a link that half works
 *        reply (data) [0x55, 1, partial: wiring, bits, where it stopped, the bits (32, low byte first; the first
 *                      received highest of them), how many so far (16 bits);
 *                      odd (a whole reply to a status or reset that isn't a GBA's): wiring, 3 bytes, how many (16 bits)]
 *
 * Which pins carry the bus depends on the cable and isn't documented for sure, so by default the
 * adapter finds them (joybusWiring.hpp): when a status (0x00) or reset (0xFF) command gets no
 * answer, it tries the same command on each wiring that only ever pulls a GBA input or a shared
 * line low, and keeps the first that a GBA answers (type 0x0004). Only those two commands are
 * retried, as they change nothing on the GBA; a write is never sent twice. */
class JoybusModule : public IModule
{
public:
    static constexpr uint8_t protocolVersion = 5;

    enum class Command : uint8_t
    {
        Transfer = 0x50,
        SetWiring = 0x51,
        SetBitLength = 0x52,
        GetState = 0x53,
        Diagnose = 0x54,
        LastReplies = 0x55,
        WriteMany = 0x56,
        Drop = 0x57,
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
        uint8_t data[63];
    };

    void handle(const Request& request);
    void transfer(const Request& request);
    void usePins(uint8_t wiring);
    void sendState();
    void sendDiagnosis(bool riseTimes);
    void sendLastReplies();
    void writeMany(const Request& request);

    uint8_t m_partialWiring = 0xFF, m_partialBits = 0, m_partialStage = 0;
    uint32_t m_partialRaw = 0;
    uint16_t m_partialCount = 0;
    uint8_t m_oddWiring = 0xFF, m_odd[3] = {};
    uint16_t m_oddCount = 0;
    int sendOn(uint8_t wiring, const uint8_t* tx, unsigned txLen, uint8_t* rx, unsigned rxLen);

    struct WiringStats
    {
        uint16_t tries = 0;
        uint16_t replies = 0;
        uint8_t lastStage = 0;
        uint8_t mostBits = 0;
    };
    WiringStats m_stats[joybus::wiringCount];

    // (the queue's storage is static in joybus.cpp: the module lives on the main thread's 1 KB stack)
    struct k_msgq m_queue;
    volatile bool m_cancel = false;

    joybus::WiringSearch m_search;
    uint8_t m_pins = 0xFF;         // the wiring the pins are set up for
    uint8_t m_bitUs = 5;           // a GameCube's bits are 4.94 us
    bool m_autoBits = true;        // until a GBA answers, 5 and 16 us in turn
    void useBits(uint8_t bitUs);
};
