#pragma once

/* Which link pins carry a GBA's JOY Bus, found by asking (module/joybus.hpp). Free of Zephyr, so
 * tests/host/joybusWiringTest.cpp runs it. */

#include <cstdint>

namespace joybus
{
    struct Wiring
    {
        uint8_t txPin;
        uint8_t rxPin;
        bool autoProbe;   // safe to try unasked: never pulls down one of the GBA's outputs
    };

    /* GPIO: 0 SC, 1 SI, 2 SO, 3 SD (GBA cable), 4 SD (GBC cable). A GBC cable crosses SO and SI. */
    inline constexpr Wiring wirings[] = {
        { 2, 1, true  },  // 0: send on SO (the GBA's SI on a GBC cable), hear SI (its SO): GBATEK's
        { 2, 2, true  },  // 1: SO alone (the GBA's SI): a GBA that answers on the line it listens on
        { 4, 4, true  },  // 2: SD alone, GBC cable
        { 3, 3, true  },  // 3: SD alone, GBA cable
        { 1, 2, false },  // 4: send on SI, hear SO: only for a cable that doesn't cross them
        { 1, 1, false },  // 5: SI alone
    };
    inline constexpr uint8_t wiringCount = sizeof(wirings) / sizeof(wirings[0]);
    inline constexpr uint8_t wiringAuto = 0xFF;

    /* status/reset commands unanswered in a row before a found wiring is looked for again (a cable
     * changed): about 2 seconds of a GameCube looking for a GBA */
    inline constexpr uint16_t silentProbesToForget = 120;

    /* A GBA's answer to status or reset: its type, 0x0004 */
    inline bool isGbaType(const uint8_t* rx, int n) { return n == 3 && rx[0] == 0x00 && rx[1] == 0x04; }

    /* By default the wiring is found: when a status (0x00) or reset (0xFF) command gets no answer,
     * the same command goes out on each wiring that is safe to try, and the first a GBA answers is
     * kept. Only those two commands are ever sent twice; they change nothing on the GBA. */
    class WiringSearch
    {
    public:
        /* Transfer(wiring, tx, txLen, rx, rxLen) -> rxLen, or < 0 for no reply. */
        template <class Transfer>
        int transfer(Transfer&& send, const uint8_t* tx, unsigned txLen, uint8_t* rx, unsigned rxLen)
        {
            const bool probe = txLen == 1 && rxLen == 3 && (tx[0] == 0x00 || tx[0] == 0xFF);
            int n = send(m_wiring, tx, txLen, rx, rxLen);
            if (!probe) return n;

            if (n > 0 && isGbaType(rx, n)) {
                found();
                return n;
            }
            if (m_found && !m_fixed && ++m_silentProbes >= silentProbesToForget) m_found = false;
            if (m_found || m_fixed) return n;
            /* not found yet: whatever came back wasn't a GBA, so look on the other wirings */

            for (uint8_t i = 1; i < wiringCount; i++) {
                const uint8_t w = static_cast<uint8_t>((m_wiring + i) % wiringCount);
                if (!wirings[w].autoProbe) continue;
                const int k = send(w, tx, txLen, rx, rxLen);
                if (k > 0 && isGbaType(rx, k)) {
                    m_wiring = w;
                    found();
                    return k;
                }
            }
            return -1;
        }

        void setFixed(uint8_t wiring)
        {
            if (wiring >= wiringCount) return;
            m_wiring = wiring;
            m_fixed = true;
            m_found = false;
        }
        void setAuto()
        {
            m_fixed = false;
            m_found = false;
        }

        uint8_t wiring() const { return m_wiring; }
        bool isFound() const { return m_found; }
        bool isFixed() const { return m_fixed; }

    private:
        void found()
        {
            m_found = true;
            m_silentProbes = 0;
        }

        uint8_t m_wiring = 0;
        bool m_found = false;
        bool m_fixed = false;
        uint16_t m_silentProbes = 0;
    };
}
