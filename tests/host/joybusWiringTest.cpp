// Host-side tests for finding a GBA's JOY Bus wiring (src/module/joybusWiring.hpp).
// Build & run:  make -C tests/host

#include <cstdio>
#include <cstring>
#include <vector>

#include "../../src/module/joybusWiring.hpp"

using namespace joybus;

static int g_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

// A GBA reachable on one wiring (or none), recording every command sent on each.
struct FakeGba
{
    int answersOn = -1;          // the wiring it answers on; -1: switched off
    bool other = false;          // answer as some other device (a controller, type 0x0900)
    std::vector<std::pair<int, uint8_t>> sent;   // (wiring, command)

    int operator()(uint8_t wiring, const uint8_t* tx, unsigned, uint8_t* rx, unsigned rxLen)
    {
        sent.push_back({ wiring, tx[0] });
        if (wiring != answersOn) return -1;
        if (tx[0] == 0x00 || tx[0] == 0xFF) {
            rx[0] = other ? 0x09 : 0x00;
            rx[1] = other ? 0x00 : 0x04;
            rx[2] = 0x08;
            return 3;
        }
        std::memset(rx, 0x5A, rxLen);
        return (int)rxLen;
    }
};

static const uint8_t kStatus[] = { 0x00 };
static const uint8_t kReset[] = { 0xFF };
static const uint8_t kWrite[] = { 0x15, 1, 2, 3, 4 };
static const uint8_t kRead[] = { 0x14 };

static int probe(WiringSearch& s, FakeGba& g, const uint8_t* cmd = kStatus)
{
    uint8_t rx[5];
    return s.transfer(g, cmd, 1, rx, 3);
}

static void testFindsTheWiringFromOneStatus()
{
    for (int w = 0; w < wiringCount; w++) {
        if (!wirings[w].autoProbe) continue;
        WiringSearch s;
        FakeGba g;
        g.answersOn = w;
        CHECK(probe(s, g) == 3);
        CHECK(s.wiring() == w);
        CHECK(s.isFound());
        // then it stays there: one command on that wiring each time
        g.sent.clear();
        CHECK(probe(s, g, kReset) == 3);
        CHECK(g.sent.size() == 1 && g.sent[0].first == w);
    }
}

static void testNeverTriesUnsafeWirings()
{
    WiringSearch s;
    FakeGba g;
    g.answersOn = 4;   // only on a wiring that would pull the GBA's SO low on a GBC cable
    CHECK(probe(s, g) == -1);
    CHECK(!s.isFound());
    for (auto& [w, cmd] : g.sent) CHECK(wirings[w].autoProbe);
    CHECK(s.wiring() == 0);   // back where it started
}

static void testNeverSendsOtherCommandsTwice()
{
    WiringSearch s;
    FakeGba g;
    g.answersOn = 2;
    uint8_t rx[5];
    CHECK(s.transfer(g, kWrite, 5, rx, 1) == -1);   // not found yet: on wiring 0 only
    CHECK(s.transfer(g, kRead, 1, rx, 5) == -1);
    CHECK(g.sent.size() == 2);
    CHECK(probe(s, g) == 3);
    CHECK(s.wiring() == 2);
    g.sent.clear();
    CHECK(s.transfer(g, kWrite, 5, rx, 1) == 1);
    CHECK(g.sent.size() == 1 && g.sent[0].first == 2);
}

static void testAnotherDeviceIsNotAGba()
{
    WiringSearch s;
    FakeGba g;
    g.answersOn = 0;
    g.other = true;
    CHECK(probe(s, g) == -1);
    CHECK(!s.isFound());
}

static void testSwitchedOffKeepsTheWiringThenLooksAgain()
{
    WiringSearch s;
    FakeGba g;
    g.answersOn = 1;
    CHECK(probe(s, g) == 3);
    g.answersOn = -1;   // switched off: the game keeps looking
    for (int i = 0; i + 1 < silentProbesToForget; i++) {
        g.sent.clear();
        CHECK(probe(s, g) == -1);
        CHECK(g.sent.size() == 1);   // only the wiring it found
        CHECK(s.isFound());
    }
    g.answersOn = 3;   // on again, on another cable
    CHECK(probe(s, g) == 3);       // the 120th: forgotten, and found again at once
    CHECK(s.wiring() == 3);
}

static void testFixedWiringNeverSearches()
{
    WiringSearch s;
    FakeGba g;
    g.answersOn = 0;
    s.setFixed(5);
    CHECK(probe(s, g) == -1);
    CHECK(g.sent.size() == 1 && g.sent[0].first == 5);
    CHECK(s.wiring() == 5);
    s.setAuto();
    g.sent.clear();
    CHECK(probe(s, g) == 3);
    CHECK(s.wiring() == 0);
    s.setFixed(42);   // not a wiring: ignored
    CHECK(s.wiring() == 0 && !s.isFixed());
}

int main()
{
    testFindsTheWiringFromOneStatus();
    testNeverTriesUnsafeWirings();
    testNeverSendsOtherCommandsTwice();
    testAnotherDeviceIsNotAGba();
    testSwitchedOffKeepsTheWiringThenLooksAgain();
    testFixedWiringNeverSearches();
    if (g_failures) {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("joybusWiringTest: all passed\n");
    return 0;
}
