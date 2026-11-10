#include "joybus.hpp"

#include <cstring>

#include "../hardware.hpp"
#include "../layers/link/joybusLayer.h"
#include "../layers/link/gbaLinkLayer.hpp"
#include "../layers/transport.hpp"

/* a reply's time beyond the bits themselves: the GBA's turnaround */
static constexpr uint32_t replySlackUs = 500;

JoybusModule::JoybusModule()
{
    k_msgq_init(&m_queue, m_queueBuffer, sizeof(Request), 8);
}

void JoybusModule::cancel()
{
    m_cancel = true;
    Request wake{};   // wakes execute() to see m_cancel
    k_msgq_put(&m_queue, &wake, K_NO_WAIT);
}

/* From the USB or serial receive path (an interrupt, for serial): queued for execute()'s thread. */
void JoybusModule::receiveCommand(std::span<const uint8_t> command)
{
    Request request{};
    request.length = static_cast<uint8_t>(command.size() < sizeof(request.data) ? command.size() : sizeof(request.data));
    std::memcpy(request.data, command.data(), request.length);
    if (k_msgq_put(&m_queue, &request, K_NO_WAIT) != 0 && request.data[0] == static_cast<uint8_t>(Command::Transfer)
        && request.length >= 2) {
        /* the queue is full (the host sends one at a time, so it shouldn't be): say so at once */
        const uint8_t reply[] = { request.data[0], request.data[1], 2, m_search.wiring(), 0 };
        Transport::sendData(std::span<const uint8_t>(reply, sizeof(reply)));
    }
}

void JoybusModule::execute()
{
    m_cancel = false;
    Hardware::getInstance().setVoltage3V3();   // a GBA's link port is 3.3 V
    link_disable();   // the GBA link layer lets go of the PIO
    if (!joybus_start(m_bitUs)) return;
    m_pins = 0xFF;
    usePins(m_search.wiring());

    Request request;
    while (!m_cancel) {
        if (k_msgq_get(&m_queue, &request, K_FOREVER) != 0) continue;
        if (m_cancel) break;
        handle(request);
    }
    joybus_stop();
    k_msgq_purge(&m_queue);
}

void JoybusModule::handle(const Request& request)
{
    if (request.length < 1) return;
    switch (static_cast<Command>(request.data[0])) {
    case Command::Transfer:
        return transfer(request);
    case Command::SetWiring:
        if (request.length >= 2) {
            if (request.data[1] == joybus::wiringAuto) m_search.setAuto();
            else m_search.setFixed(request.data[1]);
            usePins(m_search.wiring());
        }
        return sendState();
    case Command::SetBitLength:
        if (request.length >= 2 && request.data[1] >= 4 && request.data[1] <= 7 && request.data[1] != m_bitUs) {
            m_bitUs = request.data[1];
            joybus_stop();
            joybus_start(m_bitUs);
            m_pins = 0xFF;
            usePins(m_search.wiring());
        }
        return sendState();
    case Command::GetState:
        return sendState();
    case Command::Diagnose:
        return sendDiagnosis(request.length >= 2 && (request.data[1] & 1));
    case Command::LastReplies:
        return sendLastReplies();
    }
}

void JoybusModule::usePins(uint8_t wiring)
{
    if (wiring == m_pins) return;
    m_pins = wiring;
    joybus_set_pins(joybus::wirings[wiring].txPin, joybus::wirings[wiring].rxPin, joybus::wirings[wiring].invertTx, joybus::wirings[wiring].invertRx);
}

/* One command on a wiring, counted for Diagnose. */
int JoybusModule::sendOn(uint8_t wiring, const uint8_t* tx, unsigned txLen, uint8_t* rx, unsigned rxLen)
{
    const joybus::Wiring& w = joybus::wirings[wiring];
    if ((w.invertTx || w.invertRx) && wiring != m_search.wiring()) {
        /* lines the other way up rest low: tried only when the GBA's line does, so a GBA that rests
         * high never gets a command upside down */
        usePins(0);
        uint8_t high = 0, low = 0;
        joybus_sample_pins(&high, &low);
        if (high & (1u << w.rxPin)) return -1;
    }
    usePins(wiring);
    WiringStats& stats = m_stats[wiring];
    if (stats.tries < 0xFFFF) stats.tries++;
    const int n = joybus_transfer(tx, txLen, rx, rxLen, replySlackUs);
    if (n > 0) {
        if (stats.replies < 0xFFFF) stats.replies++;
        /* a whole reply to a status or reset that isn't a GBA's: kept, as it may be one read the wrong way up */
        if (txLen == 1 && rxLen == 3 && (tx[0] == 0x00 || tx[0] == 0xFF) && !joybus::isGbaType(rx, n)) {
            m_oddWiring = wiring;
            m_odd[0] = rx[0]; m_odd[1] = rx[1]; m_odd[2] = rx[2];
            if (m_oddCount < 0xFFFF) m_oddCount++;
        }
    } else {
        uint8_t bits = 0;
        joybus_last_failure(&stats.lastStage, &bits);
        if (bits > stats.mostBits) stats.mostBits = bits;
        if (bits) {
            m_partialWiring = wiring;
            m_partialBits = bits;
            m_partialStage = stats.lastStage;
            m_partialRaw = joybus_last_failure_raw();
            if (m_partialCount < 0xFFFF) m_partialCount++;
        }
    }
    return n;
}

void JoybusModule::transfer(const Request& request)
{
    uint8_t reply[5 + 5 + 4] = {};
    reply[0] = request.data[0];
    reply[1] = request.length >= 2 ? request.data[1] : 0;
    const unsigned txLen = request.length >= 3 ? request.data[2] : 0;
    const bool thenStatus = request.length >= 4 && (request.data[3] & 0x80);
    const unsigned rxLen = request.length >= 4 ? (request.data[3] & 0x7F) : 0;
    if (txLen < 1 || txLen > 5 || rxLen < 1 || rxLen > 5 || request.length < 4 + txLen) {
        reply[2] = 2;
        reply[3] = m_search.wiring();
        Transport::sendData(std::span<const uint8_t>(reply, 5));
        return;
    }
    auto send = [this](uint8_t wiring, const uint8_t* tx, unsigned txLen, uint8_t* rx, unsigned rxLen) {
        return sendOn(wiring, tx, txLen, rx, rxLen);
    };
    const int n = m_search.transfer(send, &request.data[4], txLen, &reply[5], rxLen);
    usePins(m_search.wiring());   // back on the wiring in use after a search that found nothing
    reply[2] = n > 0 ? 0 : 1;
    reply[3] = m_search.wiring();
    reply[4] = n > 0 ? static_cast<uint8_t>(n) : 0;
    size_t length = 5 + (n > 0 ? n : 0);
    if (thenStatus) {
        static const uint8_t status[] = { 0x00 };
        uint8_t* s = &reply[length];
        const int k = m_search.transfer(send, status, 1, s + 1, 3);
        usePins(m_search.wiring());
        s[0] = k == 3 ? 0 : 1;
        if (k != 3) s[1] = s[2] = s[3] = 0;
        length += 4;
    }
    Transport::sendData(std::span<const uint8_t>(reply, length));
}

void JoybusModule::sendState()
{
    const uint8_t state[] = {
        static_cast<uint8_t>(Command::GetState),
        protocolVersion,
        m_search.isFound() || m_search.isFixed() ? m_search.wiring() : joybus::wiringAuto,
        static_cast<uint8_t>(m_search.isFixed() ? 1 : 0),
        m_bitUs,
        joybus::wiringCount,
    };
    Transport::sendData(std::span<const uint8_t>(state, sizeof(state)));
}

void JoybusModule::sendDiagnosis(bool riseTimes)
{
    uint8_t d[13 + joybus::wiringCount * 6] = {};
    d[0] = static_cast<uint8_t>(Command::Diagnose);
    d[1] = 1;
    d[2] = joybus::wiringCount;
    d[3] = m_search.wiring();
    d[4] = static_cast<uint8_t>((m_search.isFound() ? 1 : 0) | (m_search.isFixed() ? 2 : 0));
    d[5] = m_bitUs;
    joybus_sample_pins(&d[6], &d[7]);
    for (unsigned pin = 0; pin < 5; pin++) d[8 + pin] = 0xFE;
    if (riseTimes) {
        /* only the lines a wiring tried unasked sends on: the others may be the GBA's outputs */
        for (uint8_t w = 0; w < joybus::wiringCount; w++) {
            const unsigned pin = joybus::wirings[w].txPin;
            if (joybus::wirings[w].autoProbe && d[8 + pin] == 0xFE) d[8 + pin] = joybus_rise_time(pin);
        }
        m_pins = 0xFF;
        usePins(m_search.wiring());
    }
    for (uint8_t w = 0; w < joybus::wiringCount; w++) {
        uint8_t* p = &d[13 + w * 6];
        p[0] = static_cast<uint8_t>(m_stats[w].tries);
        p[1] = static_cast<uint8_t>(m_stats[w].tries >> 8);
        p[2] = static_cast<uint8_t>(m_stats[w].replies);
        p[3] = static_cast<uint8_t>(m_stats[w].replies >> 8);
        p[4] = m_stats[w].lastStage;
        p[5] = m_stats[w].mostBits;
    }
    Transport::sendData(std::span<const uint8_t>(d, sizeof(d)));
}

void JoybusModule::sendLastReplies()
{
    const uint8_t d[] = {
        static_cast<uint8_t>(Command::LastReplies), 1,
        m_partialWiring, m_partialBits, m_partialStage,
        static_cast<uint8_t>(m_partialRaw), static_cast<uint8_t>(m_partialRaw >> 8),
        static_cast<uint8_t>(m_partialRaw >> 16), static_cast<uint8_t>(m_partialRaw >> 24),
        static_cast<uint8_t>(m_partialCount), static_cast<uint8_t>(m_partialCount >> 8),
        m_oddWiring, m_odd[0], m_odd[1], m_odd[2],
        static_cast<uint8_t>(m_oddCount), static_cast<uint8_t>(m_oddCount >> 8),
    };
    Transport::sendData(std::span<const uint8_t>(d, sizeof(d)));
}
