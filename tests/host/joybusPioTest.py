#!/usr/bin/env python3
"""The JOY Bus PIO program (src/layers/link/joybusProgram.h) in a PIO emulator, against a model GBA.

It builds the program with the firmware's own C (through a small shared library), checks its
encoding against an assembler, then runs it in rp2040-pio-emulator with a GBA on the line: the GBA
decodes the adapter's bits the way a JOY Bus receiver does (by the length of each low pulse), and
replies with bits of its own, jittered. It checks the adapter's bit timing, that every command
arrives intact, that every reply is read back intact, and what happens when the GBA says nothing.

Needs: a C compiler, pip install rp2040-pio-emulator adafruit-circuitpython-pioasm.
Run: python3 tests/host/joybusPioTest.py
"""
import ctypes
import os
import random
import subprocess
import sys
import tempfile

from collections import deque

from pioemu import State, emulate
import adafruit_pioasm

HERE = os.path.dirname(os.path.abspath(__file__))
HEADER = os.path.join(HERE, "..", "..", "src", "layers", "link", "joybusProgram.h")
CYCLES_PER_US = 4

SHIM = r"""
#include "%s"
unsigned build(uint16_t* out, unsigned bit_us, int push_pull, unsigned* wrap) { return joybus_build_program(out, bit_us, push_pull, wrap); }
unsigned pack(const uint8_t* tx, unsigned tx_len, unsigned rx_len, int push_pull, uint32_t* words) { return joybus_pack_command(tx, tx_len, rx_len, push_pull, words); }
unsigned sample_delay(void) { return JOYBUS_SAMPLE_DELAY; }
unsigned reply_words(unsigned rx_len) { return joybus_reply_words(rx_len); }
void unpack(const uint32_t* words, unsigned rx_len, uint8_t* rx) { joybus_unpack_reply(words, rx_len, rx); }
"""


def load_shim():
    d = tempfile.mkdtemp(prefix="joybus-test-")
    src = os.path.join(d, "shim.c")
    lib = os.path.join(d, "shim.so")
    with open(src, "w") as f:
        f.write(SHIM % os.path.abspath(HEADER))
    subprocess.check_call([os.environ.get("CC", "cc"), "-std=c99", "-Wall", "-Werror", "-shared", "-fPIC", "-o", lib, src])
    return ctypes.CDLL(lib), d


C, TMPDIR = load_shim()


def program(bit_us, push_pull):
    buf = (ctypes.c_uint16 * 32)()
    wrap = ctypes.c_uint()
    n = C.build(buf, bit_us, int(push_pull), ctypes.byref(wrap))
    assert n, f"no program for {bit_us} us bits"
    return list(buf[:n]), wrap.value


def pack(tx, rx_len, push_pull):
    words = (ctypes.c_uint32 * 3)()
    n = C.pack((ctypes.c_uint8 * 5)(*tx), len(tx), rx_len, int(push_pull), words)
    return list(words[:n])


def unpack(words, rx_len):
    w = (ctypes.c_uint32 * 2)(*(words + [0, 0])[:2])
    out = (ctypes.c_uint8 * 5)()
    C.unpack(w, rx_len, out)
    return list(out[:rx_len])


# ---------------------------------------------------------------- the encoding, against an assembler
def check_encoding():
    for bit_us in (4, 5):
        for push_pull in (False, True):
            q = bit_us
            low, high, bit = ("set pins, 0", "set pins, 1", "out pins, 1") if push_pull else ("set pindirs, 1", "set pindirs, 0", "out pindirs, 1")
            src = f"""
.program joybus
    pull block
    out x, 8
    out y, 8
    pull block
txbit:
    {low} [{q - 1}]
    {bit} [{2 * q - 2}]
    pull ifempty noblock
    {high} [{q - 2}]
    jmp x-- txbit
    {low} [{q - 1}]
    {high}
    wait 1 pin 0
rxbit:
    wait 0 pin 0 [{C.sample_delay()}]
    in pins, 1
    wait 1 pin 0
    jmp y-- rxbit
    push block
"""
            want = list(adafruit_pioasm.assemble(src))
            got, wrap = program(bit_us, push_pull)
            assert got == want, f"{bit_us} us: encoding differs\n got  {[hex(x) for x in got]}\n want {[hex(x) for x in want]}"
            assert wrap == len(got) - 1
    print("encoding: matches the assembler (4 and 5 us bits, open drain and push-pull)")


# ---------------------------------------------------------------- the model GBA
class Gba:
    """A JOY Bus device on the line. Times are in PIO cycles (CYCLES_PER_US a microsecond).

    Receiving: each low pulse is a bit, a 1 when shorter than 2 us; the first byte gives the
    command's length (5 bytes for a write, else 1), and the low pulse after it is the stop bit. The reply starts
    `delay_us` after the stop bit's rising edge: each bit low a quarter (a 1) or three quarters (a 0)
    of a `bit_us` bit (a GBA's are 3.81 us), each length jittered by up to `jitter` of itself, then
    the GBA's stop bit (2 us low)."""

    def __init__(self, replies, delay_us=3.0, jitter=0.05, seed=1, silent=False, bit_us=3.81, command_bit_us=4):
        self.half = command_bit_us / 2 * CYCLES_PER_US   # a command bit's low is a 1 when shorter than half the bit
        self.bit = bit_us * CYCLES_PER_US
        self.replies = replies          # function(command bytes) -> reply bytes
        self.delay = delay_us * CYCLES_PER_US
        self.jitter = jitter
        self.rng = random.Random(seed)
        self.silent = silent
        self.fall = None
        self.last_rise = None
        self.pulses = []                # (low length, high length before it) of the command being received
        self.received = []              # commands, as bit strings and bytes
        self.lows = []                  # every low pulse of the adapter's (cycles)
        self.highs = []                 # every high gap within a command (cycles)
        self.drive = []                 # the reply: (start, end) of each low, in cycles
        self.reply_end = 0

    def bits_for(self, data):
        t = self.reply_start
        out = []
        for byte in data:
            for i in range(7, -1, -1):
                bit = (byte >> i) & 1
                length = self.bit * (1 + self.rng.uniform(-self.jitter, self.jitter))
                low = (1 if bit else 3) * self.bit / 4 * (1 + self.rng.uniform(-self.jitter, self.jitter))
                out.append((t, t + low))
                t += length
        out.append((t, t + 2 * CYCLES_PER_US))   # stop bit
        self.reply_end = t + 4 * CYCLES_PER_US
        return out

    def line(self, clock, adapter_low):
        """The line at `clock`: low when the adapter or the GBA pulls it low."""
        gba_low = any(a <= clock < b for (a, b) in self.drive)
        return 0 if (adapter_low or gba_low) else 1

    def adapter_edge(self, clock, level):
        """The adapter let go of the line (level 1) or pulled it low (0) at `clock`."""
        if level == 0:
            self.fall = clock
            if self.last_rise is not None and self.pulses:
                self.highs.append(clock - self.last_rise)
            return
        low = clock - self.fall
        self.lows.append(low)
        self.pulses.append(low)
        self.last_rise = clock
        # the command's length is known from its first byte (a write is 5 bytes); the pulse after
        # its bits is the stop bit, and the reply follows that
        if len(self.pulses) > 8 and len(self.pulses) == 8 * self.command_length() + 1:
            self.end_of_command(clock)

    def command_length(self):
        first = int("".join("1" if p < self.half else "0" for p in self.pulses[:8]), 2)
        return 5 if first == 0x15 else 1

    def end_of_command(self, clock):
        pulses = self.pulses
        self.pulses = []
        *data, stop = pulses
        bits = "".join("1" if p < self.half else "0" for p in data)
        assert stop < self.half, f"the stop bit is {stop / CYCLES_PER_US} us low"
        assert len(bits) % 8 == 0, f"a command of {len(bits)} bits"
        cmd = [int(bits[i:i + 8], 2) for i in range(0, len(bits), 8)]
        self.received.append(cmd)
        if self.silent:
            return
        self.reply_start = self.last_rise + self.delay
        self.drive = self.bits_for(self.replies(cmd))


def run_transactions(bit_us, two_lines, transactions, gba, max_cycles=40000):
    push_pull = two_lines   # as the firmware: a line of the adapter's own is driven, a shared one open drain
    """Run the program over `transactions` [(command, reply length)], feeding the TX FIFO the way
    the firmware does (all of a transaction's words at once, the next when the reply is in).
    Returns the replies read, or None for a transaction still waiting when time ran out."""
    prog, wrap = program(bit_us, push_pull)
    tx_pin = 1 if two_lines else 0   # the emulator reads pins from 0: the RX pin is pin 0
    rx_pin = 0
    pending = list(transactions)
    replies = []
    state_box = {"want": None, "words": []}

    def pulls_low(state):
        # open drain: the output latch is 0, so a set pindir pulls low (the emulator keeps no latch apart
        # from the pin's level, so only the direction is looked at); push-pull: an output at 0
        out = bool(state.pin_directions & (1 << tx_pin))
        return out and not state.pin_values & (1 << tx_pin) if push_pull else out

    def source(state: State) -> int:
        level = gba.line(state.clock, pulls_low(state))
        return level << rx_pin if two_lines else level

    first = pending.pop(0)
    rest = (1 << tx_pin) if push_pull else 0   # push-pull: an output resting high
    state = State(transmit_fifo=deque(pack(*first, push_pull)), pin_directions=rest, pin_values=rest)
    state_box["want"] = first

    def stop_when(_, s):
        return s.clock >= max_cycles

    for before, after in emulate(prog, initial_state=state, input_source=source, stop_when=stop_when,
                            shift_osr_right=False, shift_isr_right=False, auto_push=True, push_threshold=32,
                            out_base=tx_pin, out_count=1, set_base=tx_pin, set_count=1, wrap_target=0, wrap_top=wrap):
        # the emulator runs an instruction's delay by moving its clock on: a pin it changed changed
        # when it started, at before.clock
        if pulls_low(before) != pulls_low(after):
            gba.adapter_edge(before.clock, 0 if pulls_low(after) else 1)
        # the CPU: take the reply's words as they arrive, then give the next transaction
        while after.receive_fifo:
            state_box["words"].append(after.receive_fifo.popleft())
        want = state_box["want"]
        if want and len(state_box["words"]) == C.reply_words(want[1]):
            replies.append(unpack(state_box["words"], want[1]))
            state_box["words"] = []
            state_box["want"] = None
            if pending:
                nxt = pending.pop(0)
                state_box["want"] = nxt
                after.transmit_fifo.extend(pack(*nxt, push_pull))
        if not state_box["want"] and not pending and after.clock > gba.reply_end + 40:
            break
    if state_box["want"]:
        replies.append(None)
    return replies, after


# ---------------------------------------------------------------- the tests
def gba_replies(cmd):
    """A GBA's answers (GBATEK): JOYSTAT 0x08 after a write, type 0x0004."""
    op = cmd[0]
    if op in (0x00, 0xFF):
        return [0x00, 0x04, 0x08]
    if op == 0x14:
        return [0xDE, 0xAD, 0x5A, 0x01, 0x28]
    if op == 0x15:
        return [0x02]
    raise AssertionError(f"command {op:02x}")


TRANSACTIONS = [
    ([0xFF], 3),
    ([0x00], 3),
    ([0x15, 0x12, 0x34, 0x56, 0x78], 1),
    ([0x15, 0xFF, 0x00, 0xA5, 0x5A], 1),
    ([0x14], 5),
    ([0x00], 3),
]


def check_round_trips():
    for bit_us in (4, 5, 16):
        for two_lines in (True, False):
            # (the reply's delay, jitter, seed, the GBA's bit length: 3.81 us measured, and either side of it)
            for delay_us, jitter, seed, gba_bit in ((2.0, 0.0, 1, 3.81), (6.6, 0.05, 2, 3.81), (12.0, 0.08, 3, 3.5), (60.0, 0.05, 4, 4.2)):
                gba = Gba(gba_replies, delay_us=delay_us, jitter=jitter, seed=seed, bit_us=gba_bit, command_bit_us=bit_us)
                replies, end = run_transactions(bit_us, two_lines, TRANSACTIONS, gba, max_cycles=40000 * bit_us // 4)
                label = f"{bit_us} us bits, {'two lines, driven' if two_lines else 'one line, open drain'}, {gba_bit} us reply bits after {delay_us} us, jitter {jitter:.0%}"
                assert gba.received == [t[0] for t in TRANSACTIONS], f"{label}: the GBA got {gba.received}"
                want = [gba_replies(t[0]) for t in TRANSACTIONS]
                assert replies == want, f"{label}: read {replies}, want {want}"
                # the adapter's bits: a quarter low for a 1 (and the stop bit), three quarters for a 0
                q = bit_us   # cycles in a quarter bit at 4 cycles a microsecond
                assert set(gba.lows) <= {q, 3 * q}, f"{label}: low pulses {sorted(set(gba.lows))}"
                assert set(gba.highs) <= {q, 3 * q}, f"{label}: high gaps {sorted(set(gba.highs))}"
                print(f"round trips: {label}: {len(TRANSACTIONS)} commands and replies intact")


def check_no_reply():
    gba = Gba(gba_replies, silent=True)
    replies, end = run_transactions(4, True, [([0x00], 3)], gba, max_cycles=4000)
    assert replies == [None], replies
    assert gba.received == [[0x00]]
    assert not end.receive_fifo and end.program_counter == 12, (end.program_counter, end.receive_fifo)
    print("no reply: the program waits at the first reply bit, nothing read (the firmware times out and restarts it)")


def check_packing():
    for tx in ([0x00], [0x14], [0x15, 1, 2, 3, 4], [0xFF]):
        for rx_len in (1, 3, 5):
            words = pack(tx, rx_len, False)
            assert pack(tx, rx_len, True)[1] == words[1] ^ 0xFFFFFFFF   # push-pull: the bits as they are
            assert words[0] >> 24 == len(tx) * 8 - 1 and (words[0] >> 16) & 0xFF == rx_len * 8 - 1
            assert len(words) == (3 if len(tx) > 4 else 2)
    assert unpack([0x00000408], 3) == [0x00, 0x04, 0x08]
    assert unpack([0xDEAD5A01, 0x28], 5) == [0xDE, 0xAD, 0x5A, 0x01, 0x28]
    assert unpack([0x02], 1) == [0x02]
    print("packing: words and replies")


if __name__ == "__main__":
    try:
        check_encoding()
        check_packing()
        check_round_trips()
        check_no_reply()
    finally:
        import shutil
        shutil.rmtree(TMPDIR, ignore_errors=True)
    print("all passed")
