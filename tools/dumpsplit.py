#!/usr/bin/env python3
"""Turn a DUMP: hex block from either emulator into disassembly.

Both emulators print the same format at the end of a run:

    DUMP: <emu> guest=<pc> host=<addr> guest_ops=<n> bytes=<m>
    DUMP: <64 hex chars>
    ...
    DUMP: end

Feed this script a captured console log from either side.  It writes one .bin
per block and runs sh-elf-objdump over each, so the same guest address from
bloom and from bloop can be read side by side.

    dc-tool-ip -t <ip> -x bloom.elf | tee bloom.log
    python3 tools/dumpsplit.py bloom.log bloop.log

WHY THIS EXISTS.  bloom issues 8.76 SH-4 instructions per guest instruction
against bloop's 4.76, while emitting *denser* code statically.  Both cannot be
true of the block bodies, so the difference has to sit at the edges — prologue,
epilogue, dispatcher tail.  That is an inference from two ratios taken on
different builds; this makes it a count.  Read each block and bin its
instructions into body / prologue / epilogue / tail, then divide by the guest
instruction count.  The parts should sum to the measured per-instruction rate;
if they do not, the inference was wrong and the templates are the problem.
"""

import re
import subprocess
import sys
from pathlib import Path

HDR = re.compile(
    r"DUMP:\s+(\w+)\s+guest=([0-9a-fA-F]+)\s+host=([0-9a-fA-F]+)"
    r"\s+guest_ops=(\d+)\s+bytes=(\d+)"
)
HEX = re.compile(r"DUMP:\s+([0-9a-fA-F]+)\s*$")

OBJDUMP = "sh-elf-objdump"


def parse(path):
    """Yield (emu, guest, host, ops, nbytes, data) for each block in a log."""
    cur = None
    payload = []

    for line in Path(path).read_text(errors="replace").splitlines():
        m = HDR.search(line)
        if m:
            cur = (m.group(1), int(m.group(2), 16), int(m.group(3), 16),
                   int(m.group(4)), int(m.group(5)))
            payload = []
            continue

        if cur and "DUMP: end" in line:
            yield (*cur, bytes.fromhex("".join(payload)))
            cur, payload = None, []
            continue

        if cur:
            m = HEX.match(line.strip())
            if m:
                payload.append(m.group(1))


def main(logs):
    out = Path("dumps")
    out.mkdir(exist_ok=True)
    found = 0

    for log in logs:
        for emu, guest, host, ops, nbytes, data in parse(log):
            if not data:
                continue

            found += 1
            name = out / f"{emu}_{guest:08x}.bin"
            name.write_bytes(data)

            print(f"\n=== {emu}  guest {guest:08x}  host {host:08x}  "
                  f"{len(data)} bytes  guest_ops={ops or '?'}")

            try:
                # --adjust-vma so the listing's addresses are the real host
                # ones, which is what the PC sampler and the block table print.
                subprocess.run(
                    [OBJDUMP, "-b", "binary", "-m", "sh4", "-D",
                     f"--adjust-vma=0x{host:x}", str(name)],
                    check=True)
            except FileNotFoundError:
                print(f"({OBJDUMP} not found — {name} written, disassemble "
                      f"it yourself)")

    if not found:
        print("no DUMP: blocks in", ", ".join(logs), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1:]))
