#!/usr/bin/env python3
"""
netlist_audit.py -- Phase 0 pin audit for the kayak LED turret PCB.

Reads a KiCad (8/9) S-expression netlist ("kayak working.net") and prints:

  1. Arduino pin -> GPIO -> net -> every destination on that net
  2. Electrical-risk checks that must pass BEFORE firmware assumes the wiring:
       - any MCU signal pin sharing a net with a >3.3 V rail or 5 V-side pin
       - TMC2209 VIO rail (must be 3.3 V: module pulls PDN_UART/DIAG up to VIO)
       - MCU pins wired to 74HCT245 B-side (5 V outputs) instead of A-side
       - 74HCT245 DIR / OE tie-offs (DIR -> VCC for A->B, OE -> GND)
       - TMC2209 MS1/MS2 tie-offs -> decoded UART address per driver
       - common ground: every part's GND pins on one net
       - MCU power-input pin used (VIN vs 5V/VBUS)
       - D13 use (off-limits), single-node / unconnected nets
  3. Nets with no MCU consumer (informational)

The netlist is the ground truth; this script only reports. It never guesses a
connection: anything it cannot classify is printed as UNKNOWN for a human.

Usage:
    python3 tools/netlist_audit.py "kayak working.net" [--mcu REF] [--md OUT.md]

Nano ESP32 (ABX00083) D/A -> GPIO map below is copied from the board core's
variants/arduino_nano_nora/pins_arduino.h (arduino:esp32 2.0.17, the
"#else // ESP32-style definitions" block), not typed from memory.
"""
import argparse
import re
import sys
from collections import defaultdict

NANO_ESP32_GPIO = {
    "D0": 44, "D1": 43, "D2": 5, "D3": 6, "D4": 7, "D5": 8, "D6": 9,
    "D7": 10, "D8": 17, "D9": 18, "D10": 21, "D11": 38, "D12": 47,
    "D13": 48, "A0": 1, "A1": 2, "A2": 3, "A3": 4, "A4": 11, "A5": 12,
    "A6": 13, "A7": 14,
}

# Destination classifiers: (label, regex on component value/part/description)
PART_CLASSES = [
    ("MCU", r"nano.?esp32|abx00083|arduino.?nano"),
    ("TMC2209", r"tmc2209|tmc.?22"),
    ("HCT245", r"(74)?hc[t]?245"),
    ("IMU", r"gy-?521|mpu.?6050"),
]

# A net is a rail only if its WHOLE name is a rail name (hierarchical "/" prefix
# allowed) or it carries a power pin with a rail name. Substring matches such as
# "/LED_DATA_5V" are deliberately NOT rails -- the B-side check catches those.
HIGH_RAIL = re.compile(r"^/?(\+?5V|5V0|\+5V0|\+?12V|VBUS|VUSB|VIN|\+?18V|VMOT|VM|VS|VBAT|\+?BATT?)$", re.I)
RAIL_3V3 = re.compile(r"^/?(\+?3V3|\+?3\.3V|VDD_?3V3|3V3_?OUT)$", re.I)
GND_NET = re.compile(r"^/?(GND|GNDD|GNDA|PGND|0V|COMMON.?GND)$|(^|/)GND($|[^A-Z])", re.I)


# ----------------------------------------------------------------------------
# S-expression parsing
# ----------------------------------------------------------------------------
def tokenize(text):
    tok = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+')
    for m in tok.finditer(text):
        t = m.group(0)
        if t[0] == '"':
            yield ("S", bytes(t[1:-1], "utf-8").decode("unicode_escape"))
        else:
            yield ("T", t)


def parse(text):
    stack, cur = [], []
    for kind, t in tokenize(text):
        if kind == "T" and t == "(":
            stack.append(cur)
            cur = []
        elif kind == "T" and t == ")":
            done = cur
            cur = stack.pop()
            cur.append(done)
        else:
            cur.append(t)
    return cur[0] if cur else []


def find_all(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def get(node, key, default=""):
    hits = find_all(node, key)
    if hits and len(hits[0]) > 1 and not isinstance(hits[0][1], list):
        return hits[0][1]
    return default


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
class Node:
    def __init__(self, ref, pin, func, ptype):
        self.ref, self.pin, self.func, self.ptype = ref, pin, func, ptype

    def label(self, comps):
        c = comps.get(self.ref, {})
        fn = f"({self.func})" if self.func else ""
        return f"{self.ref}.{self.pin}{fn} [{c.get('value', '?')}]"


def load(path):
    with open(path, encoding="utf-8") as f:
        root = parse(f.read())
    comps = {}
    for comp_list in find_all(root, "components"):
        for c in find_all(comp_list, "comp"):
            lib = find_all(c, "libsource")
            part = get(lib[0], "part") if lib else ""
            desc = get(c, "description") or (get(lib[0], "description") if lib else "")
            ref = get(c, "ref")
            comps[ref] = {"ref": ref, "value": get(c, "value"), "part": part,
                          "desc": desc, "footprint": get(c, "footprint")}
    nets = {}
    for net_list in find_all(root, "nets"):
        for n in find_all(net_list, "net"):
            name = get(n, "name")
            nodes = [Node(get(x, "ref"), get(x, "pin"), get(x, "pinfunction"),
                          get(x, "pintype")) for x in find_all(n, "node")]
            nets.setdefault(name, []).extend(nodes)  # merge, never overwrite
    return comps, nets


def classify(comp):
    hay = " ".join([comp.get("value", ""), comp.get("part", ""), comp.get("desc", "")])
    for label, rx in PART_CLASSES:
        if re.search(rx, hay, re.I):
            return label
    ref = comp.get("ref", "")
    if ref.startswith("J"):
        return "CONN"
    return "OTHER"


def pin_name(func):
    """Normalise a pinfunction like 'D4', 'A0/DTR', 'GPIO7/D4' to 'D4'/'A0'."""
    for tok in re.split(r"[/\s,_]+", func or ""):
        if tok.upper() in NANO_ESP32_GPIO:
            return tok.upper()
    return None


# ----------------------------------------------------------------------------
# Audit
# ----------------------------------------------------------------------------
def audit(comps, nets, mcu_ref=None):
    out, findings = [], []
    danger = lambda m: findings.append(("DANGER", m))
    warn = lambda m: findings.append(("WARN", m))
    info = lambda m: findings.append(("INFO", m))

    cls = {r: classify(c) for r, c in comps.items()}
    if not mcu_ref:
        mcus = [r for r, k in cls.items() if k == "MCU"]
        if len(mcus) != 1:
            sys.exit(f"Could not identify a single MCU (found {mcus}); pass --mcu REF")
        mcu_ref = mcus[0]

    net_of = {}  # (ref, pin) -> net name
    by_ref = defaultdict(list)  # ref -> [(node, net)]
    for name, nodes in nets.items():
        for n in nodes:
            net_of[(n.ref, n.pin)] = name
            by_ref[n.ref].append((n, name))

    def net_is_high(name):
        nodes = nets.get(name, [])
        return bool(HIGH_RAIL.search(name)) or any(
            n.ptype == "power_out" and HIGH_RAIL.search(n.func or "") for n in nodes)

    def net_level(name):
        if GND_NET.search(name or ""):
            return "LOW"
        if RAIL_3V3.search(name or "") or HIGH_RAIL.search(name or ""):
            return "HIGH"
        return "UNKNOWN"

    # ---- 1. MCU pin table ---------------------------------------------------
    out.append(f"## MCU pin table ({mcu_ref} = {comps[mcu_ref]['value']})\n")
    out.append("| Pin# | Arduino | GPIO | Net | Destinations |")
    out.append("|---|---|---|---|---|")
    mcu_nodes = sorted(by_ref[mcu_ref], key=lambda t: (
        NANO_ESP32_GPIO.get(pin_name(t[0].func) or "", 999), t[0].pin))
    for n, name in mcu_nodes:
        an = pin_name(n.func)
        gpio = NANO_ESP32_GPIO.get(an, "") if an else ""
        others = [o.label(comps) for o in nets[name] if o.ref != mcu_ref]
        dest = "<br>".join(others) if others else "**(no other node)**"
        out.append(f"| {n.pin} | {n.func or '?'} | {gpio} | `{name}` | {dest} |")

        if an and name.lower().startswith("unconnected"):
            continue
        if an:
            if net_is_high(name):
                danger(f"{mcu_ref} pin {an} (GPIO{gpio}) is on net `{name}`, which carries a >3.3 V rail. "
                       "ESP32-S3 GPIOs are NOT 5 V tolerant.")
            for o in nets[name]:
                oc = cls.get(o.ref)
                if oc == "HCT245" and re.match(r"^B\d", o.func or "", re.I):
                    danger(f"{mcu_ref} pin {an} connects to {o.label(comps)}, a 74HCT245 B-side (5 V output) pin.")
                if oc == "TMC2209" and re.search(r"DIAG", o.func or "", re.I):
                    info(f"MCU pin {an} (GPIO{gpio}) <- {o.label(comps)} DIAG: level = module VIO; see VIO check below.")
            if an == "D13":
                danger(f"D13 is wired (net `{name}`) but is off-limits in this design (onboard LED / SCK).")
            if len(nets[name]) == 1:
                warn(f"MCU pin {an} is on single-node net `{name}` (dangling label?).")

    # ---- 2. MCU power input --------------------------------------------------
    pwr = [(n, nm) for n, nm in by_ref[mcu_ref]
           if re.search(r"VIN|VBUS|VUSB|^\+?5V|3V3|GND", n.func or "", re.I)]
    for n, nm in pwr:
        info(f"MCU power pin {n.func} (pin {n.pin}) -> net `{nm}` "
             f"({len(nets[nm])} nodes)")
    for n, nm in pwr:
        if re.search(r"VIN", n.func or "", re.I) and re.search(r"^/?\+?5V", nm, re.I):
            warn(f"MCU VIN is fed from `{nm}`. Nano ESP32 VIN goes through the onboard buck and is "
                 "specified for a higher input range (6-21 V per ABX00083, verify in the datasheet); "
                 "5 V may brown out under WiFi/USB load. 12 V into VIN would be inside that range.")
    if not any(re.search(r"VIN", n.func or "", re.I) and not nm.lower().startswith("unconnected")
               for n, nm in pwr):
        warn("MCU VIN is not connected. If the board is fed through the 5V/VBUS pin from the "
             "converter, that back-feeds the USB VBUS rail -- confirm with ABX00083 datasheet "
             "before powering from USB and the converter at the same time.")

    # ---- 3. TMC2209 checks ---------------------------------------------------
    for ref in sorted(r for r, k in cls.items() if k == "TMC2209"):
        pins = {n.func.upper(): (n, nm) for n, nm in by_ref[ref] if n.func}
        def pnet(rx):
            for f, (n, nm) in pins.items():
                if re.search(rx, f, re.I):
                    return nm
            return None
        vio = pnet(r"^(VIO|VDD|VCC_IO|IOREF)$")
        vm = pnet(r"^(VM|VS|VMOT|VMOTOR)$")
        if vio is None:
            warn(f"{ref}: no VIO/VDD pin found by name -- check symbol pin names.")
        elif net_is_high(vio) and not RAIL_3V3.search(vio):
            danger(f"{ref}: VIO is on `{vio}`. The BTT module pulls PDN_UART and DIAG up to VIO, so "
                   "those lines will idle at that voltage straight into ESP32-S3 GPIOs. VIO must be 3.3 V.")
        else:
            info(f"{ref}: VIO on `{vio}`, VM on `{vm}`.")
        ms = []
        for bit in ("MS1", "MS2"):
            nm = pnet(rf"^{bit}$")
            ms.append((bit, nm, net_level(nm) if nm else "UNKNOWN"))
        if all(l in ("LOW", "HIGH") for _, _, l in ms):
            addr = (1 if ms[0][2] == "HIGH" else 0) | (2 if ms[1][2] == "HIGH" else 0)
            info(f"{ref}: MS1=`{ms[0][1]}`({ms[0][2]}) MS2=`{ms[1][1]}`({ms[1][2]}) -> UART address {addr}")
        else:
            warn(f"{ref}: MS1/MS2 tie-off not decodable from net names {ms}. "
                 "Note: BTT V1.3 has onboard pull-downs on MS1/MS2, so a floating pin reads LOW -- "
                 "verify with the module schematic before assuming.")
        uart = pnet(r"PDN|UART|PDN_UART|^RX$|^TX$")
        info(f"{ref}: PDN_UART net = `{uart}`")
        en = pnet(r"^(~\{EN\}|~\{ENN\}|~EN|/EN|EN|ENN|EN_N)$")
        info(f"{ref}: EN net = `{en}`")

    # ---- 4. 74HCT245 checks --------------------------------------------------
    for ref in sorted(r for r, k in cls.items() if k == "HCT245"):
        for n, nm in by_ref[ref]:
            f = (n.func or "").upper()
            if f in ("DIR", "A/B", "AB", "A->B", "A-B", "T/R"):
                lvl = net_level(nm)
                (info if lvl == "HIGH" else danger)(
                    f"{ref}: DIR on `{nm}` ({lvl}) -- must be HIGH for A->B.")
            if re.match(r"^~?\{?(OE|CE|G)\}?$|^(OE|CE)#$|^/(OE|CE)$", f):
                lvl = net_level(nm)
                (info if lvl == "LOW" else danger)(
                    f"{ref}: OE on `{nm}` ({lvl}) -- must be LOW (outputs enabled).")
            if f in ("VCC",):
                info(f"{ref}: VCC on `{nm}` (HCT needs 5 V; A-side VIH = 2.0 V so 3.3 V drive is fine).")
            if re.match(r"^A\d$", f) and len(nets[nm]) > 1 and \
                    not any(o.ref == mcu_ref for o in nets[nm]) and net_level(nm) == "UNKNOWN":
                info(f"{ref}: input {f} on `{nm}` has no MCU driver.")
            if re.match(r"^A\d$", f) and nm.lower().startswith("unconnected"):
                warn(f"{ref}: input {f} is floating (CMOS input -- tie unused A inputs to GND).")

    # ---- 5. Common ground ----------------------------------------------------
    gnd_nets = defaultdict(set)
    for name, nodes in nets.items():
        for n in nodes:
            if re.search(r"GND|VSS", n.func or "", re.I) or n.ptype == "power_in" and GND_NET.search(name):
                gnd_nets[name].add(n.ref)
    real = {k: v for k, v in gnd_nets.items() if not k.lower().startswith("unconnected")}
    for k in list(real):
        if not GND_NET.search(k):
            danger(f"Ground pin(s) of {sorted(real[k])} sit on net `{k}`, which is not a ground net "
                   "(rail shorted to GND, or a mislabelled net).")
            del real[k]
    if len(real) > 1:
        danger("Ground pins are split across multiple nets: " +
               "; ".join(f"`{k}`: {sorted(v)}" for k, v in real.items()) +
               " -- every logic signal needs a shared return.")
    elif real:
        k, v = next(iter(real.items()))
        info(f"Common ground net `{k}` joins: {', '.join(sorted(v))}")
    floating_gnd = [k for k in gnd_nets if k.lower().startswith("unconnected")]
    for k in floating_gnd:
        warn(f"Ground pin left unconnected: `{k}`")

    # ---- 6. Nets with no MCU consumer ---------------------------------------
    orphan = []
    for name, nodes in nets.items():
        if name.lower().startswith("unconnected") or GND_NET.search(name) or \
                RAIL_3V3.search(name) or HIGH_RAIL.search(name):
            continue
        if not any(n.ref == mcu_ref for n in nodes):
            orphan.append((name, nodes))
    out.append("\n## Signal nets with no MCU pin\n")
    out.append("| Net | Nodes |\n|---|---|")
    for name, nodes in sorted(orphan):
        out.append(f"| `{name}` | " + "<br>".join(n.label(comps) for n in nodes) + " |")

    # ---- Findings -------------------------------------------------------------
    order = {"DANGER": 0, "WARN": 1, "INFO": 2}
    out.append("\n## Electrical findings\n")
    for sev, msg in sorted(findings, key=lambda t: order[t[0]]):
        out.append(f"- **{sev}** {msg}")
    n_danger = sum(1 for s, _ in findings if s == "DANGER")
    return "\n".join(out), n_danger


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("netlist")
    ap.add_argument("--mcu", help="reference designator of the Nano ESP32 (auto-detected)")
    ap.add_argument("--md", help="also write the report to this Markdown file")
    a = ap.parse_args()
    comps, nets = load(a.netlist)
    report, n_danger = audit(comps, nets, a.mcu)
    print(report)
    if a.md:
        with open(a.md, "w", encoding="utf-8") as f:
            f.write(f"# Netlist audit: {a.netlist}\n\n{report}\n")
    sys.exit(2 if n_danger else 0)


if __name__ == "__main__":
    main()
