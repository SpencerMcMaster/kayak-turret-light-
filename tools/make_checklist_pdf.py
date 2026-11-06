#!/usr/bin/env python3
"""
Build a printable PDF of BRINGUP_CHECKLIST.md.

    python3 tools/make_checklist_pdf.py            # -> BRINGUP_CHECKLIST.pdf

Pipeline: pandoc (GitHub Markdown -> HTML) -> BeautifulSoup (adds print-only
features) -> Chromium via Playwright (tools/print_pdf.js) -> US Letter PDF.

Print-only additions (the Markdown file is not changed):
  * a fill-in header block (date, tester, PCB/modules, firmware commit)
  * a tick-box column on every check table in the Stage sections and on
    the open-items table, and a "Reading / notes" column on tables that
    have a pass/expected value to record
  * tick-boxes on the numbered steps in the Stage sections
  * the "done" criteria as a tick list with a sign-off line, placed at the
    end of the procedure (after Stage 8), before the reference sections
  * page numbers, rows never split across pages, reference sections start
    on a new page

Needs: pandoc, python3 + bs4, node + playwright (preinstalled Chromium).
"""
import datetime
import os
import subprocess
import sys
import tempfile

from bs4 import BeautifulSoup

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(ROOT, "BRINGUP_CHECKLIST.md")
OUT = os.path.join(ROOT, "BRINGUP_CHECKLIST.pdf")
PRINTER = os.path.join(ROOT, "tools", "print_pdf.js")

CSS = r"""
@page { size: Letter; margin: 0.55in 0.55in 0.6in 0.55in; }
:root { --ink: #111; --muted: #555; --rule: #9a9a9a; --soft: #f1f1f1; --warn: #fff4d6; }
html { font-family: "Inter", "DejaVu Sans", sans-serif; font-size: 9.2pt; color: var(--ink); line-height: 1.38; }
body { margin: 0; }
h1 { font-size: 18pt; margin: 0 0 4pt; letter-spacing: -0.01em; }
h2 { font-size: 12.2pt; margin: 16pt 0 5pt; padding-bottom: 2pt; border-bottom: 1.4pt solid var(--ink);
     break-after: avoid; page-break-after: avoid; }
h2 + * { break-before: avoid; }
p, ul, ol { margin: 4pt 0 6pt; orphans: 3; widows: 3; }
li { margin: 1.5pt 0; }
hr { display: none; }
a { color: inherit; text-decoration: underline; text-decoration-color: #999; }
code { font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; font-size: 0.86em;
       background: var(--soft); padding: 0.5pt 2.5pt; border-radius: 2pt; overflow-wrap: anywhere; }
pre { font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; font-size: 7.4pt; background: var(--soft);
      border: 0.6pt solid var(--rule); border-radius: 3pt; padding: 5pt 7pt; white-space: pre-wrap;
      overflow-wrap: anywhere; break-inside: avoid; margin: 5pt 0 7pt; }
pre code { background: none; padding: 0; font-size: inherit; }
strong { font-weight: 650; }

table { width: 100%; border-collapse: collapse; margin: 5pt 0 9pt; font-size: 8.5pt; }
thead { display: table-header-group; }
tr { break-inside: avoid; page-break-inside: avoid; }
th { text-align: left; font-weight: 650; background: #e4e4e4; border: 0.6pt solid var(--rule); padding: 3pt 5pt; }
td { border: 0.6pt solid var(--rule); padding: 3.5pt 5pt; vertical-align: top; }
td.tick, th.tick { width: 15pt; text-align: center; padding: 3pt 2pt; }
td.reading, th.reading { width: 1.15in; }
.box { display: inline-block; width: 9pt; height: 9pt; border: 1.1pt solid var(--ink); border-radius: 1.2pt;
       vertical-align: -1.5pt; background: #fff; }

ol.steps { list-style: none; padding-left: 0; }
ol.steps > li { position: relative; padding-left: 34pt; margin: 3.5pt 0; break-inside: avoid; }
ol.steps > li .box { position: absolute; left: 0; top: 1.5pt; }
ol.steps > li .num { position: absolute; left: 15pt; top: 0; font-weight: 650; }

.fillin { width: 100%; border-collapse: collapse; margin: 8pt 0 10pt; font-size: 9pt; }
.fillin td { border: 0.8pt solid var(--ink); padding: 6pt 6pt 13pt; width: 25%; }
.fillin td .lbl { display: block; font-size: 7pt; text-transform: uppercase; letter-spacing: 0.06em; color: var(--muted); }
.fillin td .pre { font-family: "JetBrains Mono", monospace; font-size: 8.5pt; }

.warnline { background: var(--warn); border: 0.8pt solid #c9a227; border-radius: 3pt; padding: 5pt 8pt; }
.newpage { break-before: page; page-break-before: always; }
.legend { color: var(--muted); font-size: 8pt; margin-top: -2pt; }
/* reference sections: a little smaller so they fit one page */
p.ref, ul.ref, ol.ref, table.ref { font-size: 8.3pt; }
p.ref, ul.ref, ol.ref { margin: 3pt 0 4.5pt; }
h2.ref { font-size: 11.2pt; margin-top: 11pt; }
ul.done { list-style: none; padding-left: 0; }
ul.done > li { position: relative; padding-left: 16pt; margin: 3.5pt 0; }
ul.done > li .box { position: absolute; left: 0; top: 1.5pt; }
.signoff td { padding-bottom: 18pt; }
"""

FOOTER = (
    '<div style="width:100%;font-family:Inter,DejaVu Sans,sans-serif;font-size:7pt;color:#666;'
    'padding:0 0.55in;display:flex;justify-content:space-between;">'
    '<span>Kayak LED Turret: Bring-Up Checklist · firmware {commit} · printed {date}</span>'
    '<span>Page <span class="pageNumber"></span> of <span class="totalPages"></span></span></div>'
)


def git_commit():
    try:
        h = subprocess.check_output(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], text=True).strip()
        dirty = subprocess.call(["git", "-C", ROOT, "diff", "--quiet", "HEAD", "--", "kayak_working"]) != 0
        return h + ("+local edits" if dirty else "")
    except Exception:
        return "unknown"


def new(soup, tag, cls=None, text=None):
    t = soup.new_tag(tag)
    if cls:
        t["class"] = cls
    if text is not None:
        t.string = text
    return t


def box(soup):
    return new(soup, "span", "box")


def add_columns(soup, table, reading):
    """Prepend a tick-box column; optionally append a blank Reading/notes column."""
    head = table.find("thead")
    if head:
        for tr in head.find_all("tr"):
            tr.insert(0, new(soup, "th", "tick", "✓"))
            if reading:
                tr.append(new(soup, "th", "reading", "Reading / notes"))
    for tr in table.find("tbody").find_all("tr"):
        td = new(soup, "td", "tick")
        td.append(box(soup))
        tr.insert(0, td)
        if reading:
            tr.append(new(soup, "td", "reading"))


def main():
    commit = git_commit()
    today = datetime.date.today().isoformat()
    html = subprocess.check_output(
        ["pandoc", "-f", "gfm", "-t", "html5", SRC], text=True)
    soup = BeautifulSoup(html, "html.parser")

    # ---- header fill-in block after the intro paragraph ----
    h1 = soup.find("h1")
    intro = h1.find_next_sibling("p")
    fill = new(soup, "table", "fillin")
    rows = [
        [("Date", ""), ("Tester", ""), ("PCB rev / serial", "rev 2.0 /"), ("Firmware commit", commit)],
        [("Nano ESP32 ID", ""), ("TMC module A2 (lift)", "R10:"), ("TMC module A3 (pan)", "R10:"), ("12 V supply", "")],
    ]
    for r in rows:
        tr = new(soup, "tr")
        for label, pre in r:
            td = new(soup, "td")
            td.append(new(soup, "span", "lbl", label))
            if pre:
                td.append(new(soup, "span", "pre", pre))
            tr.append(td)
        fill.append(tr)
    intro.insert_after(fill)
    legend = new(soup, "p", "legend",
                 "Tick each box when the check passes. Write DMM readings and anything unexpected in "
                 "Reading / notes. Stop at the first failure.")
    fill.insert_after(legend)

    # ---- highlight the motor hot-plug warning ----
    for p in soup.find_all("p"):
        if p.get_text().startswith("Never plug or unplug a stepper motor"):
            p["class"] = "warnline"

    # ---- walk sections ----
    section = None
    for el in soup.find_all(["h2", "table", "ol"]):
        if el.name == "h2":
            section = el.get_text()
            if section.startswith("Open item 1"):
                el["class"] = "newpage"
            continue
        if section is None:
            continue
        if el.name == "table":
            first = el.find("th").get_text().strip() if el.find("th") else ""
            headers = [th.get_text().strip() for th in el.find_all("th")]
            if section.startswith("Open items"):
                add_columns(soup, el, reading=False)
                el.find("th", class_="tick").string = "Closed"
            elif section.startswith("Stage"):
                reading = any(h in ("Pass",) for h in headers) and first != "Nano pin"
                add_columns(soup, el, reading=reading)
        elif el.name == "ol" and section.startswith("Stage"):
            el["class"] = "steps"
            for i, li in enumerate(el.find_all("li", recursive=False), 1):
                li.insert(0, new(soup, "span", "num", f"{i}."))
                li.insert(0, box(soup))

    # ---- reference sections (Open item 1 .. Board revision items): smaller ----
    ref_start = next(h for h in soup.find_all("h2") if h.get_text().startswith("Open item 1"))
    done_h2 = next(h for h in soup.find_all("h2") if h.get_text().startswith("What \"done\""))
    for el in [ref_start] + ref_start.find_next_siblings():
        if el is done_h2:
            break
        if getattr(el, "name", None) in ("p", "ul", "ol", "table", "h2"):
            cls = el.get("class") or []
            el["class"] = ([cls] if isinstance(cls, str) else list(cls)) + ["ref"]

    # ---- "done" criteria as a tick list + sign-off ----
    done_ul = done_h2.find_next_sibling("ul")
    if done_ul is not None:
        done_ul["class"] = "done"
        for li in done_ul.find_all("li", recursive=False):
            li.insert(0, box(soup))
        sign = new(soup, "table", ["fillin", "signoff"])
        tr = new(soup, "tr")
        for label in ("Bring-up complete: name", "Signature", "Date"):
            td = new(soup, "td")
            td.append(new(soup, "span", "lbl", label))
            tr.append(td)
        sign.append(tr)
        done_ul.insert_after(sign)

    # Sign-off ends the procedure: move "What done looks like" (+ sign-off)
    # to just before the reference sections, after Stage 8.
    block = [done_h2]
    for sib in done_h2.find_next_siblings():
        block.append(sib)
    for el in block:
        ref_start.insert_before(el.extract())

    doc = (
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<title>Kayak LED Turret: Bring-Up Checklist</title>"
        f"<style>{CSS}</style></head><body>{soup}</body></html>"
    )

    with tempfile.TemporaryDirectory() as tmp:
        page = os.path.join(tmp, "checklist.html")
        with open(page, "w", encoding="utf-8") as f:
            f.write(doc)
        footer = FOOTER.format(commit=commit, date=today)
        subprocess.check_call(["node", PRINTER, page, OUT, footer])
    print(f"wrote {os.path.relpath(OUT, ROOT)} (firmware {commit})")


if __name__ == "__main__":
    sys.exit(main())
