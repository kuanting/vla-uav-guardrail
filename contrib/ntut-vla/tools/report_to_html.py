"""
Render the midterm report Markdown to print-styled HTML.

Why HTML and not python-docx
-----------------------------
Word is installed on this machine and opens HTML natively, preserving headings,
tables, lists and page breaks, so `report.md -> report.html -> Word COM ->
.docx / .pdf` produces both required formats with no additional Python
dependency. python-docx and a Markdown library would each be a new download for
a job the installed toolchain already does.

Scope
-----
This handles the Markdown subset the report actually uses, and nothing else:
ATX headings, pipe tables, fenced code, unordered lists, horizontal rules,
paragraphs, and inline bold / italic / code. It is deliberately not a general
Markdown implementation - an unsupported construct should look wrong
immediately rather than be silently half-rendered.

Usage:
    python tools/report_to_html.py docs/MIDTERM-REPORT-Aug2026.md
"""
from __future__ import annotations

import argparse
import html
import re
import sys
from pathlib import Path

CSS = """
@page { size: A4; margin: 22mm 20mm; }
/* The background is set explicitly. Without it the page inherits whatever the
   viewer paints behind it, and a dark-themed browser renders this dark ink on
   a dark ground. Word and the PDF export read it correctly either way, but the
   HTML is also opened directly for review. */
html { background: #FFFFFF; }
body {
  font-family: "Aptos", "Calibri", "Segoe UI", sans-serif;
  font-size: 10.5pt; line-height: 1.5;
  color: #1A1A1A; background: #FFFFFF;
  max-width: 170mm; margin: 0 auto; padding: 6mm 0;
}
h1 { font-size: 20pt; color: #1A7484; margin: 0 0 2mm 0; line-height: 1.25; }
h2 {
  font-size: 14pt; color: #1A7484; margin: 8mm 0 2mm 0;
  padding-bottom: 1.5mm; border-bottom: 1.2pt solid #249DB2;
}
h3 { font-size: 11.5pt; color: #2B2B2B; margin: 5mm 0 1.5mm 0; }
h4 { font-size: 10.5pt; color: #2B2B2B; margin: 4mm 0 1mm 0; }
p  { margin: 0 0 2.5mm 0; text-align: justify; }
ul, ol { margin: 0 0 3mm 0; padding-left: 6mm; }
li { margin-bottom: 1.2mm; }
hr { border: none; border-top: 0.75pt solid #E3E8EC; margin: 6mm 0; }
table {
  border-collapse: collapse; width: 100%;
  margin: 2.5mm 0 4mm 0; font-size: 9.5pt;
}
th {
  background: #249DB2; color: #FFFFFF; font-weight: 600;
  text-align: left; padding: 1.8mm 2.2mm; border: 0.5pt solid #249DB2;
}
td { padding: 1.6mm 2.2mm; border: 0.5pt solid #E3E8EC; vertical-align: top; }
tr:nth-child(even) td { background: #F4FAFB; }
code {
  font-family: "Consolas", "Courier New", monospace;
  font-size: 9pt; background: #F4FAFB; padding: 0.3mm 1mm;
  border: 0.5pt solid #E3E8EC;
}
pre {
  font-family: "Consolas", "Courier New", monospace;
  font-size: 9pt; background: #F4FAFB; color: #1A1A1A;
  border: 0.5pt solid #E3E8EC; border-left: 2pt solid #249DB2;
  padding: 2.5mm 3mm; margin: 2.5mm 0 4mm 0;
  white-space: pre; overflow-x: auto; line-height: 1.35;
}
pre code { background: none; border: none; padding: 0; font-size: inherit; }
blockquote {
  margin: 2.5mm 0; padding: 2mm 3mm;
  background: #F4FAFB; border-left: 2pt solid #249DB2; color: #2B2B2B;
}
blockquote p { margin: 0; }
strong { font-weight: 600; }
.subtitle { font-size: 12.5pt; color: #2B2B2B; margin: 0 0 5mm 0; font-weight: 400; }
"""


def inline(s: str) -> str:
    """Escape, then re-introduce the inline markup the report uses."""
    s = html.escape(s, quote=False)
    # Code first: its contents must not be reinterpreted as emphasis.
    holds: list[str] = []

    def stash(m: re.Match) -> str:
        holds.append(f"<code>{m.group(1)}</code>")
        return f"\x00{len(holds) - 1}\x00"

    s = re.sub(r"`([^`]+)`", stash, s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
    s = re.sub(r"(?<![\w*])\*([^*\n]+)\*(?![\w*])", r"<em>\1</em>", s)
    s = re.sub(r"\x00(\d+)\x00", lambda m: holds[int(m.group(1))], s)
    return s


def is_table_sep(line: str) -> bool:
    return bool(re.fullmatch(r"\s*\|?[\s:|-]+\|[\s:|-]*", line)) and "-" in line


def cells(line: str) -> list[str]:
    line = line.strip()
    if line.startswith("|"):
        line = line[1:]
    if line.endswith("|"):
        line = line[:-1]
    return [c.strip() for c in line.split("|")]


def render(md: str) -> str:
    lines = md.split("\n")
    out: list[str] = []
    i = 0
    in_list = False

    def close_list() -> None:
        nonlocal in_list
        if in_list:
            out.append("</ul>")
            in_list = False

    while i < len(lines):
        line = lines[i]

        # Fenced code
        if line.strip().startswith("```"):
            close_list()
            i += 1
            buf: list[str] = []
            while i < len(lines) and not lines[i].strip().startswith("```"):
                buf.append(html.escape(lines[i], quote=False))
                i += 1
            i += 1
            out.append("<pre><code>" + "\n".join(buf) + "</code></pre>")
            continue

        # Table: a header row followed by a separator row
        if "|" in line and i + 1 < len(lines) and is_table_sep(lines[i + 1]):
            close_list()
            head = cells(line)
            i += 2
            body: list[list[str]] = []
            while i < len(lines) and "|" in lines[i] and lines[i].strip():
                body.append(cells(lines[i]))
                i += 1
            out.append("<table><thead><tr>"
                       + "".join(f"<th>{inline(c)}</th>" for c in head)
                       + "</tr></thead><tbody>")
            for row in body:
                out.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in row) + "</tr>")
            out.append("</tbody></table>")
            continue

        stripped = line.strip()

        if not stripped:
            close_list()
            i += 1
            continue

        if re.fullmatch(r"-{3,}", stripped):
            close_list()
            out.append("<hr>")
            i += 1
            continue

        m = re.match(r"^(#{1,4})\s+(.*)$", stripped)
        if m:
            close_list()
            lvl = len(m.group(1))
            out.append(f"<h{lvl}>{inline(m.group(2))}</h{lvl}>")
            i += 1
            continue

        if stripped.startswith("> "):
            close_list()
            buf = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                buf.append(lines[i].strip().lstrip(">").strip())
                i += 1
            out.append(f"<blockquote><p>{inline(' '.join(buf))}</p></blockquote>")
            continue

        m = re.match(r"^[-*]\s+(.*)$", stripped)
        if m:
            if not in_list:
                out.append("<ul>")
                in_list = True
            out.append(f"<li>{inline(m.group(1))}</li>")
            i += 1
            continue

        # Numbered list items are rendered as their own paragraphs: the report
        # uses them as labelled sections rather than as a tight list.
        m = re.match(r"^(\d+)\.\s+(.*)$", stripped)
        if m:
            close_list()
            out.append(f"<p><strong>{m.group(1)}.</strong> {inline(m.group(2))}</p>")
            i += 1
            continue

        # Paragraph: join continuation lines until a blank or a block starter.
        close_list()
        buf = [stripped]
        i += 1
        while i < len(lines):
            nxt = lines[i].strip()
            if (not nxt or nxt.startswith(("#", "-", "*", ">", "|", "```"))
                    or re.match(r"^\d+\.\s", nxt)):
                break
            buf.append(nxt)
            i += 1
        out.append(f"<p>{inline(' '.join(buf))}</p>")

    close_list()
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("source", help="the report .md")
    ap.add_argument("--out", default=None, help="output .html (default: alongside)")
    args = ap.parse_args()

    src = Path(args.source)
    md = src.read_text(encoding="utf-8")
    body = render(md)

    title = "Midterm Report"
    m = re.search(r"^#\s+(.*)$", md, re.M)
    if m:
        title = m.group(1).strip()

    doc = (
        "<!DOCTYPE html>\n<html><head><meta charset=\"utf-8\">\n"
        f"<title>{html.escape(title)}</title>\n"
        f"<style>{CSS}</style>\n</head><body>\n{body}\n</body></html>\n"
    )

    dest = Path(args.out) if args.out else src.with_suffix(".html")
    dest.write_text(doc, encoding="utf-8")

    print(f"written: {dest}  ({dest.stat().st_size / 1024:.1f} KB)")
    print(f"tables : {body.count('<table>')}")
    print(f"h2     : {body.count('<h2>')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
