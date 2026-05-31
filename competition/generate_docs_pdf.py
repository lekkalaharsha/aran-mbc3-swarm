#!/usr/bin/env python3
"""Generate MBC-3 submission PDFs for all 4 competition documents — matching MBC3_Proposal.pdf style."""

import os
from fpdf import FPDF

BASE = os.path.dirname(os.path.abspath(__file__))

DOCS = [
    ("doc1_proposal.md",       "MBC3_Doc1_Proposal.pdf",         500),
    ("doc2_products_tech.md",  "MBC3_Doc2_Products_Tech.pdf",    300),
    ("doc3_competitions.md",   "MBC3_Doc3_Competitions.pdf",     300),
    ("doc4_additional.md",     "MBC3_Doc4_Additional.pdf",       300),
]

ORG_LINE    = "Aran Technologies  |  aranrobotics@gmail.com  |  IAF MBC-3 Competition"
HEADER_TEXT = "IAF Mehar Baba Competition-3 -- Phase I Submission | Aran Technologies"
FOOTER_TEXT = "Aran Technologies | IAF Mehar Baba Competition-3"


class DocPDF(FPDF):
    def header(self):
        self.set_font("Times", "B", 10)
        self.set_fill_color(220, 220, 220)
        self.cell(0, 6, HEADER_TEXT, 0, new_x="LMARGIN", new_y="NEXT", align="C", fill=True)
        self.ln(2)

    def footer(self):
        self.set_y(-12)
        self.set_font("Times", "I", 8)
        self.cell(0, 5, f"Page {self.page_no()} | {FOOTER_TEXT}", 0, new_x="RIGHT", new_y="TOP", align="C")


def sanitize(text):
    return (text
        .replace('—', '--').replace('–', '-')
        .replace('‘', "'").replace('’', "'")
        .replace('“', '"').replace('”', '"')
        .replace('•', '*').replace('°', 'deg')
        .replace('±', '+/-').replace('×', 'x')
        .replace('≤', '<=').replace('≥', '>=')
        .replace('→', '->').replace('←', '<-')
        .replace(' ', ' ').replace('§', 'sec.')
    )


def md_to_pdf(md_path, out_path, word_limit):
    pdf = DocPDF(format="A4")
    pdf.set_margins(18, 18, 18)
    pdf.set_auto_page_break(auto=True, margin=10)
    pdf.add_page()

    with open(md_path, "r") as f:
        lines = f.readlines()

    section_n = 0
    subsection_n = 0

    for line in lines:
        line = sanitize(line.rstrip())

        if line.startswith("*Word count"):
            continue

        # H1 — title block: large bold centered + italic org line + rule
        if line.startswith("# "):
            title = line[2:]
            pdf.set_font("Times", "B", 16)
            pdf.multi_cell(0, 8, title, align="C")
            pdf.ln(2)
            pdf.set_font("Times", "I", 10)
            pdf.cell(0, 6, ORG_LINE, new_x="LMARGIN", new_y="NEXT", align="C")
            pdf.ln(2)
            pdf.set_draw_color(0, 0, 0)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(2)

        # Skip bold subtitle line right after H1
        elif line.startswith("**Aran Technologies"):
            continue

        # H2 — numbered section: "1. Section Title" bold
        elif line.startswith("## "):
            section_n += 1
            subsection_n = 0
            heading = f"{section_n}. {line[3:]}"
            pdf.ln(1)
            pdf.set_font("Times", "B", 10)
            pdf.cell(0, 5, heading, new_x="LMARGIN", new_y="NEXT")
            pdf.ln(0.5)

        # H3 — numbered subsection: "1.1 Subsection Title" bold inline
        elif line.startswith("### "):
            subsection_n += 1
            label = f"{section_n}.{subsection_n} {line[4:]}."
            pdf.set_font("Times", "B", 10)
            pdf.cell(0, 5, label, new_x="LMARGIN", new_y="NEXT")

        # Horizontal rule
        elif line.startswith("---"):
            pdf.set_draw_color(180, 180, 180)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(2)

        # Bullet
        elif line.startswith("- ") or line.startswith("* "):
            pdf.set_font("Times", "", 10)
            text = line[2:].replace("**", "").replace("`", "")
            pdf.multi_cell(0, 5, f"  -  {text}", align="L")

        # Table (skip separator rows)
        elif line.startswith("|"):
            if "---" in line:
                continue
            cells = [c.strip().replace("**", "").replace("`", "") for c in line.strip("|").split("|")]
            n = max(len(cells), 1)
            col_w = (pdf.w - pdf.l_margin - pdf.r_margin) / n
            # Detect header row
            is_header = any(c.lower() in ("sub-system", "requirement", "parameter", "phase",
                                          "sub system", "layer", "specification", "component",
                                          "feature", "metric", "item", "value", "detail")
                            for c in cells)
            if is_header:
                pdf.set_font("Times", "B", 9)
                pdf.set_fill_color(200, 200, 200)
                fill = True
            else:
                pdf.set_font("Times", "", 9)
                # alternating rows — track row index via a hack: use page_no parity
                fill = False
                pdf.set_fill_color(245, 245, 245)
            x0, y0 = pdf.get_x(), pdf.get_y()
            max_y = y0
            for i, cell in enumerate(cells):
                pdf.set_xy(x0 + i * col_w, y0)
                pdf.multi_cell(col_w, 5, cell, border=1, align="L", fill=fill)
                max_y = max(max_y, pdf.get_y())
            pdf.set_y(max_y)

        # Bold standalone line
        elif line.startswith("**") and line.endswith("**"):
            pdf.set_font("Times", "B", 10)
            pdf.multi_cell(0, 5, line.replace("**", ""), align="L")
            pdf.ln(0.5)

        # Blank line
        elif line == "":
            pdf.ln(1.5)

        # Normal paragraph — 10pt Times, justified
        else:
            pdf.set_font("Times", "", 10)
            text = line.replace("**", "").replace("`", "").replace("*", "")
            pdf.multi_cell(0, 5, text, align="J")

    pdf.output(out_path)
    size = os.path.getsize(out_path)
    print(f"  Saved: {os.path.basename(out_path)}  ({size // 1024} KB)")


if __name__ == "__main__":
    print("Generating MBC-3 submission PDFs...")
    for md_file, pdf_file, limit in DOCS:
        md_path  = os.path.join(BASE, md_file)
        out_path = os.path.join(BASE, pdf_file)
        print(f"  Processing {md_file}...")
        md_to_pdf(md_path, out_path, limit)
    print("Done.")
