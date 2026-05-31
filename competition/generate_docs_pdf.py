#!/usr/bin/env python3
"""Generate submission PDFs for all 4 MBC-3 competition documents."""

import os
from fpdf import FPDF

BASE = os.path.dirname(os.path.abspath(__file__))

DOCS = [
    ("doc1_proposal.md",       "MBC3_Doc1_Proposal.pdf",         500),
    ("doc2_products_tech.md",  "MBC3_Doc2_Products_Tech.pdf",    300),
    ("doc3_competitions.md",   "MBC3_Doc3_Competitions.pdf",     300),
    ("doc4_additional.md",     "MBC3_Doc4_Additional.pdf",       300),
]

HEADER_TEXT = "IAF Mehar Baba Competition-3 | Aran Technologies | aranrobotics@gmail.com | Confidential"
FOOTER_TEXT = "Aran Technologies -- MBC-3 Submission | Phase I: New Delhi, 13-24 July 2026"


class DocPDF(FPDF):
    def __init__(self, word_limit):
        super().__init__(format="A4")
        self.word_limit = word_limit

    def header(self):
        self.set_font("Times", "I", 8)
        self.set_fill_color(230, 230, 230)
        self.cell(0, 5, HEADER_TEXT, 0, 1, "C", fill=True)
        self.ln(2)

    def footer(self):
        self.set_y(-12)
        self.set_font("Times", "I", 8)
        self.cell(0, 5, f"Page {self.page_no()} | {FOOTER_TEXT}", 0, 0, "C")


def sanitize(text):
    return (text
        .replace('—', '--').replace('–', '-')
        .replace('‘', "'").replace('’', "'")
        .replace('“', '"').replace('”', '"')
        .replace('•', '*').replace('°', 'deg')
        .replace('±', '+/-').replace('×', 'x')
        .replace('≤', '<=').replace('≥', '>=')
        .replace('→', '->').replace('←', '<-')
        .replace(' ', ' ')
    )

def md_to_pdf(md_path, out_path, word_limit):
    pdf = DocPDF(word_limit)
    pdf.set_margins(20, 20, 20)
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()

    with open(md_path, "r") as f:
        lines = f.readlines()

    for line in lines:
        line = sanitize(line.rstrip())

        # Skip metadata lines
        if line.startswith("*Word count"):
            continue

        # H1
        if line.startswith("# "):
            pdf.set_font("Times", "B", 14)
            pdf.multi_cell(0, 7, line[2:])
            pdf.ln(2)
        # H2
        elif line.startswith("## "):
            pdf.set_font("Times", "B", 11)
            pdf.multi_cell(0, 6, line[3:])
            pdf.ln(1)
        # H3
        elif line.startswith("### "):
            pdf.set_font("Times", "B", 10)
            pdf.multi_cell(0, 5, line[4:])
            pdf.ln(1)
        # Horizontal rule
        elif line.startswith("---"):
            pdf.set_draw_color(180, 180, 180)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(3)
        # Bullet
        elif line.startswith("- ") or line.startswith("* "):
            pdf.set_font("Times", "", 10)
            text = line[2:].replace("**", "").replace("`", "")
            pdf.multi_cell(0, 5, f"  •  {text}")
        # Table row (skip separator lines like |---|---|)
        elif line.startswith("|"):
            if "---" in line:
                continue
            cells = [c.strip().replace("**", "").replace("`", "") for c in line.strip("|").split("|")]
            n = max(len(cells), 1)
            col_w = (pdf.w - pdf.l_margin - pdf.r_margin) / n
            pdf.set_font("Times", "", 9)
            x0, y0 = pdf.get_x(), pdf.get_y()
            max_y = y0
            for i, cell in enumerate(cells):
                pdf.set_xy(x0 + i * col_w, y0)
                pdf.multi_cell(col_w, 5, cell, border=1)
                max_y = max(max_y, pdf.get_y())
            pdf.set_y(max_y)
        # Bold header line (** **)
        elif line.startswith("**") and line.endswith("**"):
            pdf.set_font("Times", "B", 10)
            pdf.multi_cell(0, 5, line.replace("**", ""))
            pdf.ln(1)
        # Blank line
        elif line == "":
            pdf.ln(2)
        # Normal paragraph
        else:
            pdf.set_font("Times", "", 10)
            text = line.replace("**", "").replace("`", "").replace("*", "")
            pdf.multi_cell(0, 5, text)

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
