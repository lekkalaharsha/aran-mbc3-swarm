#!/usr/bin/env python3
"""Generate IEEE-formatted submission PDFs for all 4 MBC-3 competition documents."""

import os
from fpdf import FPDF

BASE = os.path.dirname(os.path.abspath(__file__))

DOCS = [
    ("doc1_proposal.md",       "MBC3_Doc1_Proposal.pdf",         500),
    ("doc2_products_tech.md",  "MBC3_Doc2_Products_Tech.pdf",    300),
    ("doc3_competitions.md",   "MBC3_Doc3_Competitions.pdf",     300),
    ("doc4_additional.md",     "MBC3_Doc4_Additional.pdf",       300),
]

ORG_LINE   = "Aran Technologies | aranrobotics@gmail.com | +91 72888 40612"
CONF_LINE  = "IAF Mehar Baba Competition-3 -- Phase I Submission"
FOOTER_TEXT = "Aran Technologies -- MBC-3 Submission | Phase I: New Delhi, 13-24 July 2026"

ROMAN = [(1000,'M'),(900,'CM'),(500,'D'),(400,'CD'),(100,'C'),(90,'XC'),
         (50,'L'),(40,'XL'),(10,'X'),(9,'IX'),(5,'V'),(4,'IV'),(1,'I')]

def to_roman(n):
    r = ''
    for v, s in ROMAN:
        while n >= v:
            r += s; n -= v
    return r


class DocPDF(FPDF):
    def header(self):
        self.set_font("Times", "", 8)
        self.set_fill_color(245, 245, 245)
        self.cell(0, 4, CONF_LINE, 0, new_x="LMARGIN", new_y="NEXT", align="C", fill=True)
        self.set_draw_color(150, 150, 150)
        self.line(self.l_margin, self.get_y(), self.w - self.r_margin, self.get_y())
        self.ln(2)

    def footer(self):
        self.set_y(-13)
        self.set_draw_color(150, 150, 150)
        self.line(self.l_margin, self.get_y(), self.w - self.r_margin, self.get_y())
        self.ln(1)
        self.set_font("Times", "I", 8)
        self.cell(0, 4, f"Page {self.page_no()} | {FOOTER_TEXT}", 0, new_x="RIGHT", new_y="TOP", align="C")


def sanitize(text):
    return (text
        .replace('—', '--').replace('–', '-')
        .replace('‘', "'").replace('’', "'")
        .replace('“', '"').replace('”', '"')
        .replace('•', '*').replace('°', 'deg')
        .replace('±', '+/-').replace('×', 'x')
        .replace('≤', '<=').replace('≥', '>=')
        .replace('→', '->').replace('←', '<-')
        .replace(' ', ' ').replace('§', 'S.')
    )


def md_to_pdf(md_path, out_path, word_limit):
    pdf = DocPDF(format="A4")
    # IEEE-style margins: 19mm top, 43mm bottom (for footer), 19mm sides
    pdf.set_margins(19, 19, 19)
    pdf.set_auto_page_break(auto=True, margin=14)
    pdf.add_page()

    with open(md_path, "r") as f:
        lines = f.readlines()

    section_n = 0
    subsection_n = 0
    title_done = False

    for line in lines:
        line = sanitize(line.rstrip())

        if line.startswith("*Word count"):
            continue

        # H1 — IEEE title block: centered, large bold + org line
        if line.startswith("# "):
            title = line[2:]
            pdf.set_font("Times", "B", 16)
            pdf.cell(0, 7, title, new_x="LMARGIN", new_y="NEXT", align="C")
            pdf.set_font("Times", "I", 10)
            pdf.cell(0, 5, ORG_LINE, new_x="LMARGIN", new_y="NEXT", align="C")
            pdf.ln(1)
            pdf.set_draw_color(100, 100, 100)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(3)
            title_done = True

        # Skip the bold subtitle line right after H1 (word count / limit line)
        elif line.startswith("**Aran Technologies"):
            continue

        # H2 — IEEE section: "I. SECTION TITLE" bold, all caps, with rule above
        elif line.startswith("## "):
            section_n += 1
            subsection_n = 0
            heading = f"{to_roman(section_n)}. {line[3:].upper()}"
            pdf.ln(2)
            pdf.set_draw_color(180, 180, 180)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(2)
            pdf.set_font("Times", "B", 10)
            pdf.multi_cell(0, 5, heading, align="L")
            pdf.ln(1)

        # H3 — IEEE subsection: "A. Subsection Title" bold italic
        elif line.startswith("### "):
            subsection_n += 1
            sub_chr = chr(64 + subsection_n)
            heading = f"{sub_chr}. {line[4:]}"
            pdf.set_font("Times", "BI", 10)
            pdf.multi_cell(0, 5, heading, align="L")
            pdf.ln(0.5)

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
            # First row = header: bold
            is_header = (cells[0].lower() in ("sub-system", "requirement", "parameter", "phase", "sub system"))
            col_w = (pdf.w - pdf.l_margin - pdf.r_margin) / n
            pdf.set_font("Times", "B" if is_header else "", 9)
            x0, y0 = pdf.get_x(), pdf.get_y()
            max_y = y0
            for i, cell in enumerate(cells):
                pdf.set_xy(x0 + i * col_w, y0)
                pdf.multi_cell(col_w, 5, cell, border=1, align="L")
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

        # Normal paragraph — IEEE body: 10pt Times, justified
        else:
            pdf.set_font("Times", "", 10)
            text = line.replace("**", "").replace("`", "").replace("*", "")
            pdf.multi_cell(0, 5, text, align="J")

    pdf.output(out_path)
    size = os.path.getsize(out_path)
    print(f"  Saved: {os.path.basename(out_path)}  ({size // 1024} KB)")


if __name__ == "__main__":
    print("Generating IEEE-formatted MBC-3 submission PDFs...")
    for md_file, pdf_file, limit in DOCS:
        md_path  = os.path.join(BASE, md_file)
        out_path = os.path.join(BASE, pdf_file)
        print(f"  Processing {md_file}...")
        md_to_pdf(md_path, out_path, limit)
    print("Done.")
