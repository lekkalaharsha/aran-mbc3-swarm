#!/usr/bin/env python3
"""Generate MBC-3 proposal PDF in IEEE-style two-column format."""

from fpdf import FPDF

class ProposalPDF(FPDF):
    def header(self):
        self.set_font("Times", "B", 10)
        self.set_fill_color(220, 220, 220)
        self.cell(0, 6, "IAF Mehar Baba Competition-3 -- Phase I Submission | Aran Technologies", 0, new_x="LMARGIN", new_y="NEXT", align="C", fill=True)
        self.ln(2)

    def footer(self):
        self.set_y(-12)
        self.set_font("Times", "I", 8)
        self.cell(0, 5, f"Page {self.page_no()} | Aran Technologies | IAF Mehar Baba Competition-3", 0, new_x="RIGHT", new_y="TOP", align="C")

    def title_block(self):
        self.set_font("Times", "B", 16)
        self.multi_cell(0, 8,
            "Collaborative FMCW Radar Swarm with Onboard AI Decision Engine\n"
            "for Aerial Surveillance",
            align="C")
        self.ln(2)
        self.set_font("Times", "I", 10)
        self.cell(0, 6, "Aran Technologies  |  aranrobotics@gmail.com  |  IAF MBC-3 Competition", align="C", new_x="LMARGIN", new_y="NEXT")
        self.ln(4)
        self.set_draw_color(0, 0, 0)
        self.line(self.l_margin, self.get_y(), self.w - self.r_margin, self.get_y())
        self.ln(4)

    def section_title(self, text):
        self.set_font("Times", "B", 10)
        self.cell(0, 5, text, new_x="LMARGIN", new_y="NEXT")
        self.ln(1)

    def body_text(self, text):
        self.set_font("Times", "", 10)
        self.multi_cell(0, 5, text)
        self.ln(2)

    def subsection(self, label, text):
        self.set_font("Times", "B", 10)
        self.write(5, label + "  ")
        self.set_font("Times", "", 10)
        self.write(5, text)
        self.ln(5)


def build():
    pdf = ProposalPDF(format="A4")
    pdf.set_margins(20, 20, 20)
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()

    pdf.title_block()

    # Abstract
    pdf.section_title("Abstract")
    pdf.body_text(
        "We present a five-drone swarm as a distributed airborne FMCW radar -- a micro-AWACS "
        "architecture -- for real-time target detection, tracking, and ASP generation. "
        "Each drone carries six AWR1843 24 GHz panels (360-deg coverage) and processes "
        "detections through CFAR, Random Forest, and LLM tactical engine. First onboard "
        "ML+LLM inference chain on an airborne radar platform [1]; all behaviours verified "
        "in simulation."
    )

    # 1. Technical Approach
    pdf.section_title("1. Technical Approach")

    pdf.subsection("1.1 Platform.",
        "Five hexacopters (AUW 4.267 kg): six AWR1843BOOST AERIS-10 panels (24 GHz, 60-deg "
        "H-FOV, 360-deg combined), indigenous STM32 FC (custom PCB, FreeRTOS, 250 Hz PID, "
        "Kalman IMU), Doodle Labs AES-128 radio. Leader: Jetson AGX Orin 64 GB; soldiers: "
        "Orin NX 16 GB. Endurance ~32 min (6S 10,000 mAh). EKF2: optical flow + IMU + baro.")

    pdf.subsection("1.2 AI Pipeline.",
        "L1: AWR1843 DSP -- range-Doppler FFT + CFAR in <10 ms (range, azimuth, velocity, SNR). "
        "L2: Random Forest on Jetson -- target vs. clutter in <50 ms. "
        "L3: Llama 3.2 3B (leader) / Gemma 2B (soldiers) -- JSON situation reports to track "
        "reassignment, sector reorientation, threat alerts; triggers on L2 confirmations only.")

    pdf.subsection("1.3 Swarm Resilience.",
        "GCS > Leader > Soldier LLM. Leader loss >2 s: highest-battery soldier self-elects, "
        "resumes ASP fusion (sim. verified). Soldier loss: LLM redistributes orphaned tracks. "
        "ASP continuity >=3 drones (344 tracks logged). Split-merge on GCS command; "
        "re-fusion <5 s (sim. verified). Flask GCS 2.5 Hz: track table, sector map, "
        "decision log, JSON recording.")

    # 2. Innovation & Indigenisation
    pdf.section_title("2. Innovation and Indigenisation")
    pdf.body_text(
        "Novel: (i) first airborne CFAR->ML->LLM chain; (ii) LLM track reallocation on "
        "node failure [2,3]; (iii) priority-gated fallback without GCS. "
        "Indigenous: AI stack, ROS2, GCS, antenna PCB, airframe, STM32 FC. "
        "Contributes to AERIS-10 (github.com/NawfalMotii79/PLFM_RADAR) -- "
        "open-source 10.5 GHz pulse-LFM radar. COTS: AWR1843, Jetson, Doodle Labs. "
        "Indigenous >=55% (MBC-3 sec. 2.25). Nirmaan pre-incubation Phase 1, IIT Madras."
    )

    # 3. MBC-3 Compliance (key rows only)
    pdf.section_title("3. MBC-3 Compliance Summary")

    rows = [
        ("360-deg FOV / Range",   "6 x AWR1843 @ 60-deg intervals; 0.5-5 km; 120 m resolution"),
        ("Multi-target / Revisit", "10+ tracks; 72 raw slots/drone; revisit <1 s (sim. verified)"),
        ("Graceful degradation",   "ASP maintained >=3 drones; bully election <2 s (sim. verified)"),
        ("Endurance / AUW",        "~32 min, 6S 10000 mAh, 4.267 kg; op. height 500 m AGL min"),
        ("Indigenisation",         ">=55% by mission-criticality weight (MBC-3 sec. 2.25)"),
        ("GNSS-denied / RTH",      "EKF2: optical flow + IMU 250 Hz + baro; auto-RTH on link loss"),
        ("Encrypted link / ASP",   "Doodle Labs AES-128; Flask GCS 2.5 Hz + JSON logs"),
    ]

    col_w = [68, 112]
    pdf.set_font("Times", "B", 9)
    pdf.set_fill_color(200, 200, 200)
    pdf.cell(col_w[0], 5, "Requirement", border=1, fill=True)
    pdf.cell(col_w[1], 5, "Implementation", border=1, fill=True, new_x="LMARGIN", new_y="NEXT")

    pdf.set_font("Times", "", 9)
    for i, (req, impl) in enumerate(rows):
        fill = (i % 2 == 0)
        pdf.set_fill_color(240, 240, 240) if fill else pdf.set_fill_color(255, 255, 255)
        pdf.cell(col_w[0], 5, req, border=1, fill=fill)
        pdf.cell(col_w[1], 5, impl, border=1, fill=fill, new_x="LMARGIN", new_y="NEXT")

    pdf.ln(3)

    # 4. Phase I Status
    pdf.section_title("4. Phase I Status")
    pdf.body_text(
        "Phase I complete: SITL simulation verified (ASP, LLM reallocation, 344 tracks, "
        "split-merge), broadband and drone-to-ground comms tested, 4-min ISR demo video "
        "ready (github.com/lekkalaharsha/aran-mbc3-swarm). "
        "Phase II-III: hardware prototype, field demonstration."
    )

    # References
    pdf.section_title("References")
    pdf.set_font("Times", "", 9)
    refs = [
        "[1] Anon., \"A Swarm of Drones for Detection and Localization of Airborne Targets,\" IEEE IGARSS, 2025.",
        "[2] X. Zhang et al., \"Multimodal Large Language Models-Enabled UAV Swarm,\" arXiv:2506.12710, 2025.",
        "[3] PMC, \"Multi-Agent Systems Powered by Large Language Models: Applications in Swarm Intelligence,\" Frontiers in AI, 2025.",
    ]
    for ref in refs:
        pdf.multi_cell(0, 5, ref)
        pdf.ln(1)

    import os
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MBC3_Proposal.pdf")
    pdf.output(out)
    print(f"PDF saved: {out}")


if __name__ == "__main__":
    build()
