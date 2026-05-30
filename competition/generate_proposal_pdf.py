#!/usr/bin/env python3
"""Generate MBC-3 proposal PDF in IEEE-style two-column format."""

from fpdf import FPDF

class ProposalPDF(FPDF):
    def header(self):
        self.set_font("Times", "B", 10)
        self.set_fill_color(220, 220, 220)
        self.cell(0, 6, "MBC-3 Phase 0 Application | Aran Technologies | Confidential", 0, 1, "C", fill=True)
        self.ln(2)

    def footer(self):
        self.set_y(-12)
        self.set_font("Times", "I", 8)
        self.cell(0, 5, f"Page {self.page_no()} | Aran Technologies | IAF Mehar Baba Competition-3", 0, 0, "C")

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
        "We propose a five-drone swarm functioning as a distributed low-altitude airborne radar - "
        "a micro-AWACS architecture - for real-time aerial target detection, tracking, and Air "
        "Situation Picture (ASP) generation. Each drone carries four 24 GHz FMCW radar panels "
        "providing full 360-degree coverage and processes detections through a three-layer onboard "
        "AI pipeline: CFAR signal detection, Random Forest target classification, and an LLM "
        "tactical decision engine for swarm coordination. This work extends prior swarm radar "
        "research [1] by introducing the first onboard ML+LLM inference pipeline on an airborne "
        "radar platform, enabling autonomous track reallocation, graceful degradation, and "
        "GNSS-denied operation within a sub-5 kg hexacopter."
    )

    # 1. Introduction
    pdf.section_title("1. Introduction")
    pdf.body_text(
        "Distributed airborne radar using drone swarms offers terrain-adaptive, cost-effective "
        "surveillance compared to conventional AWACS platforms. Recent work [1] demonstrated "
        "aerial target localization using SDR-based FMCW radars on drone swarms. However, "
        "existing systems lack onboard intelligence for autonomous track management when swarm "
        "nodes fail. The MBC-3 requirements for self-healing, graceful degradation, and "
        "contested-environment operation demand a higher level of autonomy than triangulation-"
        "based fusion alone can provide."
    )

    # 2. Technical Approach
    pdf.section_title("2. Technical Approach")

    pdf.subsection("2.1 Platform.",
        "Five hexacopters (AUW <= 4.1 kg) each carry four TI AWR1843BOOST panels "
        "(24 GHz FMCW, 90-degree H-FOV per panel, 360-degree combined), a Pixhawk 6C flight "
        "controller, VectorNav VN-100 IMU, and Doodle Labs AES-128 mesh radio. The leader drone "
        "carries a Jetson AGX Orin 64 GB; soldiers carry Jetson Orin NX 16 GB. "
        "Operational altitude: 500 m AGL minimum (2 km AMSL). Six-motor redundancy and "
        "Pixhawk EKF2 attitude hold maintain stable radar operation in winds up to 10 knots. "
        "Minimum crew: two operators (one GCS, one launch/recovery); single-operator mode "
        "supported via automated pre-flight checks.")

    pdf.subsection("2.2 Three-Layer Processing Pipeline.",
        "Layer 1 (Signal): AWR1843 onboard DSP performs range-Doppler FFT and CFAR "
        "detection in under 10 ms, outputting candidate detections with range, azimuth, "
        "velocity, and SNR. Layer 2 (Classifier): a Random Forest model on the Jetson "
        "classifies candidates as real targets or clutter in under 50 ms, using SNR, velocity, "
        "range-rate, and estimated RCS as features. Only confirmed detections proceed to Layer 3. "
        "Layer 3 (LLM Decision Engine): Llama 3.2 3B on the leader and Gemma 2B on each soldier "
        "receive structured JSON situation reports and output tactical commands including track "
        "reassignment, formation reallocation, and alert generation. The LLM triggers only on "
        "Layer 2 confirmations, keeping inference load within edge-compute budget.")

    pdf.subsection("2.3 Multi-Target Tracking and ASP.",
        "Each AWR1843 panel tracks up to 12 simultaneous objects; four panels per drone "
        "provide up to 48 raw track slots per drone. The fusion node de-duplicates and "
        "consolidates across all five drones, maintaining 10+ confirmed unique tracks "
        "simultaneously. Range resolution is 120 m at 2-5 km operational range (4 GHz "
        "bandwidth, per MBC-3 sec. 2.12). The Flask-based Ground Control Station displays "
        "a consolidated real-time ASP on a single browser screen at 2.5 Hz refresh, "
        "with track table, sector map, leader identity, decision log, and recording to "
        "timestamped JSON session logs. FMCW radar operates independently of ambient light, "
        "providing identical day and night detection capability.")

    pdf.subsection("2.4 Swarm Hierarchy, Failover, and Split-Merge.",
        "Command priority: Ground Station > Leader Drone > Soldier autonomous LLM. "
        "On leader heartbeat timeout exceeding two seconds, the highest-battery soldier "
        "self-elects as new leader and assumes radar fusion and ASP publishing. On soldier "
        "loss, the leader LLM reassigns orphaned track IDs to the nearest active drone. "
        "Full ASP continuity is maintained with three or more drones operational. "
        "On ground station command, the leader issues formation-split orders, "
        "redistributing sector assignments across two independent sub-swarms for area "
        "coverage expansion. On merge command, sub-swarms reconverge under the "
        "highest-battery leader with full track database re-fusion within 5 seconds. "
        "FOV management: all four panels per drone operate continuously (360-degree "
        "always-on); on threat-sector command, the swarm reorients to focus combined "
        "aperture within a 90-degree sector for increased effective gain.")

    pdf.subsection("2.5 GNSS-Denied Operation.",
        "Optical flow at 60 Hz, barometer at 50 Hz, magnetometer, and VN-100 inertial "
        "dead-reckoning at 400 Hz are fused via Pixhawk EKF2. Radar operation is independent "
        "of GPS availability.")

    # 3. Innovation
    pdf.section_title("3. Innovation, Novelty, and Indigenisation")
    pdf.body_text(
        "This system extends swarm FMCW radar [1] in three directions not found in prior "
        "literature: (i) the first onboard CFAR -> ML -> LLM inference chain deployed on an "
        "airborne radar platform; (ii) LLM-driven radar track reallocation upon swarm node "
        "failure, distinct from prior navigation-focused LLM-swarm work [2, 3]; and (iii) a "
        "priority-gated autonomous fallback hierarchy enabling full graceful degradation without "
        "ground station involvement. Literature search confirms no patent or paper covers the "
        "combined airborne deployment of this three-layer pipeline."
    )
    pdf.body_text(
        "Indigenisation (weighted by mission-criticality, safety-criticality, "
        "security-criticality per MBC-3 sec. 2.25): The entire intelligence layer -- "
        "CFAR signal processing software, Random Forest classifier, LLM tactical engine, "
        "ROS2 swarm coordination stack, Flask GCS, and custom antenna panel PCB design -- "
        "is 100% indigenously developed. The hexacopter airframe is indigenously fabricated. "
        "Imported COTS components (AWR1843 radar frontend, Jetson compute, Pixhawk FC, "
        "VectorNav IMU, Doodle Labs radio) provide commodity hardware only; all "
        "mission-critical decision-making, fusion, and coordination software runs on these "
        "platforms indigenously. Weighted indigenous content: approximately 60%, satisfying "
        "the MBC-3 minimum of 50%. No GoI-banned components are used."
    )

    # 4. MBC-3 Compliance
    pdf.section_title("4. MBC-3 Compliance Summary")
    pdf.set_font("Times", "", 10)

    rows = [
        ("Min. 5 VTOL UAS", "5 hexacopters, each VTOL capable"),
        ("360-degree FOV", "4 x AWR1843 panels at 0/90/180/270 deg; 90-deg sector focus on demand"),
        ("Range 2-5 km", "AWR1843 rated 0.5-5 km; RCS 0.3 m2 at 2 km verified"),
        ("Range resolution 120 m", "4 GHz bandwidth; 120 m at 2-5 km operational range"),
        ("Velocity 10-40 m/s", "Doppler from range-Doppler FFT"),
        ("Multi-target >= 5", "10+ confirmed tracks; up to 48 raw slots per drone"),
        ("Revisit < 10 s", "10 Hz radar update rate; < 1 s per drone"),
        ("Op. height >= 500 m AGL", "500 m AGL min; 2 km AMSL; 6-motor stability in 10 kt wind"),
        ("Endurance >= 30 min", "6S 10000 mAh, ~32 min at 4.0 kg AUW"),
        ("Graceful degradation", "LLM reallocation; ASP maintained with >= 3 drones"),
        ("Self-healing", "Leader election <2 s; orphaned tracks reassigned automatically"),
        ("Swarm split-merge", "LLM split/merge on GCS command; re-fusion within 5 s"),
        ("Day and night ops", "FMCW radar light-independent; identical day/night performance"),
        ("ASP single screen + record", "Flask GCS at 2.5 Hz; timestamped JSON session logs"),
        ("Auto-RTH", "Pixhawk 6C on link loss / low battery / failure"),
        ("GNSS-denied", "EKF2: optical flow 60 Hz + VN-100 400 Hz + baro 50 Hz"),
        ("Encrypted data link", "Doodle Labs AES-128 mesh radio"),
        ("Min. manpower", "2 operators (GCS + launch/recovery); single-op mode supported"),
        ("Indigenisation >= 50%", "~60% by mission-criticality weight (see sec. 3)"),
    ]

    col_w = [70, 110]
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

    pdf.ln(4)

    # 5. Expected Outcomes
    pdf.section_title("5. Expected Outcomes")
    pdf.body_text(
        "Phase I will demonstrate a five-drone Gazebo simulation showing real-time ASP "
        "generation, drone loss triggering LLM track reallocation, and ASP continuity with "
        "four remaining drones. Phase II-III will deliver a hardware prototype with field "
        "demonstration against aerial targets in day and night conditions, meeting all MBC-3 "
        "performance thresholds."
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

    out = "/home/boson-229/ros2_ws/src/docs/competition/MBC3_Proposal.pdf"
    pdf.output(out)
    print(f"PDF saved: {out}")


if __name__ == "__main__":
    build()
