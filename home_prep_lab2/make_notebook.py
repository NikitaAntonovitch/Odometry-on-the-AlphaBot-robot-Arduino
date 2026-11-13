"""Generate D5 observation notebook (observation_notebook.docx) for ISS LAB2.
Tables per the Master Prompt D5 spec; empty cells = to be measured at school.
Known values are pre-filled and tagged with their evidence class.
Requires: python-docx
"""
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH

doc = Document()
for s in doc.sections:
    s.page_width, s.page_height = Cm(21.0), Cm(29.7)  # A4
    s.left_margin = s.right_margin = Cm(1.5)
    s.top_margin = s.bottom_margin = Cm(1.5)

st = doc.styles["Normal"]
st.font.name = "Calibri"
st.font.size = Pt(11)

def h(text, size=14):
    p = doc.add_paragraph()
    r = p.add_run(text)
    r.bold = True
    r.font.size = Pt(size)
    return p

def note(text):
    p = doc.add_paragraph()
    r = p.add_run(text)
    r.italic = True
    r.font.size = Pt(9)
    r.font.color.rgb = RGBColor(0x66, 0x66, 0x66)
    return p

def table(headers, rows):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    hdr = t.rows[0].cells
    for i, htxt in enumerate(headers):
        hdr[i].text = htxt
        hdr[i].paragraphs[0].runs[0].bold = True
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = str(v)
    return t

# ---------- cover ----------
p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = p.add_run("ISS LAB 2 - PID Line-Following Robot - Observation Notebook")
r.bold = True; r.font.size = Pt(16)
p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
p.add_run("Name: ____________   Date: ____________   Instructor: ____________")
note("Fill in tables during school testing. Bring to the demo for reference. "
     "Cells marked [HW 09-11] are pre-filled from the 2026-11-09 school session "
     "(hardware-verified); [SIM] cells come from the host simulator and MUST be "
     "re-checked on the real robot; empty cells = to be measured.")

# ---------- Section 1 ----------
h("Section 1 - Stage 1: Sensor & Calibration")
note("Pre-filled polarity evidence [HW 09-11]: line reads LOWER than background "
     "(dark line on light floor). white_line = 0 chosen and verified.")
table(
    ["Sensor", "Raw over background", "Raw over line", "Cal over background", "Cal over line", "Contrast (bg - line)"],
    [["IR1 (idx0)", "~872 [HW]", "~307 [HW]", "", "", ""],
     ["IR2 (idx1)", "", "", "", "", ""],
     ["IR3 (idx2)", "", "", "", "", ""],
     ["IR4 (idx3)", "", "", "", "", ""],
     ["IR5 (idx4)", "", "", "", "", ""]])
doc.add_paragraph()
note("Weakest sensor so far: sensor 0 (smallest MIN/MAX range) [HW 09-11] - "
     "supersedes the older 'sensor 4 weakest' note from 2026-11-06. Re-measure.")
table(["Run #", "calibratedMin[0..4]", "calibratedMax[0..4]", "Repeatable? (Y/N)"],
      [["ref (saved to EEPROM)", "286,276,270,223,183 [HW]", "878,868,767,720,586 [HW]", "ranges 403-592, all >400"],
       ["2", "", "", ""],
       ["3", "", "", ""]])
note("Anti-pattern reminder [HW]: stationary calibration gives ranges 22/526/14/32/407 "
     "and linePos pinned at 2000. ALWAYS rotate the robot through the line for the full 10 s.")

# ---------- Section 2 ----------
h("Section 2 - Stage 2: Line Position")
note("Already verified [HW 09-11]: useful range +-2000, monotonic, centre linePos~2000; "
     "loop 20 ms over=0; white_line=0. Re-confirm quickly with 'p' at session start.")
table(["Line position", "error", "side (L/R)", "Correct? (Y/N)"],
      [["right of centre", ">0", "R", "[HW] yes"],
       ["left of centre", "<0", "L", "[HW] yes"],
       ["", "", "", ""],
       ["", "", "", ""]])
table(["Offset (mm)", "linePos", "error", "lost (0/1)"],
      [[f"{i}" , "", "", ""] for i in range(-40, 41, 10)])
table(["Slide direction", "Transition clean? (Y/N)", "Flips at boundary? (Y/N)", "Action needed"],
      [["L->R", "", "", ""], ["R->L", "", "", ""], ["off left edge", "", "", ""], ["off right edge", "", "", ""]])

# ---------- Section 3 ----------
h("Section 3 - Stage 3: Proportional Tuning")
note("Symptom -> cause table from the assignment (pre-filled). 'Action' column: fill at school.")
table(["Symptom", "Likely cause", "Action"],
      [["Runs smoothly on straights but cuts curves", "kp too low", ""],
       ["Oscillates about the line, worse at higher speed", "kp too high", ""],
       ["Holds line on straights but leaves it on a curve", "kp too low OR baseSpeed too high", ""],
       ["Leaves line immediately and does not return", "wrong sign of correction OR wrong white_line", ""]])
note("[HW 09-11] evidence so far: kp=0.3 kd=1.0 -> fast zigzag (D-term saturation "
     "suspected); kp=0.25/0.2 with kd=0.3 -> curve loss. Treat [SIM] thresholds "
     "(no oscillation up to kp=2.0) as NOT transferred to hardware.")
table(["Run #", "kp", "baseSpeed", "Section", "Behaviour", "Notes"],
      [["1", "0.3 [HW]", "90", "straight+gentle", "fast zigzag [HW]", "kd=1.0 saturating"],
       ["2", "0.2 [HW]", "90", "straight+gentle", "zigzag [HW]", "kd=0.3"],
       ["3", "0.25 [HW]", "90", "curve", "lost on curve [HW]", "smooth=on"],
       ["4", "", "", "", "", ""],
       ["5", "", "", "", "", ""]])

# ---------- Section 4 ----------
h("Section 4 - Stage 4: Full PID Tuning")
table(["dt_avg (ms)", "over (%)", "OK? (Y/N)"],
      [["20 [HW]", "0 [HW]", "trace off only - recheck with trace on"]])
table(["Step", "kp", "ki", "kd", "baseSpeed", "smoothError", "Behaviour", "Trace file"],
      [["2 P-oscillate", "", "", "", "", "", "", ""],
       ["3 P-backed-off", "", "", "", "", "", "", ""],
       ["4 PD", "", "", "", "", "", "", ""],
       ["5 P-retune", "", "", "", "", "", "", ""],
       ["6 PDI", "", "", "", "", "", "", ""],
       ["6b PD-only-final", "", "", "", "", "", "", ""],
       ["7 high-speed", "", "", "", "", "", "", ""]])
note("Sketch defaults now kp=0.25 ki=0 kd=0.3 base=90 smooth=on [DEFAULTS]. "
     "Period convention: absorbed into ki/kd (LOOP_PERIOD_MS=20). D on raw error "
     "unless 'm' toggles smoothing. Neither choice is final.")

# ---------- Section 5 ----------
h("Section 5 - Stage 5: Line Search")
table(["searchSpinSpeed", "timeoutMs", "foundReadingsNeeded"],
      [["120 (default) [DEFAULTS]", "3000 (default) [DEFAULTS]; T0 now refused - falls back to 3000", "3 (default) [DEFAULTS]"]])
table(["Situation", "Spin speed", "timeoutMs", "foundNeeded", "Recovered? (Y/N)", "Time to recover (s)", "Notes"],
      [["A left straight", "", "", "", "", "", ""],
       ["B right straight", "", "", "", "", "", ""],
       ["C sharp curve", "", "", "", "", "", ""],
       ["D end of line", "", "", "", "", "", ""],
       ["E free", "", "", "", "", "", ""]])
note("Bug 1 (return to FOLLOWING with search disabled) is fixed in code and "
     "host-tested 9/9 [SIM]; bench regression test = fix_prompt steps 3-5. "
     "Known suspicion: during a fast spin the strip may sweep the line in 1-2 ticks "
     "so 3 consecutive readings never accumulate - observe 'spins past the line' [HW].")

# ---------- Section 6 ----------
h("Section 6 - Stage 6: Test Track")
table(["Run #", "white_line chosen", "calibratedMin[0..4]", "calibratedMax[0..4]", "OK? (Y/N)"],
      [["ref", "0 [HW]", "286,276,270,223,183 [HW]", "878,868,767,720,586 [HW]", "yes [HW]"],
       ["2", "", "", "", ""],
       ["3", "", "", "", ""]])
table(["Section", "Direction", "kp", "ki", "kd", "base", "spinSpeed", "Completed?", "Lost line?", "Notes"],
      [[s, d, "", "", "", "", "", "", "", ""]
       for s, d in [("Straight fwd","fwd"),("Gentle curve fwd","fwd"),("Tight slalom fwd","fwd"),
                    ("Sharp corner fwd","fwd"),("Crossing fwd","fwd"),("Straight rev","rev"),
                    ("Gentle curve rev","rev"),("Slalom rev","rev")]])
table(["Run #", "Direction", "baseSpeed", "settings", "Outcome", "Time (s)", "Notes"],
      [["", "", "", "", "", "", ""] for _ in range(6)])
table(["Voltage (V)", "Behaviour change", "Settings adjustment"],
      [["", "", ""], ["", "", ""]])
note("Crossing handling defaults: handling=on, threshold=3 sensors, hold=200 ms [DEFAULTS]. "
     "Crossing NEVER tested on hardware [HW-PENDING].")

# ---------- Section 7 ----------
h("Section 7 - Demo Q&A Cheat Sheet (templates; numbers = fill at school)")
qa = [
 ("1. Why does kp have the value it has?",
  "I started from the max-correction formula kp = maxCorrection / errorAtFullCorrection, "
  "then tuned live. At kp = ____ the robot ____ (zigzag / cuts curves / follows). "
  "Observation: trace e-sign flips vs sustained sign (see dump)."),
 ("2. Why does ki have the value it has?",
  "ki = 0 because the line follower passes through zero error continuously and the sim "
  "showed no long-curve improvement [SIM]; at school I tried ki = ____ and observed ____."),
 ("3. Why does kd have the value it has?",
  "kd damps oscillation. kd = ____ gave ____ (with kd=1.0 the DRY output saturated at "
  "+-255 [HW]); the trade-off is D amplifying sensor noise."),
 ("4. Why did you choose that line-search strategy?",
  "Spin toward the side where the line was last seen (readLine returns -1 / 4001 with the "
  "modified library), N=3 consecutive good readings before committing, 3 s timeout that "
  "stops the robot. Observed: ____."),
 ("5. What does the controller do at a crossing, and why?",
  "When >= 3 sensors see the line, u is frozen (lastGoodU) for 200 ms so the weighted "
  "average cannot snap to the transverse stripe; observed: ____."),
 ("6. What happens when one coefficient is changed?",
  "Raising kp: ____; raising kd: ____; adding ki: ____ (fill from live experiments)."),
]
for q, a in qa:
    p = doc.add_paragraph(); r = p.add_run(q); r.bold = True
    doc.add_paragraph(a)

# ---------- Section 8 ----------
h("Section 8 - Library Knowledge (pre-written)")
doc.add_paragraph(
    "readLine() returns a weighted average in 0..4000 normally. When no sensor sees "
    "the line it returns -1 (line last seen left of centre) or 4001 (last seen right) - "
    "both from the modified TRSensors library (static last_value vs centre 2000). "
    "After subtracting the 2000 setpoint these become -2001/+2001, outside +-2000, "
    "which is exactly how lineLost() detects a lost line and derives the search side. "
    "The manufacturer's original returned 0 or 4000, indistinguishable from 'line "
    "under the outermost sensor', making lost-line detection impossible without extra code. "
    "The unsigned-int trap: storing -1 in unsigned int yields 65535, the sign is destroyed.")
note("Quote TRSensors.cpp lines 178-223 (readLine), 126-156 (readCalibrated), "
     "84-123 (calibrate) - see library_explainer.md for the full annotated source.")

doc.save("observation_notebook.docx")
print("saved observation_notebook.docx")
