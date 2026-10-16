import pymupdf, docx, sys, os

base = r"C:\4work\study\ISS\LAB\LAB1"
out = []

for f in ["ISS_LAB1_Odometry_AlphaBot_EN.pdf", "LAB1_ISS_Odometry.pdf"]:
    p = os.path.join(base, f)
    doc = pymupdf.open(p)
    out.append("=" * 30 + " " + f + " pages:" + str(len(doc)) + " " + "=" * 30)
    for pg in doc:
        out.append(pg.get_text())

# progress docx in parent folder
p = r"C:\4work\study\ISS\LAB\cur_progress.docx"
d = docx.Document(p)
out.append("=" * 30 + " cur_progress.docx " + "=" * 30)
for para in d.paragraphs:
    if para.text.strip():
        out.append(para.text)
for t in d.tables:
    out.append("--- table ---")
    for row in t.rows:
        out.append(" | ".join(c.text.strip() for c in row.cells))

with open(os.path.join(base, "_hostcheck", "extract_dump.txt"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(out))
print("written", len("\n".join(out)), "chars")
