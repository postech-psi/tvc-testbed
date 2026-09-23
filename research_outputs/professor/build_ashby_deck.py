"""Optionally combine the four standalone vector figures into one PDF."""

from pathlib import Path
import os
from pypdf import PdfReader, PdfWriter

ROOT = Path(__file__).resolve().parent
PLOTS = Path(os.environ.get("TVC_FIGURE_OUT", ROOT / "focused_plots"))
DEST = Path(os.environ.get("TVC_DECK_OUT", ROOT.parents[1] / "output" / "pdf" / "TVC_Ashby_Plot_Deck.pdf"))
DEST.parent.mkdir(parents=True, exist_ok=True)
NAMES = [
    "01_hardware_thrust.pdf",
    "02a_close_tvc_control.pdf",
    "02b_transfer_methods.pdf",
    "03_validation.pdf",
]

writer = PdfWriter()
for name in NAMES:
    reader = PdfReader(PLOTS / name)
    if len(reader.pages) != 1:
        raise ValueError(f"Expected one plot per source PDF: {name}")
    writer.add_page(reader.pages[0])
writer.add_metadata({
    "/Title": "TVC physical and literature positioning",
    "/Author": "POSTECH TVC research group",
    "/Subject": "One numerical property chart and three categorical evidence maps",
})
with DEST.open("wb") as output:
    writer.write(output)
print(DEST)
