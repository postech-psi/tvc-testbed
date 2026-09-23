"""Check package integrity, paper coverage and local reading links."""
from pathlib import Path
import json
import re
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parent
data = json.loads((ROOT/'professor_data.json').read_text(encoding='utf-8'))
papers = data['papers']
assert len(papers) == len({p['label'] for p in papers}) == 7
for p in papers:
    assert set(['G','N','C','A','evidence','locator','source']).issubset(p)
    assert 0 <= p['x'] <= 3 and 0 <= p['y'] <= 2
for key in ['tvc_positioning_map','tvc_gnc_matrix']:
    svg = (ROOT/f'{key}.svg').read_text(encoding='utf-8')
    for p in papers:
        assert p['label'] in svg, (key, p['label'])
    assert (ROOT/f'{key}.png').stat().st_size > 10000
map_svg = (ROOT/'tvc_positioning_map.svg').read_text(encoding='utf-8')
for forbidden in ['Chih','Neural-Fly','Torrente','Liu (2022)','Denton (2025)','Cuniato','Elke','Zhang']:
    assert forbidden not in map_svg, forbidden

pdf = ROOT.parents[1]/'output/pdf/TVC_Professor_Brief.pdf'
reader = PdfReader(pdf)
assert len(reader.pages) == 4
text = '\n'.join(p.extract_text() for p in reader.pages)
for p in papers:
    assert p['label'] in text, p['label']
for word in ['Torrente (2021)', 'Li (2024)', 'Zhang (2026)', '0.077', '0.042', '19 successful',
             'simulation only', 'Chih excluded', 'A. Actuator fidelity', 'D. Guidance feasibility']:
    assert word in text, word
assert '■' not in text and '\ufffd' not in text
link_count = sum(len(p.get('/Annots',[])) for p in reader.pages)
assert link_count >= 10, link_count

local_links = []
for name in ['README.md','TVC_Research_Notes.md']:
    md = (ROOT/name).read_text(encoding='utf-8')
    for target in re.findall(r'\]\((?:<([^>]+)>|([^\)]+))\)', md):
        val = target[0] or target[1]
        if val.startswith('C:/'):
            assert Path(val).exists(), val
            local_links.append(val)
assert len(local_links) >= 18
print(f'PASS: 4 PDF pages, 7 unique map/matrix papers, {link_count} PDF source links, '
      f'{len(local_links)} valid local Markdown links; exclusions and key caveats present.')
