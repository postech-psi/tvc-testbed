"""Extract and render source PDFs; never change the originals."""
from pathlib import Path
import json
import subprocess
from pypdf import PdfReader

OUT = Path(__file__).parent / 'source_inspection'
OUT.mkdir(exist_ok=True, parents=True)
SOURCES = {
    'spannagl2021': 'C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Design, Optimal Guidance and Control of a Low-cost Re-usable Electric Model Rocket - Spannagl et al. - 2021.pdf',
    'elke2024': 'C:/Users/tae06/Downloads/Elke and Caverly (2024) - Dynamics Guidance and Control of a Low-Cost Quadcopter-Based Space Vehicle Testbed.pdf',
    'elke2021': 'C:/Users/tae06/Downloads/Elke Pei Caverly and Gebre-Egziabher (2021) - A Low-Cost and Low-Risk Testbed for Control Design of Launch Vehicles and Landing Systems.pdf',
    'uploaded2602': 'C:/Users/tae06/Downloads/2602.21583v2.pdf',
    'linsen2022': 'C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Optimal Thrust Vector Control of an Electric Small-Scale Rocket Prototype - Linsen et al. - 2022.pdf',
    'chen2024': 'C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Design, Modeling, and Control of a Coaxial Drone - Chen et al. - 2024.pdf',
    'denton2022': 'C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/Design, development, and flight testing of a tube-launched coaxial-rotor based micro air vehicle - .pdf',
    'denton2025': 'C:/Users/tae06/OneDrive - postech.ac.kr/02 Papers/UGRP/System identification of a thrust-vectoring, coaxial-rotor-based gun-launched micro air vehicle in h - .pdf',
}
for key, path in SOURCES.items():
    reader = PdfReader(path)
    pages = [{'page':i, 'text':p.extract_text()} for i,p in enumerate(reader.pages,1)]
    (OUT/f'{key}.json').write_text(json.dumps({'source':path,'pages':pages}, ensure_ascii=False, indent=2), encoding='utf-8')
    (OUT/f'{key}.txt').write_text('\n\n'.join(f"=== PDF PAGE {p['page']} ===\n{p['text']}" for p in pages),encoding='utf-8')
    print(f'\n{key}: {len(pages)} pages\n{pages[0]["text"][:5000]}')
    subprocess.run(['pdftoppm','-f','1','-l','1','-scale-to','1400','-png',path,str(OUT/key)], check=True)
