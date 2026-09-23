"""A categorical scoping map, not a numerical performance or novelty ranking."""
from pathlib import Path
import json
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).parent
X = ['Platform / integrated\nGNC demonstration', 'Actuation dynamics /\nforce–torque coupling', 'Plant identification /\nsurrogate fidelity', 'Uncertainty /\ndisturbance rejection']
Y = ['Feedback + allocation\n(PID / nonlinear)', 'Model-based optimization\n(MPC / optimal G / LQR)', 'Robust / adaptive /\nincremental feedback', 'System / parameter\nidentification', 'Learned model /\nlearning-based adaptation', 'Direct reinforcement\nlearning', 'Residual reinforcement\nlearning']

# One primary emphasis per paper. This is an editorial reading guide, not a
# claim that the paper lacks every contribution outside the selected cell.
PAPERS = [
 ('Spannagl (2021)', 'EmboRockETH', 'platform', 0, 1, 'https://doi.org/10.1109/IROS51168.2021.9636430'),
 ('Linsen (2022)', 'Electric rocket', 'platform', 0, 1, 'https://doi.org/10.1109/ICRA46639.2022.9811938'),
 ('Santos (2026, E-Rocket)', 'Coaxial gimbal', 'platform', 0, 0, 'https://arxiv.org/html/2512.06535v2'),
 ('Santos (2026, QuadRocket)', 'Quadrotor surrogate', 'platform', 3, 2, 'https://doi.org/10.1109/TAES.2026.3706328'),
 ('Chen (2024)', 'Coaxial vectoring', 'platform', 1, 0, 'https://doi.org/10.1109/TRO.2024.3354161'),
 ('Denton (2022)', 'Tube-launched coaxial MAV', 'platform', 0, 0, 'https://doi.org/10.1177/17568293221117189'),
 ('Chih (2024)', 'Rocket-type coaxial UAV', 'platform', 1, 2, 'https://doi.org/10.1016/j.isatra.2024.06.029'),
 ('Denton (2025)', 'Coaxial hover identification', 'platform', 2, 3, 'https://doi.org/10.1177/17568293251361078'),
 ('Chih (2026)', 'Rocket-type parameter ID', 'platform', 2, 3, 'https://doi.org/10.1016/j.apm.2026.117000'),
 ('Elke (2024)', 'CRQS surrogate, simulation', 'platform', 2, 1, 'https://doi.org/10.2514/6.2024-0778'),
 ('Li (2024)', 'Tiltable quadrotor', 'method', 1, 1, 'https://doi.org/10.1109/LRA.2024.3451391'),
 ('Osedo (2023)', 'Uniaxial TVC rig', 'method', 3, 5, 'https://doi.org/10.1186/s40648-023-00260-0'),
 ('Cuniato (2024)', 'Omnidirectional tilt-rotor', 'method', 3, 5, 'https://doi.org/10.1007/978-3-031-63596-0_33'),
 ("O’Connell (2022)", 'Neural-Fly, quadrotor', 'method', 3, 4, 'https://doi.org/10.1126/scirobotics.abm6597'),
 ('Liu (2022)', 'Blimp; method transfer only', 'method', 3, 6, 'https://doi.org/10.1109/IROS47612.2022.9981182'),
 ('Smeur (2018)', 'INDI, quadrotor', 'method', 3, 2, 'https://doi.org/10.1016/j.conengprac.2018.01.003'),
 ('Torrente (2021)', 'GP-MPC, quadrotor', 'method', 3, 4, 'https://doi.org/10.1109/LRA.2021.3061307'),
]

def main():
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':11, 'svg.fonttype':'none'})
    fig, axs = plt.subplots(2, 1, figsize=(15.5, 17.5))
    fig.subplots_adjust(left=.205, right=.985, top=.915, bottom=.16, hspace=.32)
    fig.suptitle('Electric TVC testbed: literature positioning', x=.055, y=.983, ha='left', fontsize=23, weight='bold')
    fig.text(.055, .954, 'Scoping draft • primary emphasis per paper • named categories, not scores', fontsize=13)
    for ax, group, title in zip(axs, ['platform','method'], ['A. Platform and model studies (10 papers)', 'B. Method comparators (7 papers; different plants)']):
        ax.set_xlim(-.5, 3.5)
        ax.set_ylim(6.5, -.5)
        ax.set_xticks(range(4), X, fontsize=11)
        ax.set_yticks(range(7), Y, fontsize=11)
        ax.tick_params(axis='both', length=0, pad=10)
        for x in range(5): ax.axvline(x-.5, color='#d4d4d4', lw=.7, zorder=0)
        for y in range(8): ax.axhline(y-.5, color='#d4d4d4', lw=.7, zorder=0)
        for spine in ax.spines.values(): spine.set_visible(False)
        ax.set_title(title, loc='left', pad=17, fontsize=14, weight='bold')
        ax.set_xlabel('Primary research emphasis', labelpad=11, fontsize=12)
        ax.set_ylabel('Main approach used', labelpad=15, fontsize=12)
        cells=defaultdict(list)
        for name, platform, scope, x, y, url in PAPERS:
            if scope==group: cells[x,y].append(name)
        for (x,y), names in cells.items():
            offsets = [0] if len(names)==1 else [-.18,.18]
            for name, offset in zip(names, offsets):
                ax.text(x, y+offset, name, ha='center', va='center', fontsize=10.5, color='black')
    fig.text(.055, .067, 'Read with the Markdown evidence and GNC matrices. Blank cells mean no selected example, not an established research gap.', fontsize=11)
    fig.text(.055, .048, 'Panel A includes simulations and purpose-matched surrogates, not only free-flying gimbal vehicles. Panel B does not establish TVC transfer.', fontsize=11)
    fig.text(.055, .029, 'Spannagl and Linsen also address mismatch; QuadRocket also addresses coupling and actuation. Primary placement is not exclusive.', fontsize=11)
    fig.savefig(OUT/'tvc_literature_positioning_draft.png', dpi=150, facecolor='white')
    fig.savefig(OUT/'tvc_literature_positioning_draft.svg', facecolor='white')
    records=[dict(label=a,platform=b,scope=c,x_category=X[x].replace('\n',' '),y_category=Y[y].replace('\n',' '),source=url) for a,b,c,x,y,url in PAPERS]
    (OUT/'positioning_data.json').write_text(json.dumps({'status':'scoping draft; not an exhaustive review','date':'2026-09-23','papers':records},ensure_ascii=False,indent=2),encoding='utf-8')
    assert len(records)==17 and len({p['label'] for p in records})==17
    print('Rendered 17 distinct author-labelled papers: 10 platform/model + 7 method comparators.')

if __name__=='__main__': main()
