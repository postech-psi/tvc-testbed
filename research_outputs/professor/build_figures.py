"""Reproducible, monochrome categorical research figures (not performance plots)."""
from pathlib import Path
import json
import textwrap
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
DATA = json.loads((ROOT / 'professor_data.json').read_text(encoding='utf-8'))
plt.rcParams.update({'font.family': 'DejaVu Sans', 'svg.fonttype': 'none',
                     'text.color': 'black', 'axes.labelcolor': 'black'})

fig, ax = plt.subplots(figsize=(16, 6.6))
fig.subplots_adjust(left=.21, right=.99, top=.88, bottom=.22)
ax.set_xlim(-.5, 3.5)
ax.set_ylim(2.5, -.5)
ax.set_xticks(range(4), DATA['x_categories'], fontsize=11)
ax.set_yticks(range(3), DATA['y_categories'], fontsize=11)
ax.tick_params(axis='both', length=0, pad=13)
for x in [.5, 1.5, 2.5]:
    ax.axvline(x, color='.78', lw=.8)
for y in [.5, 1.5]:
    ax.axhline(y, color='.78', lw=.8)
for spine in ax.spines.values():
    spine.set_color('.6')
    spine.set_linewidth(.8)
for p in DATA['papers']:
    x, y = p['x'] - .43, p['y'] + p['offset']
    ax.plot(x, y, 'o', color='black', markersize=4.5)
    ax.text(x + .055, y - .058, p['label'], fontsize=10.5, weight='bold', va='center')
    subtitle = p['subtitle'].replace('Optimal G; offset-free MPC + inner feedback',
                                     'Optimal G; MPC + inner feedback').replace(
        'Adaptive backstepping; dynamic-surface control', 'Adaptive backstepping + actuator dynamics')
    ax.text(x + .055, y + .062, subtitle, fontsize=8.5, va='center')
fig.text(.02, .96, 'Electric TVC: controller architecture × experimental platform',
         fontsize=19, weight='bold', va='top')
fig.text(.02, .908, 'Seven selected studies | first-author labels | no color, shape or size encoding', fontsize=11)
fig.text(.21, .075, 'Control architecture (categorical; no ranking)', fontsize=11, weight='bold')
fig.text(.02, .025, 'Within-cell spacing separates labels only. Empty cells are not evidence of novelty. '
         'Detailed GNC roles and validation limits are in the companion matrix.', fontsize=9)
fig.savefig(ROOT / 'tvc_positioning_map.png', dpi=210, facecolor='white')
fig.savefig(ROOT / 'tvc_positioning_map.svg', facecolor='white')
plt.close(fig)

fig, ax = plt.subplots(figsize=(17.5, 7.8))
fig.subplots_adjust(left=.015, right=.985, top=.9, bottom=.05)
ax.axis('off')
headers = ['Paper', 'G: guidance', 'N: estimation', 'C: control', 'A: actuation', 'Evidence / boundary']
widths = [.145, .15, .18, .185, .155, .185]
keys = ['label', 'G', 'N', 'C', 'A', 'evidence']
chars = [26, 28, 34, 35, 29, 35]
rows = [[textwrap.fill(p[k], width=n, break_long_words=False) for k,n in zip(keys,chars)]
        for p in DATA['papers']]
tab = ax.table(cellText=rows, colLabels=headers, colWidths=widths, cellLoc='left',
               colLoc='left', bbox=[0, .06, 1, .90])
tab.auto_set_font_size(False)
tab.set_fontsize(10.5)
for (r,c), cell in tab.get_celld().items():
    cell.set_facecolor('white')
    cell.set_edgecolor('.7')
    cell.set_linewidth(.6)
    cell.PAD = .05
    if r == 0 or c == 0:
        cell.get_text().set_weight('bold')
fig.text(.02, .97, 'What each paper actually contributes to GNC', fontsize=20, weight='bold', va='top')
fig.text(.02, .92, 'Separate guidance, estimation, feedback, allocation and experimental evidence; '
         'do not treat “MPC” and “GNC” as competing categories.', fontsize=11)
fig.text(.02, .03, 'Same seven papers as the map. Presence of an estimator does not imply navigation novelty. '
         'Read the paper-specific source locators in TVC_Research_Notes.md.', fontsize=10)
fig.savefig(ROOT / 'tvc_gnc_matrix.png', dpi=200, facecolor='white')
fig.savefig(ROOT / 'tvc_gnc_matrix.svg', facecolor='white')
plt.close(fig)
assert len(DATA['papers']) == 7
assert len({p['label'] for p in DATA['papers']}) == 7
print('Created map and GNC matrix as PNG + SVG; 7 distinct paper labels.')
