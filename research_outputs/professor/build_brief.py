"""Four landscape pages for a professor discussion; detailed audit stays in Markdown."""
from pathlib import Path
import json
from xml.sax.saxutils import escape
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT = Path(__file__).resolve().parent
OUT = ROOT.parents[1] / 'output' / 'pdf'
OUT.mkdir(exist_ok=True, parents=True)
DATA = json.loads((ROOT / 'professor_data.json').read_text(encoding='utf-8'))
FONTDIR = Path('C:/Windows/Fonts')
pdfmetrics.registerFont(TTFont('Brief', str(FONTDIR / 'arial.ttf')))
pdfmetrics.registerFont(TTFont('Brief-Bold', str(FONTDIR / 'arialbd.ttf')))
pdfmetrics.registerFontFamily('Brief', normal='Brief', bold='Brief-Bold', italic='Brief', boldItalic='Brief-Bold')
W,H = landscape(A4)
M = 36
CW = W-2*M
c = canvas.Canvas(str(OUT / 'TVC_Professor_Brief.pdf'), pagesize=(W,H))
c.setTitle('Electric TVC - Literature Positioning and Research Direction')
c.setAuthor('Prepared for Lee Taeho - source-audited discussion draft')

def para(text, x, top, width, size=11, leading=None, bold=False):
    style = ParagraphStyle('p', fontName='Brief-Bold' if bold else 'Brief',
                           fontSize=size, leading=leading or size*1.32, textColor=colors.black,
                           spaceAfter=0)
    p = Paragraph(text, style)
    _, h = p.wrap(width, H)
    p.drawOn(c, x, top-h)
    return top-h

def head(n, kicker, title, deck):
    c.setFillColor(colors.black)
    para(kicker.upper(), M, H-26, CW, 9, bold=True)
    para(title, M, H-48, CW, 23, bold=True)
    para(deck, M, H-83, CW, 10.5)
    c.setStrokeColor(colors.HexColor('#999999'))
    c.setLineWidth(.6)
    c.line(M, 29, W-M, 29)
    para('LEE TAEHO  /  ELECTRIC TVC  /  23 SEP 2026  /  DISCUSSION DRAFT', M, 21, CW-40, 7.5)
    c.setFont('Brief', 9)
    c.drawRightString(W-M, 13, f'{n} / 4')

def table(rows, widths, top, size=9.3, padding=7):
    style = ParagraphStyle('cell', fontName='Brief', fontSize=size, leading=size*1.27)
    elements = [[Paragraph(v, style) for v in row] for row in rows]
    t = Table(elements, colWidths=widths)
    t.setStyle(TableStyle([
        ('VALIGN',(0,0),(-1,-1),'TOP'),
        ('LINEBELOW',(0,0),(-1,0),1,colors.black),
        ('LINEBELOW',(0,1),(-1,-1),.4,colors.HexColor('#BBBBBB')),
        ('LEFTPADDING',(0,0),(-1,-1),padding),
        ('RIGHTPADDING',(0,0),(-1,-1),padding),
        ('TOPPADDING',(0,0),(-1,-1),padding),
        ('BOTTOMPADDING',(0,0),(-1,-1),padding),
    ]))
    _, h = t.wrap(CW, H)
    assert top-h > 38, f'Table overflows page: bottom={top-h}'
    t.drawOn(c, M, top-h)
    return top-h

def link(url, label):
    return f'<link href="{url}"><u>{label}</u></link>'

head(1, '01 / Scope and positioning', 'Which literature belongs beside our testbed?',
     'A coaxial, counter-rotating propulsion unit on a two-axis gimbal. '
     'Compare demonstrated control architectures without mixing different actuation capabilities.')
c.drawImage(str(ROOT/'tvc_positioning_map.png'), M-9, 148, width=CW+18,
            height=(CW+18)*6.6/16, preserveAspectRatio=True, mask='auto')
para('<b>Scope:</b> the six platform papers from the review + Osedo\'s adjacent one-axis experiment. '
     'Denton (2022) is the single platform representative; Denton (2025) remains an identification follow-up.', M, 131, CW, 10.3)
para('<b>Read the empty cells carefully:</b> this is a categorical literature-positioning map, not a performance ranking or a proof of novelty. '
     'The separate transfer-method section retains Li, Torrente and Zhang without treating their vehicles as ours.', M, 87, CW, 10.3)
c.showPage()

head(2, '02 / GNC comparison', 'Compare the control stack, not only the algorithm name',
     'G = guidance; N = navigation/state and disturbance estimation; C = tracking/stabilization; A = allocation/actuation.')
rows = [['<b>Paper / source</b>','<b>G</b>','<b>N</b>','<b>C + A</b>','<b>Evidence / caveat</b>']]
order = ['Spannagl (2021)','Linsen (2022)','Santos (2026, E-Rocket)',
         'Santos (2026, QuadRocket)','Chen (2024)','Denton (2022)','Osedo (2023)']
short = {
    'Spannagl (2021)': ['Optimal free-time guidance; retargeting','State EKF + offset filter',
        'Offset-free MPC + inner feedback; inverse allocation; thrust lag modeled',
        'Indoor + 19 outdoor missions; onboard G/C'],
    'Linsen (2022)': ['Optimal free-time terminal-target guidance','Pixhawk filter + augmented EKF',
        'NMPC jointly tracks position and attitude; servo/motor constraints',
        'Indoor tracking/wind; outdoor apogee + landing'],
    'Santos (2026, E-Rocket)': ['Offline trajectory; mission logic','PX4 EKF + indoor motion capture',
        'PID cascade + allocation; simplified actuation model',
        'Indoor flights; IFAC accepted/program-listed'],
    'Santos (2026, QuadRocket)': ['Supplied reference trajectory','Motion-capture-supported feedback',
        'Adaptive backstepping + dynamic-surface control; different actuator',
        'Flight experiments; main tracking loop offboard'],
    'Chen (2024)': ['Ascent target; manual maneuvers also shown','IMU/ToF; separate Pixhawk hover',
        'Nonlinear allocation + damping; two serial tilt servos',
        'Custom ascent and Pixhawk hover are different stacks'],
    'Denton (2022)': ['Pilot commands; launch-to-hover','Onboard attitude estimation',
        'Cascaded feedback; gimbal + differential rotor speed',
        'Rig + free flight; not autonomous landing guidance'],
    'Osedo (2023)': ['Prescribed pitch target','Pitch/rate feedback',
        'PPO/LSTM + domain randomization; thrust/vectoring increments',
        'Pitch-only rig; out-of-range model case fails']}
for name in order:
    p = next(p for p in DATA['papers'] if p['label']==name)
    rows.append([link(p['source'], escape(name))] + short[name])
bottom=table(rows,[CW*.17,CW*.17,CW*.16,CW*.265,CW*.235],H-117,size=9.05,padding=6.4)
para('<b>Correction from the final Spannagl PDF:</b> 100 m maximum altitude, 50 m maximum diversion and 19 successful outdoor flights are supported '
     '(Sec. IV-C, p.6350). First-order thrust response is already in its MPC model (Eq.2e).', M, bottom-15, CW, 9.6)
para('<b>Research attribution:</b> hold G and N fixed when testing C. Existing state estimation is not, by itself, a new navigation contribution.',
     M, bottom-52, CW, 9.6)
c.showPage()

head(3, '03 / Transferable methods', 'Keep the useful mechanism; preserve the plant boundary',
     'These are supporting method references, not extra points in the seven-paper main map.')
rows = [
    ['<b>Reference</b>','<b>Mechanism worth transferring</b>','<b>Boundary for our project</b>'],
    [link('https://doi.org/10.1109/LRA.2021.3061307','<b>Torrente (2021)</b>')+'<br/>Residual-model MPC',
     'Learn dynamics error with Gaussian processes; insert it into MPC. Compare nominal, simple fitted and learned corrections.',
     'Fixed-rotor quadrotor, not TVC. Supervised learning, not RL. Benefits depend on operating regime; low-speed improvement is not guaranteed.'],
    [link('https://doi.org/10.1109/LRA.2024.3451391','<b>Li (2024)</b>')+'<br/>Servo-integrated NMPC',
     'Include actuator response in prediction/control; test the effect of the actuator model.',
     'Four tiltable rotors are overactuated. Servo ablation in simulation must not be presented as a hardware ablation.'],
    [link('https://arxiv.org/abs/2602.21583v2','<b>Zhang (2026)</b>')+'<br/>Direct actuator-level RL',
     'PPO + actuator identification + latency alignment + domain randomization; same-platform comparison against Li-style NMPC.',
     'This is the uploaded PDF, not Cuniato (2024). Four rotor/servo pairs. RA-L acceptance reported; final publication metadata unverified.']
]
bottom=table(rows,[CW*.205,CW*.38,CW*.415],H-117,size=10,padding=8)
top=bottom-18
top=para('Zhang\'s hardware results show a trade-off, not an overall RL victory', M, top, CW, 12, bold=True)-10
top=para('Waypoint tests: mean steady-state position-error norm 0.077 m (RL) vs 0.042 m (NMPC); '
         'orientation-error norm 4.261 vs 4.350 degrees (Table V, three sequence repeats). '
         'Trajectory tests favor NMPC in mean position and orientation error (Sec. III-D, five repeats). '
         'Allocation-singularity traversal is simulation only; the 90-degree hardware case uses a reconfigured plant and retrained policy.',
         M, top, CW, 10)-12
top=para('<b>Keep separate:</b> residual-model learning changes the prediction model; residual RL adds a learned action correction to a baseline controller. '
         'Neural-Fly adapts coefficients of learned features and is not RL. Neural-Fly and Liu remain idea notes, not main-map points.', M, top, CW, 10)-10
para('<b>Most useful lesson from Torrente:</b> test against a simple identified correction, not only an uncorrected nominal model. '
     'A learned method must earn its additional data and computation cost on our own platform.', M, top, CW, 10)
c.showPage()

head(4, '04 / Thesis-direction decision', 'Choose a measurable limitation before choosing a controller',
     'A proposed sequence for discussion, not a claim that the listed questions are already novel.')
rows = [
    ['<b>Candidate question</b>','<b>Prior-art boundary</b>','<b>Smallest decisive experiment</b>'],
    ['<b>A. Actuator fidelity</b><br/>When does measured motor/gimbal response improve constrained tracking?',
     'Spannagl models thrust lag; Linsen constrains actuator rates; Li integrates servo dynamics.',
     'Static vs identified dynamic actuator model, with the same control architecture, estimator, reference and limits.'],
    ['<b>B. Residual-model value</b><br/>What remains after physical identification and simple compensation?',
     'Offset estimation and learned residual models already exist. A new platform alone is insufficient.',
     'Identified baseline vs simple correction vs learned residual. Use shared training data and held-out complete runs.'],
    ['<b>C. Learned-control trade-off</b><br/>What robustness/computation benefit survives a matched comparison?',
     'Osedo provides constrained TVC RL; Zhang compares RL/NMPC on an overactuated plant.',
     'One tuned conventional baseline vs one learned architecture. Match sensing/action authority; report failures and runtime.'],
    ['<b>D. Guidance feasibility</b><br/>Does a specific actuator-feasibility constraint improve tracking/landing?',
     'Optimal guidance already exists in Spannagl and Linsen; identify the exact missing condition.',
     'Hold tracking and estimation fixed; vary only guidance feasibility treatment. Requires reliable tracking first.']
]
bottom=table(rows,[CW*.335,CW*.305,CW*.36],H-117,size=9.7,padding=7.5)
top=bottom-16
top=para('<b>Immediate gate:</b> safely measure actuator response, latency and static maps; validate on held-out runs; establish a tuned conventional baseline. '
         'Then choose A/B/C/D from the dominant measured limitation. This step does not commit the thesis to MPC or RL.',M,top,CW,10.5)-12
top=para('<b>Minimum evidence:</b> repeatable tracking error, peak error, saturation time, failure count and runtime distribution. '
         'Balance test order and battery state; fix G/N when evaluating C. Learning also needs data/compute and out-of-distribution reporting.',M,top,CW,10)-10
para('<b>Scope/access record:</b> Chih excluded; Elke retained only for surrogate-fidelity background; Denton (2025) not double-counted. '
     'The complete Markdown notes contain source locators, reading depth and limitations. No broad literature-completeness or novelty claim is made.',M,top,CW,9.3)
c.save()
print(str(OUT / 'TVC_Professor_Brief.pdf'))
