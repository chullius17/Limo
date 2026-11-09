"""Render source-derived diagrams for the traj_package inflation explanation."""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np


OUT = Path(__file__).resolve().parent
BG = '#f7f9fc'
INK = '#172c43'
MUTED = '#52657a'
BLUE = '#1769aa'
ORANGE = '#dc7b20'
GREEN = '#218a64'
LINE = '#8d9eb0'

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 12,
    'svg.fonttype': 'none',
    'savefig.facecolor': BG,
    'axes.unicode_minus': False,
})


def write(fig, basename):
    fig.savefig(OUT / (basename + '.png'), dpi=200)
    fig.savefig(OUT / (basename + '.svg'))
    plt.close(fig)


def panel(ax, x, y, w, h, fill='white', edge='#d3dde8', lw=1.3):
    p = FancyBboxPatch((x, y), w, h,
                       boxstyle='round,pad=0.008,rounding_size=0.013',
                       facecolor=fill, edgecolor=edge, linewidth=lw,
                       transform=ax.transAxes)
    ax.add_patch(p)
    return p


def arrow(ax, start, end, color=LINE, lw=1.8, connection='arc3'):
    p = FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=14,
                        linewidth=lw, color=color, connectionstyle=connection,
                        transform=ax.transAxes)
    ax.add_patch(p)


def line(ax, xy, color=LINE, lw=1.8):
    x, y = zip(*xy)
    ax.plot(x, y, color=color, linewidth=lw, transform=ax.transAxes,
            solid_capstyle='round')


def txt(ax, x, y, text, size=12, color=INK, weight=None,
        ha='left', va='center', **kwargs):
    return ax.text(x, y, text, fontsize=size, color=color, weight=weight,
                   ha=ha, va=va, transform=ax.transAxes, **kwargs)


def draw_flow():
    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    txt(ax, .045, .935, 'Inflation map: il flusso nel traj_package',
        27, weight='bold')
    txt(ax, .045, .883,
        'Stessa mappa sorgente; StaticLayer precede BorderFollowLayer',
        15, MUTED)

    # Input and split.
    panel(ax, .04, .525, .17, .19, fill='#eaf2fc', edge='#aac9e9')
    txt(ax, .125, .67, 'OccupancyGrid', 16, BLUE, 'bold', ha='center')
    txt(ax, .125, .622, 'map_topic', 13, ha='center')
    txt(ax, .125, .568, 'mappa online\no /map (ideale)', 12, MUTED,
        ha='center', linespacing=1.5)
    line(ax, [(.21, .62), (.23, .62), (.23, .79)])
    arrow(ax, (.23, .79), (.272, .79))
    line(ax, [(.23, .62), (.23, .46)])
    arrow(ax, (.23, .46), (.272, .46))
    ax.plot([.23], [.62], 'o', ms=5, color=LINE, transform=ax.transAxes)

    # Static cost branch.
    panel(ax, .28, .718, .405, .132)
    txt(ax, .302, .815, 'StaticLayer  ·  costo della mappa di base', 15,
        BLUE, 'bold')
    txt(ax, .302, .775,
        'Sorgente ≥ 60  →  ostacolo letale (254)', 13.5)
    txt(ax, .302, .739,
        'I valori intermedi mantengono un costo semantico', 11.5, MUTED)

    # Geometric cost branch with explicit source/transform/metric steps.
    panel(ax, .28, .26, .405, .405, fill='#edf5fc', edge='#bed4e8')
    txt(ax, .302, .632, 'BorderFollowLayer  ·  costo geometrico',
        15, BLUE, 'bold')

    steps = [
        (.534, '1', 'Maschera binaria',
         'Sorgente ≥ 10 → 0; altre celle → 255'),
        (.423, '2', 'Distanza euclidea esatta',
         'OpenCV: DIST_L2 + DIST_MASK_PRECISE'),
        (.312, '3', 'Distanza in metri → profilo C(d)',
         'd = distanza in pixel × risoluzione della mappa'),
    ]
    for sy, num, heading, detail in steps:
        panel(ax, .30, sy, .365, .083, fill='white', edge='#d0dfec', lw=1)
        txt(ax, .319, sy+.049, num, 14, BLUE, 'bold')
        txt(ax, .347, sy+.053, heading, 12.7, INK, 'bold')
        txt(ax, .347, sy+.021, detail, 10.7, MUTED)
    arrow(ax, (.48, .534), (.48, .513), BLUE, 1.3)
    arrow(ax, (.48, .423), (.48, .402), BLUE, 1.3)
    txt(ax, .302, .278,
        'C(d) solo su celle note con valore sorgente < 10', 10.8, MUTED)

    # Join, published costmap and consuming planner.
    line(ax, [(.685, .79), (.716, .79), (.716, .64)])
    line(ax, [(.685, .353), (.716, .353), (.716, .64)])
    arrow(ax, (.716, .64), (.755, .64))
    panel(ax, .763, .565, .191, .15, fill='#e9f4ef', edge='#acd0be')
    txt(ax, .8585, .68, 'MAX', 22, GREEN, 'bold', ha='center')
    txt(ax, .8585, .634, 'max(costo base, C(d))', 12.5, ha='center')
    txt(ax, .8585, .596, 'Celle unknown conservate', 10.8, MUTED, ha='center')

    arrow(ax, (.8585, .565), (.8585, .509))
    panel(ax, .763, .398, .191, .104)
    txt(ax, .8585, .452, '/global_costmap/costmap', 12, BLUE,
        'bold', ha='center')
    txt(ax, .8585, .417, 'costmap globale risultante', 10.5, MUTED, ha='center')
    arrow(ax, (.8585, .398), (.8585, .339))
    panel(ax, .763, .216, .191, .116, fill='#eaf2fc', edge='#aac9e9')
    txt(ax, .8585, .295, 'SMAC Hybrid A*', 16, BLUE, 'bold', ha='center')
    txt(ax, .8585, .246, 'Valuta costi e collisioni\nper scegliere il percorso',
        11.8, ha='center', linespacing=1.4)

    panel(ax, .04, .085, .645, .112, fill='white')
    txt(ax, .057, .161,
        'Le sorgenti ≥ 10 includono anche le linee bianche (30).', 12.5,
        weight='bold')
    txt(ax, .057, .122,
        'C(d) = 0 non cancella il costo di base; la cresta C(d) = 100 è attraversabile.',
        11.8, MUTED)
    txt(ax, .045, .035,
        'Configurazione: traj_sim.yaml  •  Costi interni Nav2 (0–254), distinti dai valori OccupancyGrid (0–100)',
        10.5, MUTED)
    write(fig, 'diagramma_inflation_map')


def cost(d):
    d = np.asarray(d, dtype=float)
    y = np.zeros_like(d)
    near = (d >= 0) & (d < .18)
    mid = (d > .22) & (d < .53)
    far = d > .57
    y[near] = 120 * (1 - d[near] / .18)
    y[mid] = 100 * (1 - abs(d[mid] - .375) / .155)
    y[far] = np.minimum(8, 10 * (d[far] - .57))
    # C++ std::lround for nonnegative profile outputs.
    return np.floor(np.maximum(y, 0) + .5)


def draw_profile():
    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    canvas = fig.add_axes([0, 0, 1, 1])
    canvas.set_axis_off()
    txt(canvas, .045, .935, 'Il profilo di costo in simulazione',
        27, weight='bold')
    txt(canvas, .045, .883,
        'Due fasce preferite: d₁ = 0,20 m e d₂ = 0,55 m, entrambe con tolleranza ±0,02 m',
        15, MUTED)

    ax = fig.add_axes([.085, .325, .86, .485], facecolor='white')
    ax.set_xlim(-.012, 1.45)
    ax.set_ylim(-6, 131)
    ax.set_axisbelow(True)
    ax.grid(axis='y', color='#e4eaf1', linewidth=1)
    for edge in ['top', 'right']:
        ax.spines[edge].set_visible(False)
    for edge in ['bottom', 'left']:
        ax.spines[edge].set_color('#b8c6d5')
    ax.tick_params(colors=MUTED, labelsize=11)
    ax.set_xticks([0, .18, .375, .57, .8, 1., 1.2, 1.37])
    ax.set_xticklabels(['0', '0,18', '0,375', '0,57', '0,8', '1,0', '1,2', '1,37'])
    ax.set_yticks([0, 20, 40, 60, 80, 100, 120])
    ax.set_xlabel('Distanza d dalla sorgente più vicina [m]', color=INK,
                  fontsize=12, labelpad=9)
    ax.set_ylabel('Costo aggiuntivo C(d)', color=INK, fontsize=12, labelpad=10)

    for a, b in [(.18, .22), (.53, .57)]:
        ax.axvspan(a, b, color='#b8e1cf', alpha=.8, zorder=0)
    ranges = [(.0001, .18, ORANGE), (.18, .22, GREEN),
              (.22, .53, ORANGE), (.53, .57, GREEN), (.57, 1.45, BLUE)]
    for start, end, color in ranges:
        d = np.linspace(start, end, max(20, int((end-start)*7000)))
        ax.plot(d, cost(d), color=color, linewidth=2.7,
                solid_capstyle='round')
    ax.plot([0], [120], 'o', markerfacecolor='white', markeredgecolor=ORANGE,
            markeredgewidth=2, markersize=7, zorder=10)
    ax.plot([.375], [100], 'o', color=ORANGE, markersize=5, zorder=10)
    ax.annotate('Cresta: 100\na d = 0,375 m', xy=(.375, 100), xytext=(.48, 116),
                fontsize=12, color=INK, ha='left', va='center',
                arrowprops={'arrowstyle': '-', 'color': MUTED},
                linespacing=1.4)
    ax.text(.066, 75, 'Vicino alla\nsorgente', fontsize=11.5,
            color=ORANGE, ha='left', linespacing=1.4)
    ax.annotate('1ª fascia\n0,18–0,22 m', xy=(.20, 0), xytext=(.20, 29),
                color=GREEN, ha='center', fontsize=11.5,
                arrowprops={'arrowstyle': '-|>', 'color': GREEN},
                linespacing=1.4)
    ax.annotate('2ª fascia\n0,53–0,57 m', xy=(.55, 0), xytext=(.55, 29),
                color=GREEN, ha='center', fontsize=11.5,
                arrowprops={'arrowstyle': '-|>', 'color': GREEN},
                linespacing=1.4)
    ax.annotate('Tetto: 8', xy=(1.37, 8), xytext=(1.28, 24),
                color=BLUE, ha='center', fontsize=11.5,
                arrowprops={'arrowstyle': '-|>', 'color': BLUE})

    # Magnification keeps the small far-distance slope legible.
    inset = fig.add_axes([.60, .535, .305, .24], facecolor='#f7fbff')
    d = np.linspace(.57, 1.45, 2000)
    inset.plot(d, cost(d), color=BLUE, linewidth=2.2)
    inset.set_xlim(.57, 1.45)
    inset.set_ylim(-.5, 9)
    inset.set_title('Ingrandimento: dopo la 2ª fascia', loc='left',
                    color=BLUE, fontsize=11.5, pad=8)
    inset.set_xticks([.57, .8, 1.1, 1.37])
    inset.set_xticklabels(['0,57', '0,8', '1,1', '1,37'])
    inset.set_yticks([0, 4, 8])
    inset.tick_params(colors=MUTED, labelsize=9.5)
    inset.grid(color='#e1eaf4', linewidth=.7)
    for spine in inset.spines.values():
        spine.set_color('#c7d9e9')

    # Formula cards use the exact profile, before integer rounding.
    cards = [
        (.045, 'Vicino alla sorgente', '120 · (1 − d / 0,18)',
         'per 0 < d < 0,18 m', ORANGE),
        (.353, 'Fra le fasce preferite', '100 · (1 − |d − 0,375| / 0,155)',
         'per 0,22 < d < 0,53 m', ORANGE),
        (.661, 'Oltre la seconda fascia', 'min(8, 10 · (d − 0,57))',
         'per d > 0,57 m', BLUE),
    ]
    for x, heading, formula, detail, color in cards:
        panel(canvas, x, .125, .286, .116, fill='white')
        txt(canvas, x+.014, .218, heading, 11.8, color, 'bold')
        txt(canvas, x+.014, .181, formula, 12.1, INK)
        txt(canvas, x+.014, .146, detail, 10.5, MUTED)

    txt(canvas, .045, .079,
        'Costi interni Nav2; fascia a zero = nessun costo aggiuntivo. Il grafico mostra l’arrotondamento a interi.',
        11.2, MUTED)
    txt(canvas, .045, .044,
        'd = distanza dal centro della cella sorgente più vicina. A d = 0 è mostrato solo il limite del profilo: le sorgenti sono escluse.',
        10.8, MUTED)
    write(fig, 'profilo_costi_simulazione')


if __name__ == '__main__':
    assert cost([.1, .18, .20, .22, .375, .53, .55, .57, .8, 1.37]).tolist() == [53, 0, 0, 0, 100, 0, 0, 0, 2, 8]
    draw_flow()
    draw_profile()
    print('Rendered diagramma_inflation_map and profilo_costi_simulazione as PNG + SVG')
