"""Draw lettered planning-costmap regions from the current trajectory YAMLs."""

from html import escape
import math
from pathlib import Path

import cairo
import gi
import yaml

gi.require_version('Rsvg', '2.0')
from gi.repository import Rsvg


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
WIDTH, HEIGHT = 1600, 1450
X0, X1, DMAX = 225, 1485, 1.5
INK, MUTED, GRID = '#172c43', '#52657a', '#dbe3eb'
COLORS = {
    'A': '#334155', 'B': '#f8cec6', 'C': '#bce3cc',
    'D': '#ffe0ac', 'E': '#bce3cc', 'F': '#d8e8f8',
}
parts = []


def element(name, attrs=None, body=None):
    attributes = ' '.join(f'{k}="{escape(str(v), quote=True)}"'
                          for k, v in (attrs or {}).items())
    parts.append(f'<{name} {attributes}>' + (body or '') + f'</{name}>')


def text(x, y, content, size=18, fill=INK, weight='normal', anchor='start'):
    element('text', {
        'x': x, 'y': y, 'font-size': size, 'fill': fill,
        'font-weight': weight, 'text-anchor': anchor,
    }, escape(content))


def rect(x, y, width, height, fill, radius=0, **extra):
    element('rect', dict(x=x, y=y, width=width, height=height,
                         rx=radius, fill=fill, **extra))


def line(xa, ya, xb, yb, color=GRID, width=1, **extra):
    element('line', dict(x1=xa, y1=ya, x2=xb, y2=yb,
                         stroke=color, **{'stroke-width': width}, **extra))


def badge(x, y, letter):
    element('circle', {'cx': x, 'cy': y, 'r': 17,
                       'fill': COLORS[letter], 'stroke': INK, 'stroke-width': 1.2})
    text(x, y + 6, letter, 20, 'white' if letter == 'A' else INK, 'bold', 'middle')


def px(distance):
    return X0 + (X1 - X0) * distance / DMAX


def read_profile(name):
    path = ROOT / 'ros2_ws' / 'traj_package' / 'config' / f'traj_{name}.yaml'
    values = yaml.safe_load(path.read_text())['global_costmap']['global_costmap']['ros__parameters']
    layer = values['border_follow_layer']
    return dict(
        first=0.5 * layer['robot_width'] + layer['safety_margin'],
        second=layer['second_lane_distance'], tol=layer['distance_tolerance'],
        near=layer['near_max_cost'], ridge=layer['inter_lane_peak_cost'],
        slope=layer['far_cost_slope'], cap=layer['far_max_cost'],
        resolution=values['resolution'],
    )


def bounds(p):
    return (p['first'] - p['tol'], p['first'] + p['tol'],
            p['second'] - p['tol'], p['second'] + p['tol'])


def cost(distance, p):
    a, b, c, d = bounds(p)
    if distance < a:
        value = p['near'] * max(0, 1 - distance / a)
    elif distance <= b:
        value = 0
    elif distance < c:
        value = p['ridge'] * (1 - abs(distance - (b + c) / 2) / ((c - b) / 2))
    elif distance <= d:
        value = 0
    else:
        value = min(p['cap'], p['slope'] * (distance - d))
    return math.floor(max(0, value) + 0.5)


def cm(value):
    return f'{value * 100:g}'.replace('.', ',')


def draw_profile(top, title, p):
    rect(40, top, 1520, 515, '#ffffff', 15, stroke=GRID)
    text(65, top + 37, title, 26, weight='bold')
    text(1535, top + 36,
         f"d₁ = {cm(p['first'])} cm · d₂ = {cm(p['second'])} cm · tolleranza ±{cm(p['tol'])} cm",
         18, MUTED, anchor='end')
    a, b, c, d = bounds(p)
    regions = [('B', 0, a), ('C', a, b), ('D', b, c), ('E', c, d), ('F', d, DMAX)]
    map_y, map_h = top + 95, 100
    rect(X0 - 75, map_y, 75, map_h, COLORS['A'])
    badge(X0 - 37, map_y + 50, 'A')
    text(65, map_y + 45, 'Celle', 18, MUTED)
    text(65, map_y + 70, 'sorgente', 18, MUTED)
    for letter, start, end in regions:
        rect(px(start), map_y, px(end) - px(start), map_h, COLORS[letter])
        center = (start + end) / 2
        if letter == 'F':
            center = max(d + 0.2, 1.05)
        line(px(center), map_y - 4, px(center), map_y, INK)
        badge(px(center), map_y - 23, letter)
    rect(X0 - 75, map_y, X1 - X0 + 75, map_h, 'none', stroke=GRID)
    for preferred in (p['first'], p['second']):
        line(px(preferred), map_y, px(preferred), map_y + map_h,
             '#237e54', 1.7, **{'stroke-dasharray': '5 5'})
    text(X0, top + 225,
         f'B: 0–{cm(a)} cm     C: {cm(a)}–{cm(b)} cm     '
         f'D: {cm(b)}–{cm(c)} cm     E: {cm(c)}–{cm(d)} cm     F: oltre {cm(d)} cm',
         18)

    plot_y, plot_h = top + 270, 170
    bottom = plot_y + plot_h
    for letter, start, end in regions:
        rect(px(start), plot_y, px(end) - px(start), plot_h, COLORS[letter],
             **{'fill-opacity': '.38'})
    for value in (0, 40, 80, 120):
        y = bottom - value / 120 * plot_h
        line(X0, y, X1, y)
        text(X0 - 15, y + 6, str(value), 17, MUTED, anchor='end')
    text(65, plot_y + 20, 'Costo', 18, MUTED)
    text(65, plot_y + 45, 'aggiunto', 18, MUTED)
    text(65, plot_y + 70, 'C(d)', 21, weight='bold')
    line(X0, plot_y, X0, bottom, '#7c90a4', 1.5)
    line(X0, bottom, X1, bottom, '#7c90a4', 1.5)
    # d=0 is the profile limit; source cells themselves are excluded by the layer.
    distances = sorted(set([i * DMAX / 1800 for i in range(1801)] +
                           [a, b, c, d, (b + c) / 2, d + p['cap'] / p['slope']]))
    path = ' '.join(f'{"M" if i == 0 else "L"}{px(distance):.2f},'
                    f'{bottom - cost(distance, p) / 120 * plot_h:.2f}'
                    for i, distance in enumerate(distances))
    element('path', {'d': path, 'fill': 'none', 'stroke': '#1769aa', 'stroke-width': 3})
    element('circle', {'cx': X0, 'cy': plot_y, 'r': 4, 'fill': 'white',
                       'stroke': '#1769aa', 'stroke-width': 2})
    peak_x, peak_y = px((b + c) / 2), bottom - p['ridge'] / 120 * plot_h
    line(peak_x, peak_y - 3, peak_x + 36, peak_y - 30, MUTED)
    text(peak_x + 42, peak_y - 27, f"D: picco {p['ridge']}", 17, weight='bold')
    saturation = d + p['cap'] / p['slope']
    cap_x, cap_y = px(saturation), bottom - p['cap'] / 120 * plot_h
    line(cap_x, cap_y - 3, cap_x - 30, cap_y - 45, MUTED)
    text(cap_x - 36, cap_y - 49, f"F: tetto {p['cap']}",
         17, weight='bold', anchor='end')
    for distance in (0, .25, .5, .75, 1, 1.25, 1.5):
        x = px(distance)
        line(x, bottom, x, bottom + 6, '#7c90a4', 1.5)
        text(x, bottom + 27, cm(distance), 17, MUTED, anchor='middle')
    text((X0 + X1) / 2, top + 494,
         'Distanza d dalla cella sorgente più vicina [cm]', 19, anchor='middle')


def main():
    real, sim = read_profile('real'), read_profile('sim')
    parts.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" '
                 f'height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">')
    element('title', body='Inflation map di traj_package: regioni A–F')
    element('desc', body='Schema delle fasce di BorderFollowLayer e relativi profili di costo '
            'nei parametri reali e simulati attuali. A sorgenti, B repulsione vicina, '
            'C prima fascia preferita, D cresta intermedia, E seconda fascia preferita, F zona lontana.')
    parts.append('<g font-family="DejaVu Sans, sans-serif">')
    rect(0, 0, WIDTH, HEIGHT, '#f7f9fc')
    text(45, 52, 'Inflation map di traj_package: regioni A–F', 33, weight='bold')
    text(45, 87, 'BorderFollowLayer · schema attorno a un bordo rettilineo isolato · valori dagli YAML attuali',
         19, MUTED)
    legend = [
        ('A', 'Celle sorgente', 'Valore ≥10; letali da 60'),
        ('B', 'Repulsione vicina', 'Costo decrescente: 120 → 0'),
        ('C', 'Prima fascia preferita', 'Costo aggiunto C(d) = 0'),
        ('D', 'Cresta tra le fasce', 'Costo massimo 100, attraversabile'),
        ('E', 'Seconda fascia preferita', 'Costo aggiunto C(d) = 0'),
        ('F', 'Zona lontana', 'Costo crescente fino al tetto'),
    ]
    for i, (letter, heading, description) in enumerate(legend):
        x, y = 45 + (i % 3) * 510, 110 + (i // 3) * 60
        rect(x, y, 490, 53, '#ffffff', 10, stroke=GRID)
        badge(x + 28, y + 27, letter)
        text(x + 58, y + 23, heading, 18, weight='bold')
        text(x + 58, y + 44, description, 16, MUTED)
    draw_profile(245, 'Profilo REALE', real)
    draw_profile(785, 'Profilo SIMULAZIONE', sim)
    text(45, 1343, 'C(d) è il solo costo di BorderFollowLayer: costo finale = MAX(costo di base, C(d)).',
         19, weight='bold')
    text(45, 1375, 'Le fasce C/E non cancellano altri costi. Le celle sconosciute restano sconosciute.', 18, MUTED)
    text(45, 1407, 'Schema geometrico; d è misurata dai centri delle celle sorgente. Griglia: 5 cm; costi arrotondati a interi.',
         17, MUTED)
    parts.append('</g></svg>')
    svg_path = OUT / 'inflation_regions_af.svg'
    svg_path.write_text('\n'.join(parts) + '\n')
    handle = Rsvg.Handle.new_from_file(str(svg_path))
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, WIDTH, HEIGHT)
    context = cairo.Context(surface)
    viewport = Rsvg.Rectangle()
    viewport.x, viewport.y, viewport.width, viewport.height = 0, 0, WIDTH, HEIGHT
    handle.render_document(context, viewport)
    png_path = OUT / 'inflation_regions_af.png'
    surface.write_to_png(str(png_path))
    print(svg_path)
    print(png_path)


if __name__ == '__main__':
    main()
