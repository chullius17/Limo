# Copyright 2026 Giulio Cataldo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Render an MPC snapshot without ROS, TF, NumPy or a graphical display."""

from dataclasses import dataclass, field
from functools import lru_cache
import math

from PIL import Image, ImageChops, ImageDraw, ImageFont


FAMILIES = ('braking', 'nominal', 'constant_curvature',
            'perturbed_nominal', 'broad_exploration', 'structured', 'monophase', 'startup')
LABELS = ('Braking', 'Nominal', 'Constant curvature',
          'Perturbed nominal', 'Broad exploration', 'Block actions', 'Single phase', 'Slow startup')
DEFAULT_COLORS = ((220, 75, 65), (35, 105, 220), (145, 75, 190),
                  (25, 165, 170), (220, 140, 35), (230, 60, 180), (70, 185, 195), (120, 90, 220))


@dataclass
class Candidate:
    """A sampled base_link trajectory, in the snapshot's costmap frame."""

    candidate_id: int
    family: int
    score_status: int
    cost: float
    poses: tuple


@dataclass
class PreviewFrame:
    """One controller cycle; costmap bytes retain Nav2's 0..255 costs."""

    robot_pose: tuple
    footprint: tuple
    costmap_size: tuple
    costmap_resolution: float
    costmap_origin: tuple
    costmap_data: bytes
    wheelbase: float
    rear_axle_to_base: float
    initial_velocity: float
    initial_steering: float
    command_velocity: float
    command_steering: float
    model_dt: float
    time_steps: int
    generated_count: int
    selected_id: int
    candidates: tuple


@dataclass
class PreviewBackground:
    """A published local costmap aligned to the current robot pose."""

    robot_pose: tuple
    costmap_size: tuple
    costmap_resolution: float
    costmap_origin: tuple
    costmap_data: bytes
    footprint: tuple = ()


@dataclass
class PreviewSources:
    """Independent laser and semantic observations, before Nav2 inflation."""

    laser: tuple = ()
    semantic: tuple = ()


@dataclass
class RenderOptions:
    """Display settings; the robot geometry always comes from the controller."""

    image_width: int = 960
    image_height: int = 960
    pixels_per_meter: float = 140.0
    grid_spacing: float = 0.5
    line_width: int = 2
    selected_line_width: int = 5
    trajectory_alpha: float = 0.65
    show_rejected: bool = True
    show_cost_pruned: bool = False
    axis_length: float = 0.45
    font_size: int = 17
    colors: tuple = field(default_factory=lambda: DEFAULT_COLORS)
    selected_color: tuple = (0, 180, 65)
    global_path_color: tuple = (190, 40, 140)
    tube_radius: float = 0.30
    tube_line_width: int = 2
    goal_circle_radius: float = 0.20
    costmap_as_points: bool = True
    costmap_colormap: str = 'classic'
    costmap_point_size: int = 3
    costmap_point_color: tuple = (35, 40, 50)
    laser_color: tuple = (210, 50, 40)
    semantic_color: tuple = (30, 100, 210)
    source_cost_colors: bool = False
    source_class_costs: tuple = (0, 0, 60, 30, 90, 0, 90)
    source_laser_cost: int = 100
    free_color: tuple = (238, 241, 245)
    inflated_color: tuple = (245, 185, 105)
    critical_color: tuple = (245, 100, 45)
    lethal_color: tuple = (0, 0, 0)
    unknown_color: tuple = (110, 115, 125)
    background_color: tuple = (250, 250, 250)

    def validate(self):
        """Reject invalid geometry and unbounded image allocations."""
        if not (320 <= self.image_width <= 4096 and 320 <= self.image_height <= 4096):
            raise ValueError('Image dimensions must be between 320 and 4096 pixels')
        for name in ('pixels_per_meter', 'grid_spacing', 'axis_length'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not 0 <= self.trajectory_alpha <= 1:
            raise ValueError('trajectory_alpha must be between 0 and 1')
        if (not math.isfinite(self.tube_radius) or not 0 <= self.tube_radius <= 10.0
                or not 1 <= self.tube_line_width <= 10):
            raise ValueError('Invalid path tube radius or line width')
        if not math.isfinite(self.goal_circle_radius) or not 0 <= self.goal_circle_radius <= 10.0:
            raise ValueError('Goal circle radius must be finite and between 0 and 10 meters')
        if not (1 <= self.line_width <= 30 and 1 <= self.selected_line_width <= 30
                and 8 <= self.font_size <= 72):
            raise ValueError('Invalid line width or font size')
        if not isinstance(self.costmap_as_points, bool) or not 1 <= self.costmap_point_size <= 15:
            raise ValueError('Invalid costmap point display settings')
        if self.costmap_colormap not in ('classic', 'jet'):
            raise ValueError('costmap_colormap must be classic or jet')
        if (not isinstance(self.source_cost_colors, bool)
                or len(self.source_class_costs) != 7
                or any(not isinstance(v, int) or not 0 <= v <= 100
                       for v in self.source_class_costs)
                or not isinstance(self.source_laser_cost, int)
                or not 0 <= self.source_laser_cost <= 100):
            raise ValueError('Point class costs must be integers in [0, 100]')
        colors = self.colors + (self.selected_color, self.global_path_color,
                                self.costmap_point_color, self.free_color,
                                self.laser_color, self.semantic_color,
                                self.inflated_color, self.critical_color,
                                self.lethal_color, self.unknown_color,
                                self.background_color)
        if len(self.colors) != len(FAMILIES) or any(
                len(color) != 3 or any(not 0 <= value <= 255 for value in color)
                for color in colors):
            raise ValueError('Colors must contain three RGB values in [0, 255]')


def world_to_base(point, robot_pose):
    """Rotate/translate a costmap-frame point into the snapshot robot frame."""
    x, y, yaw = robot_pose
    dx, dy = point[0] - x, point[1] - y
    return (math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy)


def base_to_pixel(point, options):
    """ROS x points up, ROS y points left; base_link is at the image center."""
    return (options.image_width / 2 - point[1] * options.pixels_per_meter,
            options.image_height / 2 - point[0] * options.pixels_per_meter)


def costmap_affine(frame, options):
    """Map output pixels to input cell centers, including both frame rotations."""
    rx, ry, yaw = frame.robot_pose
    ox, oy, origin_yaw = frame.costmap_origin
    resolution = frame.costmap_resolution

    def source(u, v):
        bx = (options.image_height / 2 - v) / options.pixels_per_meter
        by = (options.image_width / 2 - u) / options.pixels_per_meter
        wx = rx + math.cos(yaw) * bx - math.sin(yaw) * by
        wy = ry + math.sin(yaw) * bx + math.cos(yaw) * by
        dx, dy = wx - ox, wy - oy
        return ((math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy) / resolution,
                (-math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy) / resolution)

    # Pillow's affine coordinates refer to pixel edges; cell (0,0) occupies
    # [0,1)x[0,1). Its center is therefore (0.5,0.5), not (0,0).
    p0, pu, pv = source(0, 0), source(1, 0), source(0, 1)
    return (pu[0] - p0[0], pv[0] - p0[0], p0[0],
            pu[1] - p0[1], pv[1] - p0[1], p0[1])


@lru_cache(maxsize=16)
def _font(size):
    try:
        return ImageFont.truetype('DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default()


def _arrow(draw, start, end, color, width=3):
    draw.line((start, end), fill=color, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    head = 10
    draw.polygon((end,
                  (end[0] - head * math.cos(angle - 0.45),
                   end[1] - head * math.sin(angle - 0.45)),
                  (end[0] - head * math.cos(angle + 0.45),
                   end[1] - head * math.sin(angle + 0.45))), fill=color)


def _dashed_line(draw, points, color, width):
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance == 0:
            continue
        for offset in range(0, int(math.ceil(distance)), 12):
            a, b = offset / distance, min(offset + 6, distance) / distance
            draw.line(((start[0] + a * (end[0] - start[0]),
                        start[1] + a * (end[1] - start[1])),
                       (start[0] + b * (end[0] - start[0]),
                        start[1] + b * (end[1] - start[1]))), fill=color, width=width)


def _costmap_size(frame):
    """Validate the full grid shared by raster and cell-center views."""
    width, height = frame.costmap_size
    if (width <= 0 or height <= 0 or len(frame.costmap_data) != width * height
            or not math.isfinite(frame.costmap_resolution)
            or frame.costmap_resolution <= 0):
        raise ValueError('Invalid costmap dimensions, resolution or data')
    return width, height


@lru_cache(maxsize=255)
def jet_color(cost):
    """Map raw Nav2 costs 0..254 to a fixed blue/cyan/green/yellow/red scale."""
    t = cost / 254.0
    return tuple(round(255 * max(0.0, min(1.0, 1.5 - abs(4 * t - center))))
                 for center in (3, 2, 1))


def point_cost_color(cost, options):
    """Color a source point's own 0..100 class cost without costmap lookup."""
    return options.lethal_color if cost >= 100 else jet_color(round(254 * cost / 100))


def _costmap(frame, options):
    width, height = _costmap_size(frame)
    source = Image.frombytes('P', (width, height), frame.costmap_data)
    palette = []
    for cost in range(256):
        if cost == 255:
            color = options.unknown_color
        elif cost == 254:
            color = options.lethal_color
        elif options.costmap_colormap == 'jet':
            color = jet_color(cost)
        elif cost == 253:
            color = options.critical_color
        else:
            t = cost / 252.0
            color = tuple(round(a + t * (b - a))
                          for a, b in zip(options.free_color, options.inflated_color))
        palette.extend(color)
    source.putpalette(palette)
    return source.convert('RGB').transform(
        (options.image_width, options.image_height), Image.AFFINE,
        costmap_affine(frame, options), resample=Image.NEAREST,
        fillcolor=options.background_color)


@lru_cache(maxsize=4)
def _costmap_cells(data):
    """Reuse occupied cell indices until the complete costmap data changes."""
    cells = tuple((index, cost == 255) for index, cost in enumerate(data) if cost)
    return cells, sum(not unknown for _, unknown in cells)


def _costmap_points(frame, options, image=None):
    """Draw every nonfree master-costmap cell at its center, without downsampling."""
    width, _ = _costmap_size(frame)
    if image is None:
        image = Image.new('RGB', (options.image_width, options.image_height),
                          options.background_color)
    draw = ImageDraw.Draw(image)
    ox, oy, yaw = frame.costmap_origin
    origin_x, origin_y = world_to_base((ox, oy), frame.robot_pose)
    angle = yaw - frame.robot_pose[2]
    cosine, sine = math.cos(angle), math.sin(angle)
    scale = frame.costmap_resolution * options.pixels_per_meter
    u0 = options.image_width / 2 - origin_y * options.pixels_per_meter
    v0 = options.image_height / 2 - origin_x * options.pixels_per_meter
    ux, uy, vx, vy = -sine * scale, -cosine * scale, -cosine * scale, sine * scale
    radius = options.costmap_point_size // 2
    cells, _ = _costmap_cells(frame.costmap_data)
    for index, unknown in cells:
        x, y = index % width + 0.5, index // width + 0.5
        u, v = round(u0 + ux * x + uy * y), round(v0 + vx * x + vy * y)
        color = (options.unknown_color if unknown else
                 options.lethal_color if frame.costmap_data[index] == 254 else
                 jet_color(frame.costmap_data[index]) if options.costmap_colormap == 'jet' else
                 options.costmap_point_color)
        draw.rectangle((u - radius, v - radius,
                        u - radius + options.costmap_point_size - 1,
                        v - radius + options.costmap_point_size - 1), fill=color)
    return image


@lru_cache(maxsize=2)
def _tube_outline(points, size, radius, width):
    """Outline the union of round capsules, including joins and endpoint caps."""
    mask = Image.new('L', size, 0)
    draw = ImageDraw.Draw(mask)
    for start, end in zip(points, points[1:]):
        draw.line((start, end), fill=255, width=2 * radius + 1)
    for x, y in points:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=255)
    # Taking the boundary of the union avoids drawing false internal edges at
    # bends, overlapping segments and self-intersections. The interior stays clear.
    inner = mask
    # Separable erosion avoids Pillow's expensive square minimum filter.
    # White padding keeps a clipped tube open at the viewport boundaries.
    for dx, dy in ((1, 0), (0, 1)):
        source = inner
        for distance in range(1, width + 1):
            for sign in (-1, 1):
                shifted = Image.new('L', size, 255)
                shifted.paste(source, (sign * distance * dx, sign * distance * dy))
                inner = ImageChops.darker(inner, shifted)
    return ImageChops.subtract(mask, inner)


def render_preview(frame, options, status='', background=None, global_path=(), sources=None):
    """Draw live context even when no fresh MPC predictions are available."""
    options.validate()
    view = frame if frame is not None else background
    image = (_costmap(view, options) if sources is None and view is not None
             and not options.costmap_as_points else
             Image.new('RGB', (options.image_width, options.image_height),
                       options.background_color))
    draw = ImageDraw.Draw(image)
    font = _font(options.font_size)
    spacing = max(1, round(options.grid_spacing * options.pixels_per_meter))
    cx, cy = options.image_width / 2, options.image_height / 2
    for center, limit, vertical in ((cx, options.image_width, True),
                                    (cy, options.image_height, False)):
        for coordinate in range(round(center) % spacing, limit, spacing):
            line = ((coordinate, 0, coordinate, options.image_height) if vertical else
                    (0, coordinate, options.image_width, coordinate))
            draw.line(line, fill=(210, 213, 219), width=1)

    if sources is None and view is not None and options.costmap_as_points:
        image = _costmap_points(view, options, image)
        draw = ImageDraw.Draw(image)

    if sources is not None:
        radius = options.costmap_point_size // 2
        for points, default_color, laser in ((sources.semantic, options.semantic_color, False),
                                              (sources.laser, options.laser_color, True)):
            for point in points:
                color = default_color
                if options.source_cost_colors:
                    cost = (options.source_laser_cost if laser else
                            options.source_class_costs[point[2]] if len(point) >= 3 else None)
                    if cost is not None:
                        color = point_cost_color(cost, options)
                u, v = (round(value) for value in base_to_pixel(point, options))
                draw.rectangle((u - radius, v - radius,
                                u - radius + options.costmap_point_size - 1,
                                v - radius + options.costmap_point_size - 1), fill=color)

    if global_path:
        points = [base_to_pixel(point, options) for point in global_path]
        if options.tube_radius > 0:
            radius = max(1, round(options.tube_radius * options.pixels_per_meter))
            outline = _tube_outline(tuple(points), image.size, radius, options.tube_line_width)
            image.paste((255, 255, 255), (0, 0), outline)
        if len(points) >= 2:
            draw.line(points, fill=(255, 255, 255), width=options.line_width + 5)
            draw.line(points, fill=options.global_path_color, width=options.line_width + 1)

    if frame is not None:
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        traces = ImageDraw.Draw(overlay)
        winner = None
        for candidate in frame.candidates:
            if candidate.candidate_id == frame.selected_id:
                winner = candidate
                continue
            if not 0 <= candidate.family < len(FAMILIES):
                continue
            if candidate.score_status == 2 and not options.show_rejected:
                continue
            if candidate.score_status == 0 and not options.show_cost_pruned:
                continue
            points = [base_to_pixel(world_to_base(pose, frame.robot_pose), options)
                      for pose in candidate.poses]
            color = options.colors[candidate.family] + (round(255 * options.trajectory_alpha),)
            if len(points) >= 2:
                if candidate.score_status != 1:
                    _dashed_line(traces, points, color, options.line_width)
                else:
                    traces.line(points, fill=color, width=options.line_width)
        image = Image.alpha_composite(image.convert('RGBA'), overlay).convert('RGB')
        draw = ImageDraw.Draw(image)
        if winner is not None:
            points = [base_to_pixel(world_to_base(pose, frame.robot_pose), options)
                      for pose in winner.poses]
            if len(points) >= 2:
                draw.line(points, fill=(255, 255, 255), width=options.selected_line_width + 4)
                draw.line(points, fill=options.selected_color, width=options.selected_line_width)
                end = points[-1]
                draw.ellipse((end[0] - 4, end[1] - 4, end[0] + 4, end[1] + 4),
                             fill=options.selected_color)
    if global_path and options.goal_circle_radius > 0:
        end_x, end_y = base_to_pixel(global_path[-1], options)
        radius = options.goal_circle_radius * options.pixels_per_meter
        draw.ellipse((end_x - radius, end_y - radius, end_x + radius, end_y + radius),
                     outline=(0, 180, 65), width=options.line_width + 1)

    if view is not None:
        polygon = [base_to_pixel(point, options) for point in view.footprint]
        if len(polygon) >= 3:
            draw.line(polygon + [polygon[0]], fill=(255, 255, 255), width=7)
            draw.line(polygon + [polygon[0]], fill=(20, 30, 45), width=3)

    origin = base_to_pixel((0, 0), options)
    _arrow(draw, origin, base_to_pixel((options.axis_length, 0), options), (195, 35, 35), 2)
    _arrow(draw, origin, base_to_pixel((0, options.axis_length), options), (35, 130, 55), 2)
    draw.text((origin[0] + 8, origin[1] + 8), 'base_link', fill=(20, 30, 45), font=font)
    draw.text(base_to_pixel((options.axis_length + 0.08, 0), options), '+x',
              fill=(195, 35, 35), font=font)
    draw.text(base_to_pixel((0, options.axis_length + 0.15), options), '+y',
              fill=(35, 130, 55), font=font)
    title = ('MPC - sources before inflation / base_link' if sources is not None else
             'MPC - top view / base_link')
    detail = status or 'Waiting for MPC data'
    if frame is not None:
        detail = (f'{frame.generated_count} candidates | {len(frame.candidates)} exported | '
                  f'horizon {frame.time_steps * frame.model_dt:.2f} s')
        if status:
            detail += ' | ' + status
    draw.rectangle((0, 0, options.image_width, 70), fill=options.background_color)
    draw.text((16, 10), title, fill=(20, 30, 45), font=font)
    draw.text((16, 36), detail, fill=(60, 65, 75), font=font)
    source_costs = sources is not None and options.source_cost_colors
    if source_costs or (sources is None and options.costmap_colormap == 'jet'):
        x, y, width = options.image_width - 270, 82, 240
        draw.rectangle((x - 8, y - 6, x + width + 18, y + 64),
                       fill=options.background_color)
        top = 99 if source_costs else 253
        title = 'Jet points 0-99 | black:100' if source_costs else 'Jet 0-253 | black: 254'
        draw.text((x, y), title, fill=(30, 35, 45), font=font)
        for column in range(width):
            value = round(top * column / (width - 1))
            color = point_cost_color(value, options) if source_costs else jet_color(value)
            draw.line((x + column, y + 24, x + column, y + 36), fill=color)
        for cost in ((0, 25, 50, 75, 99) if source_costs else (0, 64, 128, 192, 253)):
            tick = x + (width - 1) * cost / top
            draw.text((tick - (0 if cost == 0 else 14), y + 40), str(cost),
                      fill=(30, 35, 45), font=font)
    legend_y = options.image_height - 172
    draw.rectangle((0, legend_y, options.image_width, options.image_height),
                   fill=options.background_color)
    for index, label in enumerate(LABELS):
        x = 16 + (index % 3) * (options.image_width // 3)
        y = legend_y + 10 + (index // 3) * 27
        draw.line((x, y + 8, x + 24, y + 8), fill=options.colors[index], width=3)
        draw.text((x + 30, y), label, fill=(30, 35, 45), font=font)
    x, y = 16 + 2 * (options.image_width // 3), legend_y + 64
    draw.line((x, y + 8, x + 24, y + 8), fill=options.selected_color, width=5)
    draw.text((x + 30, y), 'Selected', fill=(30, 35, 45), font=font)
    layer_label = ('Jet: point class cost' if source_costs else
                   'Laser red | CV blue' if sources is not None else
                   'Dashed: rejected / cost-pruned')
    draw.text((16, legend_y + 94), layer_label,
              fill=(60, 65, 75), font=font)
    x, y = options.image_width // 2, legend_y + 94
    draw.line((x, y + 8, x + 24, y + 8), fill=options.global_path_color, width=3)
    path_label = 'Global path'
    if options.tube_radius > 0:
        path_label += f' | tube r={options.tube_radius:g} m'
    draw.text((x + 30, y), path_label, fill=(30, 35, 45), font=font)
    if frame is not None:
        steering = (f'initial v={frame.initial_velocity:.2f} m/s  '
                    f'initial steering={math.degrees(frame.initial_steering):.1f}°')
        if frame.selected_id >= 0:
            steering += (f' | command v={frame.command_velocity:.2f} m/s  '
                         f'command steering={math.degrees(frame.command_steering):.1f}°')
        else:
            steering += ' | no valid trajectory'
        draw.text((16, legend_y + 119), steering, fill=(30, 35, 45), font=font)
    if sources is not None:
        map_label = (f'Laser: {len(sources.laser)} | Local points: {len(sources.semantic)} | '
                     + ('point costs 0-100' if source_costs else 'view without inflation'))
    elif options.costmap_as_points:
        count = _costmap_cells(view.costmap_data)[1] if view is not None else 0
        map_label = (f'{count} cells with cost > 0 | Jet=MPC costs, black=lethal, gray=unknown'
                     if options.costmap_colormap == 'jet' else
                     f'{count} cells with cost > 0 | dark points=MPC costs, gray=unknown')
    else:
        map_label = ('Jet: 0-253 | black=254 lethal, gray=255 unknown'
                     if options.costmap_colormap == 'jet' else
                     'Costmap: light=free, orange=inflation, dark=obstacle')
    draw.text((16, legend_y + 145), f'Grid: {options.grid_spacing:g} m | ' + map_label,
              fill=(60, 65, 75), font=font)
    return image
