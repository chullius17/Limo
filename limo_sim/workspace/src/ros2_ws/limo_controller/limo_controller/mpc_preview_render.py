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
import math

from PIL import Image, ImageDraw, ImageFont


FAMILIES = ('braking', 'nominal', 'constant_curvature',
            'perturbed_nominal', 'broad_exploration')
LABELS = ('Frenata', 'Nominale', 'Curvatura costante',
          'Perturbata', 'Esplorazione ampia')
DEFAULT_COLORS = ((220, 75, 65), (35, 105, 220), (145, 75, 190),
                  (25, 165, 170), (220, 140, 35))


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
    wheel_length: float = 0.13
    wheel_width: float = 0.05
    steering_arrow_length: float = 0.45
    axis_length: float = 0.45
    font_size: int = 17
    model_inset_enabled: bool = True
    model_inset_pixels_per_meter: float = 600.0
    colors: tuple = field(default_factory=lambda: DEFAULT_COLORS)
    selected_color: tuple = (0, 180, 65)
    free_color: tuple = (238, 241, 245)
    inflated_color: tuple = (245, 185, 105)
    critical_color: tuple = (245, 100, 45)
    lethal_color: tuple = (100, 25, 30)
    unknown_color: tuple = (110, 115, 125)
    background_color: tuple = (250, 250, 250)

    def validate(self):
        """Reject invalid geometry and unbounded image allocations."""
        if not (320 <= self.image_width <= 4096 and 320 <= self.image_height <= 4096):
            raise ValueError('Image dimensions must be between 320 and 4096 pixels')
        for name in ('pixels_per_meter', 'grid_spacing', 'wheel_length',
                     'wheel_width', 'steering_arrow_length', 'axis_length',
                     'model_inset_pixels_per_meter'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not 0 <= self.trajectory_alpha <= 1:
            raise ValueError('trajectory_alpha must be between 0 and 1')
        if not (1 <= self.line_width <= 30 and 1 <= self.selected_line_width <= 30
                and 8 <= self.font_size <= 72):
            raise ValueError('Invalid line width or font size')
        colors = self.colors + (self.selected_color, self.free_color,
                                self.inflated_color, self.critical_color,
                                self.lethal_color, self.unknown_color,
                                self.background_color)
        if len(self.colors) != 5 or any(
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


def _costmap(frame, options):
    width, height = frame.costmap_size
    if (width <= 0 or height <= 0 or len(frame.costmap_data) != width * height
            or not math.isfinite(frame.costmap_resolution)
            or frame.costmap_resolution <= 0):
        raise ValueError('Invalid costmap dimensions, resolution or data')
    source = Image.frombytes('P', (width, height), frame.costmap_data)
    palette = []
    for cost in range(256):
        if cost == 255:
            color = options.unknown_color
        elif cost == 254:
            color = options.lethal_color
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


def _model_inset(frame, options):
    """Enlarge the same footprint and virtual wheels without changing map scale."""
    inset = Image.new('RGB', (300, 340), options.background_color)
    draw = ImageDraw.Draw(inset)
    font = _font(14)
    scale = options.model_inset_pixels_per_meter

    def pixel(x, y):
        return (150 - y * scale, 190 - x * scale)

    draw.rectangle((0, 0, 299, 339), outline=(90, 95, 105), width=2)
    draw.text((12, 10), 'Modello bicicletta (ingrandito)', fill=(20, 30, 45), font=font)
    polygon = [pixel(x, y) for x, y in frame.footprint]
    if len(polygon) >= 3:
        draw.line(polygon + [polygon[0]], fill=(20, 30, 45), width=3)
    rear, front = -frame.rear_axle_to_base, frame.wheelbase - frame.rear_axle_to_base
    draw.line((pixel(rear, 0), pixel(front, 0)), fill=(90, 95, 105), width=3)
    for x, steering, label in ((rear, 0.0, 'posteriore'),
                               (front, frame.initial_steering, 'anteriore')):
        polygon = []
        for dx, dy in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
            a, b = dx * options.wheel_length / 2, dy * options.wheel_width / 2
            polygon.append(pixel(x + a * math.cos(steering) - b * math.sin(steering),
                                 a * math.sin(steering) + b * math.cos(steering)))
        draw.polygon(polygon, fill=(30, 30, 35))
        u, v = pixel(x, 0)
        draw.ellipse((u - 4, v - 4, u + 4, v + 4), fill=(255, 255, 255))
        draw.text((u + 30, v - 8), label, fill=(30, 35, 45), font=font)
    for steering, color, length in (
            (frame.initial_steering, (220, 130, 0), 0.12),
            (frame.command_steering, (180, 40, 160), 0.09)):
        if color == (180, 40, 160) and frame.selected_id < 0:
            continue
        _arrow(draw, pixel(front, 0),
               pixel(front + length * math.cos(steering), length * math.sin(steering)), color)
    draw.ellipse((146, 186, 154, 194), fill=(195, 35, 35))
    draw.text((14, 185), 'base_link', fill=(195, 35, 35), font=font)
    draw.text((12, 317), f'L={frame.wheelbase:.2f} m | asse-base={frame.rear_axle_to_base:.2f} m',
              fill=(30, 35, 45), font=font)
    return inset


def render_preview(frame, options, status=''):
    """Draw one coherent snapshot; a missing/stale frame displays no trajectories."""
    options.validate()
    image = (_costmap(frame, options) if frame is not None else
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
        polygon = [base_to_pixel(point, options) for point in frame.footprint]
        if len(polygon) >= 3:
            draw.line(polygon + [polygon[0]], fill=(20, 30, 45), width=3)

        rear = -frame.rear_axle_to_base
        front = rear + frame.wheelbase
        draw.line((base_to_pixel((rear, 0), options), base_to_pixel((front, 0), options)),
                  fill=(40, 40, 40), width=3)
        for wheel_x, steering in ((rear, 0.0), (front, frame.initial_steering)):
            vertices = []
            for dx, dy in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
                x, y = dx * options.wheel_length / 2, dy * options.wheel_width / 2
                vertices.append(base_to_pixel((wheel_x + math.cos(steering) * x
                                               - math.sin(steering) * y,
                                               math.sin(steering) * x
                                               + math.cos(steering) * y), options))
            draw.polygon(vertices, fill=(30, 30, 35))
            u, v = base_to_pixel((wheel_x, 0), options)
            draw.ellipse((u - 3, v - 3, u + 3, v + 3), fill=(255, 255, 255))
        start = base_to_pixel((front, 0), options)
        for steering, color, length in (
                (frame.initial_steering, (220, 130, 0), options.steering_arrow_length),
                (frame.command_steering, (180, 40, 160), options.steering_arrow_length * 0.8)):
            if color == (180, 40, 160) and frame.selected_id < 0:
                continue
            end = base_to_pixel((front + length * math.cos(steering),
                                 length * math.sin(steering)), options)
            _arrow(draw, start, end, color)

    # The main view retains the true metric scale; an optional inset makes
    # the small footprint and equivalent steering wheel easy to inspect.
    origin = base_to_pixel((0, 0), options)
    _arrow(draw, origin, base_to_pixel((options.axis_length, 0), options), (195, 35, 35), 2)
    _arrow(draw, origin, base_to_pixel((0, options.axis_length), options), (35, 130, 55), 2)
    draw.text((origin[0] + 8, origin[1] + 8), 'base_link', fill=(20, 30, 45), font=font)
    draw.text(base_to_pixel((options.axis_length + 0.08, 0), options), '+x',
              fill=(195, 35, 35), font=font)
    draw.text(base_to_pixel((0, options.axis_length + 0.15), options), '+y',
              fill=(35, 130, 55), font=font)
    if frame is not None and options.model_inset_enabled and options.image_width >= 640:
        image.paste(_model_inset(frame, options), (options.image_width - 316, 86))

    title = 'MPC - vista dall\'alto / base_link'
    detail = status or 'In attesa di uno snapshot MPC'
    if frame is not None:
        detail = (f'{frame.generated_count} candidati | {len(frame.candidates)} esportati | '
                  f'orizzonte {frame.time_steps * frame.model_dt:.2f} s')
    draw.rectangle((0, 0, options.image_width, 70), fill=options.background_color)
    draw.text((16, 10), title, fill=(20, 30, 45), font=font)
    draw.text((16, 36), detail, fill=(60, 65, 75), font=font)
    legend_y = options.image_height - 170
    draw.rectangle((0, legend_y, options.image_width, options.image_height),
                   fill=options.background_color)
    for index, label in enumerate(LABELS):
        x = 16 + (index % 3) * (options.image_width // 3)
        y = legend_y + 10 + (index // 3) * 27
        draw.line((x, y + 8, x + 24, y + 8), fill=options.colors[index], width=3)
        draw.text((x + 30, y), label, fill=(30, 35, 45), font=font)
    x, y = 16 + 2 * (options.image_width // 3), legend_y + 37
    draw.line((x, y + 8, x + 24, y + 8), fill=options.selected_color, width=5)
    draw.text((x + 30, y), 'Scelta', fill=(30, 35, 45), font=font)
    draw.text((16, legend_y + 67), 'Tratteggio: scartata / potata; ruote: modello bicicletta',
              fill=(60, 65, 75), font=font)
    draw.text((16, legend_y + 92), 'Sterzo iniziale: arancio | primo comando: viola',
              fill=(60, 65, 75), font=font)
    if frame is not None:
        steering = (f'v0={frame.initial_velocity:.2f} m/s  '
                    f'delta0={math.degrees(frame.initial_steering):.1f} deg')
        if frame.selected_id >= 0:
            steering += (f' | v1={frame.command_velocity:.2f} m/s  '
                         f'delta1={math.degrees(frame.command_steering):.1f} deg')
        else:
            steering += ' | nessuna traiettoria valida'
        draw.text((16, legend_y + 118), steering, fill=(30, 35, 45), font=font)
    draw.text((16, legend_y + 143), f'Griglia: {options.grid_spacing:g} m | '
              'costmap: chiaro=libero, arancio=inflazione, scuro=ostacolo',
              fill=(60, 65, 75), font=font)
    return image
