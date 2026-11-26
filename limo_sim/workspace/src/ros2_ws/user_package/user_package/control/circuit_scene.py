"""Read the circuit texture and obstacle footprints from Gazebo Classic SDF.

This viewer supports planar model/link/visual poses and box, cylinder and
sphere visuals. It deliberately does not render robot models, lights or shadows.
"""

import ast
from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in metres and radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def compose(self, child):
        """Apply a child pose in this pose's coordinate frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * child.x - s * child.y,
                      self.y + s * child.x + c * child.y, self.yaw + child.yaw)

    def point(self, x, y):
        """Transform a point into the parent frame."""
        pose = self.compose(Pose2D(x, y))
        return pose.x, pose.y

    @classmethod
    def from_message(cls, pose):
        """Read a geometry_msgs/Pose without importing ROS in this module."""
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite Gazebo pose')
        norm = math.sqrt(sum(v * v for v in values[2:]))
        if norm < 1e-12:
            raise ValueError('Invalid Gazebo quaternion')
        x, y, z, w = (v / norm for v in values[2:])
        return cls(values[0], values[1], math.atan2(
            2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def pose_from_link_states(msg, link_name):
    """Select one exact scoped link in a complete Gazebo world-pose snapshot."""
    if len(msg.name) != len(msg.pose) or len(set(msg.name)) != len(msg.name):
        raise ValueError('Malformed Gazebo link states')
    if link_name not in msg.name:
        return None
    return Pose2D.from_message(msg.pose[msg.name.index(link_name)])


@dataclass(frozen=True)
class Shape:
    """A local footprint and flat RGB colour belonging to a model."""

    points: tuple
    color: str


@dataclass(frozen=True)
class Model:
    """Geometry relative to a replaceable model pose."""

    name: str
    pose: Pose2D
    shapes: tuple


@dataclass(frozen=True)
class Ground:
    """Textured box top face with Gazebo's unit-box UV orientation."""

    model_name: str
    model_pose: Pose2D
    local_pose: Pose2D
    size: tuple
    texture: Path

    def texture_corners(self, model_pose=None):
        """Return world points for image top-left, top-right, bottom-left."""
        pose = (model_pose or self.model_pose).compose(self.local_pose)
        x, y = self.size[0] / 2, self.size[1] / 2
        # Gazebo box UV: image rows point toward -X, columns toward -Y.
        return tuple(pose.point(a, b) for a, b in ((x, y), (x, -y), (-x, y)))


@dataclass
class CircuitScene:
    """Parsed static geometry; live data replaces model poses only."""

    ground: Ground
    models: dict
    warnings: list = field(default_factory=list)
    world_name: str = 'default'

    def footprints(self, poses=None):
        """Yield model name, world polygon, colour; hide absent live models."""
        for name, model in self.models.items():
            if poses is not None and name not in poses:
                continue
            pose = model.pose if poses is None else poses[name]
            for shape in model.shapes:
                yield name, tuple(pose.point(*p) for p in shape.points), shape.color

    def unknown_models(self, poses, excluded=('limo', 'robot', 'ground_plane')):
        """Report live objects without geometry instead of guessing their size."""
        return sorted(set(poses) - set(self.models) - {self.ground.model_name} - set(excluded))


def _pose(element):
    pose = element.find('pose')
    if pose is None or not pose.text:
        return Pose2D()
    if pose.get('relative_to') or pose.get('frame'):
        raise ValueError('Named SDF pose frames are not supported')
    values = [float(v) for v in pose.text.split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected six finite SDF pose values')
    if abs(values[3]) > 1e-5 or abs(values[4]) > 1e-5:
        raise ValueError('Only planar SDF geometry is supported')
    return Pose2D(values[0], values[1], values[5])


def _resolve(uri, roots):
    if not uri.startswith('model://'):
        raise ValueError('Expected a local model:// URI: ' + uri)
    relative = uri[len('model://'):]
    for root in roots:
        path = root / relative
        if path.exists():
            return path
    raise FileNotFoundError('Gazebo asset not found: ' + uri)


def _color(visual):
    diffuse = visual.findtext('material/diffuse')
    if diffuse:
        rgb = [max(0, min(255, round(float(v) * 255))) for v in diffuse.split()[:3]]
        if len(rgb) == 3:
            return '#{:02x}{:02x}{:02x}'.format(*rgb)
    name = visual.findtext('material/script/name', '')
    return {'Gazebo/White': '#f2f2f2', 'Gazebo/Grey': '#999999',
            'Gazebo/Red': '#cc3333', 'Gazebo/Blue': '#3366cc',
            'Gazebo/Black': '#333333'}.get(name, '#aaaaaa')


def _outline(geometry):
    if geometry.find('box') is not None:
        size = [float(v) for v in geometry.findtext('box/size').split()]
        if len(size) != 3 or any(not math.isfinite(v) or v <= 0 for v in size):
            raise ValueError('Invalid box size')
        x, y = size[0] / 2, size[1] / 2
        return ((-x, -y), (x, -y), (x, y), (-x, y))
    for kind in ('cylinder', 'sphere'):
        if geometry.find(kind) is not None:
            radius = float(geometry.findtext(kind + '/radius'))
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError('Invalid radius')
            return tuple((radius * math.cos(i * math.tau / 64),
                          radius * math.sin(i * math.tau / 64)) for i in range(64))
    raise ValueError('Only box, cylinder and sphere visuals are supported')


def world_sdf_from_response(response):
    """Decode the GzString DebugString returned by `gz topic -r world_sdf`."""
    literal = r'"(?:[^"\\]|\\.)*"'
    match = re.fullmatch(r'\s*data:\s*((?:' + literal + r'\s*)+)\s*', response)
    if match is None:
        raise ValueError('Gazebo did not return a world description')
    # DebugString uses C escapes, including octal-escaped UTF-8 bytes.
    chunks = re.findall(literal, match.group(1))
    return b''.join(ast.literal_eval('b' + chunk) for chunk in chunks).decode('utf-8')


def load_world_scene(world_path, model_paths=(), excluded=('limo', 'robot')):
    """Load inline/includes and saved model poses from the selected world file."""
    world_path = Path(world_path).expanduser().resolve()
    return load_scene_sdf(world_path.read_text(encoding='utf-8'), world_path,
                          model_paths=model_paths, excluded=excluded)


def load_scene_sdf(text, source_path, model_paths=(), excluded=('limo', 'robot'), ground=None):
    """Parse a live snapshot using original resource paths and optional fixed ground."""
    world_path = Path(source_path).expanduser().resolve()
    world = ET.fromstring(text).find('world')
    if world is None:
        raise ValueError('The SDF file has no world')
    roots = [Path(p).expanduser() for p in model_paths]
    roots += [world_path.parent.parent / 'models', Path.home() / '.gazebo/models']
    saved, warnings = {}, []
    state = world.find('state')
    if state is not None:
        for model in state.findall('model'):
            if model.get('name') not in excluded:
                try:
                    saved[model.get('name')] = _pose(model)
                except ValueError as exc:
                    warnings.append('{}: {}'.format(model.get('name'), exc))
    entries = []
    for element in world:
        if element.tag == 'model':
            if element.get('name') in excluded or element.find('.//joint') is not None:
                continue
            try:
                entries.append((element.get('name'), element, _pose(element)))
            except ValueError as exc:
                warnings.append('{}: {}'.format(element.get('name'), exc))
        elif element.tag == 'include':
            uri = element.findtext('uri', '')
            try:
                directory = _resolve(uri, roots)
                model = ET.parse(str(directory / 'model.sdf')).getroot().find('model')
                if model is None:
                    raise ValueError('Included SDF has no model')
                name = element.findtext('name') or model.get('name')
                # SDF include pose overrides the included model pose.
                pose = _pose(element) if element.find('pose') is not None else _pose(model)
                entries.append((name, model, pose))
            except (OSError, ValueError, ET.ParseError) as exc:
                warnings.append(str(exc))
    preserved_ground, models = ground, {}
    for name, model, initial_pose in entries:
        if preserved_ground is not None and name == preserved_ground.model_name:
            continue
        if name in excluded or model.find('.//joint') is not None:
            continue
        model_pose = saved.get(name, initial_pose)
        shapes = []
        for link in model.findall('link'):
            try:
                link_pose = _pose(link)
            except ValueError as exc:
                warnings.append('{}: {}'.format(name, exc))
                continue
            for visual in link.findall('visual'):
                try:
                    local = link_pose.compose(_pose(visual))
                    geometry = visual.find('geometry')
                    if geometry is None:
                        continue
                    if visual.findtext('material/script/name') == 'vrc/circuit':
                        size = tuple(float(v) for v in geometry.findtext('box/size').split())
                        texture = _resolve(
                            'model://plane_test/materials/textures/circuit.png', roots)
                        ground = Ground(name, model_pose, local, size[:2], texture)
                        continue
                    points = tuple(local.point(*p) for p in _outline(geometry))
                    shapes.append(Shape(points, _color(visual)))
                except (OSError, ValueError, TypeError, AttributeError) as exc:
                    warnings.append('{}: {}'.format(name, exc))
        if shapes:
            models[name] = Model(name, model_pose, tuple(shapes))
    if ground is None:
        raise ValueError('No vrc/circuit textured plane found in ' + str(world_path))
    return CircuitScene(ground, models, warnings, world.get('name', 'default'))
