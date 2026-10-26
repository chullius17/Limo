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

"""Exercise robot/map frame alignment and compressed preview contents without ROS."""

from io import BytesIO
import math
import unittest

from PIL import Image

from limo_controller.mpc_preview_render import (
    Candidate, PreviewBackground, PreviewFrame, PreviewSources, RenderOptions,
    _costmap, _costmap_points,
    base_to_pixel,
    render_preview, world_to_base,
)


def sample_frame():
    """Create an obstacle snapshot and one trajectory of each family."""
    costs = bytearray(100 * 100)
    for y in range(45, 55):
        for x in range(68, 78):
            costs[y * 100 + x] = 254
    candidates = tuple(
        Candidate(index, index, 1, 10.0 + index,
                  tuple((t * 0.05, (index - 2) * 0.015 * t, 0.0) for t in range(21)))
        for index in range(5))
    return PreviewFrame(
        robot_pose=(0.0, 0.0, 0.0),
        footprint=((-0.161, -0.110), (-0.161, 0.110),
                   (0.161, 0.110), (0.161, -0.110)),
        costmap_size=(100, 100), costmap_resolution=0.05,
        costmap_origin=(-2.5, -2.5, 0.0), costmap_data=bytes(costs),
        wheelbase=0.24, rear_axle_to_base=0.12,
        initial_velocity=0.2, initial_steering=0.1,
        command_velocity=0.265, command_steering=0.125,
        model_dt=0.05, time_steps=80, generated_count=768,
        selected_id=4, candidates=candidates)


class PreviewRenderTest(unittest.TestCase):
    """Check actual image coordinates and independent map/candidate transforms."""

    def test_robot_frame_uses_ros_axes_and_measured_yaw(self):
        options = RenderOptions()
        self.assertEqual(base_to_pixel((0, 0), options), (480, 480))
        self.assertEqual(base_to_pixel((1, 0), options), (480, 340))
        self.assertEqual(base_to_pixel((0, 1), options), (340, 480))
        local = world_to_base((3, 5), (3, 4, math.pi / 2))
        self.assertAlmostEqual(local[0], 1.0)
        self.assertAlmostEqual(local[1], 0.0)

    def test_raw_costmap_cells_keep_critical_lethal_and_unknown_colors(self):
        frame, options = sample_frame(), RenderOptions()
        frame.costmap_size = (4, 4)
        frame.costmap_resolution = 1.0
        frame.costmap_origin = (-2, -2, 0)
        data = bytearray(16)
        data[3 * 4 + 3] = 254
        data[3 * 4 + 2] = 253
        data[2 * 4 + 3] = 255
        frame.costmap_data = bytes(data)
        image = _costmap(frame, options)
        for point, color in (((1.5, 1.5), options.lethal_color),
                             ((0.5, 1.5), options.critical_color),
                             ((1.5, 0.5), options.unknown_color),
                             ((-0.5, -0.5), options.free_color)):
            pixel = tuple(round(value) for value in base_to_pixel(point, options))
            self.assertEqual(image.getpixel(pixel), color)

    def test_rotated_map_origin_and_robot_pose_align_same_obstacle(self):
        frame, options = sample_frame(), RenderOptions()
        frame.costmap_size = (2, 2)
        frame.costmap_resolution = 1.0
        frame.costmap_origin = (2, -1, math.pi / 2)
        frame.costmap_data = bytes((254, 0, 0, 0))
        frame.robot_pose = (1, 0.5, math.pi / 2)
        # Cell (0,0) center (0.5,0.5) maps to world (1.5,-0.5).
        local = world_to_base((1.5, -0.5), frame.robot_pose)
        pixel = tuple(round(value) for value in base_to_pixel(local, options))
        self.assertEqual(_costmap(frame, options).getpixel(pixel), options.lethal_color)

    def test_winner_is_highlighted_and_png_round_trips(self):
        frame, options = sample_frame(), RenderOptions()
        image = render_preview(frame, options)
        end = world_to_base(frame.candidates[-1].poses[-1], frame.robot_pose)
        pixel = tuple(round(value) for value in base_to_pixel(end, options))
        self.assertEqual(image.getpixel(pixel), options.selected_color)
        buffer = BytesIO()
        image.save(buffer, format='PNG', compress_level=3)
        decoded = Image.open(BytesIO(buffer.getvalue()))
        self.assertEqual(decoded.size, (960, 960))
        self.assertEqual(decoded.getpixel(pixel), options.selected_color)

    def test_family_color_comes_from_sampler_metadata(self):
        frame, options = sample_frame(), RenderOptions(trajectory_alpha=1.0)
        frame.selected_id = -1
        # Use an arbitrary geometric path and change only its family label.
        for family in range(5):
            poses = ((0, 0, 0), (0.9, 0.6, 0))
            frame.candidates = (Candidate(10, family, 1, 1.0, poses),)
            pixel = tuple(round(value) for value in base_to_pixel((0.9, 0.6), options))
            self.assertEqual(render_preview(frame, options).getpixel(pixel),
                             options.colors[family])

    def test_waiting_or_stale_image_does_not_keep_previous_winner(self):
        frame, options = sample_frame(), RenderOptions()
        end = frame.candidates[-1].poses[-1]
        pixel = tuple(round(value) for value in base_to_pixel(end, options))
        self.assertNotEqual(render_preview(None, options, 'Snapshot scaduto').getpixel(pixel),
                            options.selected_color)

    def test_idle_view_keeps_inflation_and_global_path_without_old_candidates(self):
        frame, options = sample_frame(), RenderOptions(costmap_as_points=False)
        costs = bytearray(frame.costmap_data)
        costs[60 * 100 + 60] = 180
        background = PreviewBackground(
            robot_pose=frame.robot_pose, costmap_size=frame.costmap_size,
            costmap_resolution=frame.costmap_resolution,
            costmap_origin=frame.costmap_origin, costmap_data=bytes(costs))
        path = ((0.5, -0.8), (1.5, -0.8))
        image = render_preview(None, options, 'MPC: in attesa', background, path)
        path_pixel = tuple(round(v) for v in base_to_pixel(path[-1], options))
        self.assertEqual(image.getpixel(path_pixel), options.global_path_color)
        inflation_pixel = tuple(round(v) for v in base_to_pixel((0.525, 0.525), options))
        self.assertEqual(image.getpixel(inflation_pixel),
                         _costmap(background, options).getpixel(inflation_pixel))
        self.assertNotEqual(image.getpixel(inflation_pixel), options.free_color)
        winner_pixel = tuple(round(v) for v in base_to_pixel(
            frame.candidates[-1].poses[-1], options))
        self.assertNotEqual(image.getpixel(winner_pixel), options.selected_color)

    def test_global_path_can_be_displayed_before_the_costmap_arrives(self):
        options = RenderOptions()
        path = ((0.5, 0.8), (1.5, 0.8))
        image = render_preview(None, options, global_path=path)
        pixel = tuple(round(v) for v in base_to_pixel(path[-1], options))
        self.assertEqual(image.getpixel(pixel), options.global_path_color)

    def test_white_tube_has_metric_radius_round_caps_and_clear_interior_before_start(self):
        options = RenderOptions(pixels_per_meter=100.0, tube_radius=0.30)
        path = ((0.8, 0.0), (1.8, 0.0))
        image = render_preview(None, options, global_path=path)
        baseline = render_preview(None, RenderOptions(
            pixels_per_meter=100.0, tube_radius=0.0), global_path=path)
        for point in ((1.2, -0.30), (1.2, 0.30), (2.10, 0.0), (0.50, 0.0)):
            pixel = tuple(round(v) for v in base_to_pixel(point, options))
            self.assertEqual(image.getpixel(pixel), (255, 255, 255))
        for point in ((1.2, 0.15), (1.2, 0.35), (2.15, 0.0)):
            pixel = tuple(round(v) for v in base_to_pixel(point, options))
            self.assertEqual(image.getpixel(pixel), baseline.getpixel(pixel))
        self.assertEqual(image.getpixel((480, 360)), options.global_path_color)

    def test_tube_radius_changes_boundaries_and_clear_path_removes_the_tube(self):
        options = RenderOptions(pixels_per_meter=100.0, tube_radius=0.50)
        path = ((0.8, 0.0), (1.8, 0.0))
        image = render_preview(None, options, global_path=path)
        self.assertEqual(image.getpixel((430, 360)), (255, 255, 255))
        self.assertNotEqual(image.getpixel((450, 360)), (255, 255, 255))
        cleared = render_preview(None, options)
        self.assertNotEqual(cleared.getpixel((430, 360)), (255, 255, 255))

    def test_tube_bends_form_one_boundary_without_internal_overlap_lines(self):
        options = RenderOptions(pixels_per_meter=100.0, tube_radius=0.30)
        path = ((0.8, 0.0), (1.8, 0.0), (1.8, 1.0))
        image = render_preview(None, options, global_path=path)
        baseline = render_preview(None, RenderOptions(
            pixels_per_meter=100.0, tube_radius=0.0), global_path=path)
        # Inside the overlap of the two capsules: preserve the map/grid pixels.
        pixel = tuple(round(v) for v in base_to_pixel((1.6, 0.2), options))
        self.assertEqual(image.getpixel(pixel), baseline.getpixel(pixel))
        self.assertEqual(image.getpixel((430, 270)), (255, 255, 255))

    def test_point_view_keeps_low_soft_inscribed_and_lethal_costs_without_filled_bands(self):
        frame, options = sample_frame(), RenderOptions()
        frame.costmap_size = (4, 2)
        frame.costmap_resolution = 0.20
        frame.costmap_origin = (1.0, -1.0, math.pi / 2)
        frame.robot_pose = (0.5, -0.5, math.pi / 2)
        frame.costmap_data = bytes((1, 60, 180, 252, 253, 254, 255, 0))
        image = _costmap_points(frame, options)
        for index, cost in enumerate(frame.costmap_data):
            x, y = (index % 4 + 0.5) * 0.20, (index // 4 + 0.5) * 0.20
            local = world_to_base((1.0 - y, -1.0 + x), frame.robot_pose)
            u, v = (round(value) for value in base_to_pixel(local, options))
            expected = (options.background_color if cost == 0 else
                        options.unknown_color if cost == 255 else
                        options.lethal_color if cost == 254 else options.costmap_point_color)
            self.assertEqual(image.getpixel((u, v)), expected)
            self.assertEqual(image.getpixel((u + 3, v)), options.background_color)

    def test_invalid_costmap_and_display_parameters_are_rejected(self):
        frame = sample_frame()
        frame.costmap_data = b''
        with self.assertRaises(ValueError):
            render_preview(frame, RenderOptions())
        with self.assertRaises(ValueError):
            RenderOptions(pixels_per_meter=0).validate()
        with self.assertRaises(ValueError):
            RenderOptions(image_width=10000).validate()
        for radius in (-0.1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                RenderOptions(tube_radius=radius).validate()

    def test_source_view_draws_observations_and_omits_the_inflated_master_grid(self):
        frame, options = sample_frame(), RenderOptions()
        sources = PreviewSources(laser=((0.6, 0.8),), semantic=((0.8, -0.8),))
        image = render_preview(frame, options, sources=sources)
        for points, color in ((sources.laser, options.laser_color),
                              (sources.semantic, options.semantic_color)):
            pixel = tuple(round(v) for v in base_to_pixel(points[0], options))
            self.assertEqual(image.getpixel(pixel), color)
        # This cell is occupied in the snapshot, but no sensor endpoint lies here.
        pixel = tuple(round(v) for v in base_to_pixel((1.225, 0.125), options))
        self.assertEqual(image.getpixel(pixel), options.background_color)

    def test_source_jet_costs_come_from_each_points_class_not_from_grid_cells(self):
        frame = sample_frame()
        frame.candidates = ()
        options = RenderOptions(source_cost_colors=True)
        sources = PreviewSources(
            laser=((0.65, 0.90),),
            semantic=((0.73, -0.83, 2), (0.95, -0.72, 3), (1.27, -0.50, 4)))
        expected = (((0.65, 0.90), (0, 0, 0)),
                    ((0.73, -0.83), (228, 255, 27)),  # Yellow class: cost 60.
                    ((0.95, -0.72), (0, 178, 255)),   # Soft class: cost 30.
                    ((1.27, -0.50), (228, 0, 0)))     # Boardwalk class: cost 90.
        for grid_cost in (0, 254):
            frame.costmap_data = bytes([grid_cost]) * 10000
            image = render_preview(frame, options, sources=sources)
            for point, color in expected:
                pixel = tuple(round(v) for v in base_to_pixel(point, options))
                self.assertEqual(image.getpixel(pixel), color)


if __name__ == '__main__':
    unittest.main()
