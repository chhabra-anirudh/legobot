"""Conservative mesh-envelope measurements, not physical grip calibration.

Clip finger triangles to the cube's X/Z slab in left_eef coordinates. Measure
inner Y surfaces over that slab rather than trusting isolated mesh vertices.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from visualize_urdf import load_stl, parse_urdf, joint_transform, rpy_to_matrix

HERE = Path(__file__).resolve().parent
URDF = HERE / 'chopped_urdf_v2/urdf/chopped_urdf_v2.urdf'


def clip_polygon(polygon, axis, bound, keep_greater):
    """Sutherland-Hodgman clipping against one axis-aligned half-space."""
    result = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        a_in = a[axis] >= bound if keep_greater else a[axis] <= bound
        b_in = b[axis] >= bound if keep_greater else b[axis] <= bound
        if a_in:
            result.append(a)
        if a_in != b_in:
            result.append(a + (b-a) * ((bound-a[axis])/(b[axis]-a[axis])))
    return result


def slab_vertices(triangles, center, size):
    points = []
    for tri in triangles:
        # Cheap rejection, then exact polygon clipping (no mesh sampling).
        if any(tri[:, ax].max() < center[ax]-size/2 or
               tri[:, ax].min() > center[ax]+size/2 for ax in [0, 2]):
            continue
        polygon = list(tri)
        for ax in [0, 2]:
            polygon = clip_polygon(polygon, ax, center[ax]-size/2, True)
            polygon = clip_polygon(polygon, ax, center[ax]+size/2, False)
        points.extend(polygon)
    return np.asarray(points).reshape(-1, 3)


class GripperGeometry:
    def __init__(self):
        links, joints, _ = parse_urdf(str(URDF))
        self.joints = {j['name']: j for j in joints}
        self.eef_t, self.eef_R = joint_transform(self.joints['left_eef_frame'], 0)
        self.meshes = {}
        for name in ['left_left_gripper', 'left_right_gripper']:
            arrays = []
            for visual in links[self.joints[name]['child']]:
                vertices, faces, _ = load_stl(visual['mesh'])
                vertices = vertices * visual['scale']
                vertices = vertices @ rpy_to_matrix(visual['rpy']).T + visual['xyz']
                arrays.append(vertices[faces])
            self.meshes[name] = np.concatenate(arrays)

    def triangles(self, angle):
        driver = self.joints['left_left_gripper']
        if not np.isfinite(angle) or not driver['lower'] <= angle <= driver['upper']:
            raise ValueError('angle outside URDF gripper limits')
        result = []
        for name, triangles in self.meshes.items():
            joint = self.joints[name]
            mimic = joint['mimic']
            value = angle if mimic is None else angle*mimic['multiplier']+mimic['offset']
            t, R = joint_transform(joint, value)
            result.append((triangles @ R.T + t-self.eef_t) @ self.eef_R)
        return result

    def measure(self, angle, center, size):
        center = np.asarray(center, dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)) or not np.isfinite(size) or size <= 0:
            raise ValueError('finite 3D center and positive cube size required')
        left, right = self.triangles(angle)
        a, b = slab_vertices(left, center, size), slab_vertices(right, center, size)
        base = {'angle_rad': float(angle), 'slab_intersects_both_fingers': bool(len(a) and len(b))}
        if not len(a) or not len(b):
            return base
        inner_left, inner_right = a[:, 1].min(), b[:, 1].max()
        base.update(
            gap_m=float(inner_left-inner_right),
            gap_center_y_m=float((inner_left+inner_right)/2),
            left_clearance_m=float(inner_left-(center[1]+size/2)),
            right_clearance_m=float((center[1]-size/2)-inner_right),
            outer_width_m=float(a[:, 1].max()-b[:, 1].min()),
            # Downward tool orientation maps local +Z toward the table.
            fingertip_table_clearance_m=float(size/2+center[2]-max(left[:, :, 2].max(), right[:, :, 2].max())),
        )
        return base


    def contact_candidate(self, center, size, open_angle):
        """Find an envelope-width match within a checked monotonic interval.

        This estimates geometry only. It does not establish two opposing contact
        patches, compliance, friction, or a commanded hardware grip force.
        """
        samples = [self.measure(q, center, size) for q in np.linspace(0, open_angle, 16)]
        if any(not m['slab_intersects_both_fingers'] for m in samples):
            raise ValueError('finger slab missing in search interval')
        gaps = [m['gap_m'] for m in samples]
        if any(b < a for a, b in zip(gaps, gaps[1:])) or not gaps[0] <= size <= gaps[-1]:
            raise ValueError('no bracketed monotonic envelope-width solution')
        lo, hi = 0., open_angle
        for _ in range(24):
            mid = (lo+hi)/2
            measurement = self.measure(mid, center, size)
            if measurement['gap_m'] < size:
                lo = mid
            else:
                hi = mid
        measurement = self.measure((lo+hi)/2, center, size)
        return {'angle_rad': measurement['angle_rad'],
                'cube_center_tool_m': [float(center[0]), measurement['gap_center_y_m'], float(center[2])],
                'gap_m': measurement['gap_m'],
                'status': 'geometric candidate only; verify contacts and hardware before use'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE/'assembly_config.json')
    parser.add_argument('--output', type=Path, help='optional JSON report')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    size = config['block_size_m']
    if len(size) != 3 or not np.allclose(size, [size[0]]*3):
        raise ValueError('this calibration probe requires cubic blocks')
    geometry = GripperGeometry()
    angles = sorted(set([0, .05, .1, .15, .185, .2, .25, .35, .6, 1.,
                         config['gripper_open_rad'], config['gripper_grasp_rad']]))
    report = {
        'method': 'visual-mesh conservative envelope clipped to cube X/Z slab',
        'frame': 'left_eef', 'cube_size_m': size[0],
        'cube_center_tool_m': config['tool_grasp_point_m'],
        'limitations': 'Not contact force, collision certification, pad compliance, or hardware calibration. Negative clearance indicates envelope interference; positive clearance does not prove a stable grasp.',
        'measurements': [geometry.measure(q, config['tool_grasp_point_m'], size[0]) for q in angles],
    }
    report['contact_candidate'] = geometry.contact_candidate(config['tool_grasp_point_m'], size[0], config['gripper_open_rad'])
    result = json.dumps(report, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result)
    print(result)


if __name__ == '__main__':
    main()
