"""
hole_visualiser_node.py
━━━━━━━━━━━━━━━━━━━━━━━
Subscribes to /tof_costmap (std_msgs/String, JSON, LATCHED) and
continuously republishes MarkerArray topics at publish_hz.

Fixes applied vs. previous version
──────────────────────────────────
  • frame_id default already matches tof_costmap ('tof_sensor_link')
  • Spline integration now MASKS phantom cells (cells that are inside the
    bounding rectangle but not in the original cluster footprint) so
    L/T/ring-shaped holes don't get inflated volume estimates
  • Cubic spline only used when the cluster is large enough in BOTH axes
    (≥4×4 grid); falls back to linear for smaller clusters
  • DELETEALL + ADD markers now sent in a SINGLE MarkerArray (no flicker)
  • Marker republish rate reduced from 1 Hz → 0.5 Hz (data only changes
    once per scan; latched QoS handles late subscribers)
  • Bounding-box rendered as LINE_LIST edges, not solid transparent cube
    (eliminates z-fighting with hole cubes inside)
  • cz convention now consistent with tof_costmap: mid-depth below floor=0
  • Type hint for _last_data fixed
"""

import json

import numpy as np
from scipy.interpolate import RectBivariateSpline, NearestNDInterpolator

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# Removed explicit latched QoS; use default QoS or simple depth args.


# ── colour helpers ────────────────────────────────────────────────────────────
def _lerp3(t, c0, c1):
    return tuple(c0[i] + (c1[i] - c0[i]) * t for i in range(3))


def hole_color(depth_m, depth_range_m):
    dr = max(depth_range_m, 1e-6)
    t  = min(1.0, depth_m / dr)
    if t < 0.5:
        r, g, b = _lerp3(t * 2.0, (1.00, 0.85, 0.05), (1.00, 0.28, 0.05))
    else:
        r, g, b = _lerp3((t - 0.5) * 2.0, (1.00, 0.28, 0.05), (0.50, 0.02, 0.02))
    return ColorRGBA(r=r, g=g, b=b, a=0.78 + 0.20 * t)


def cluster_color(idx):
    palette = [
        (0.9, 0.6, 0.1), (0.2, 0.8, 0.6), (0.7, 0.3, 0.9),
        (0.2, 0.6, 1.0), (1.0, 0.4, 0.4), (0.5, 0.9, 0.3),
        (0.9, 0.2, 0.7), (0.3, 0.4, 0.9),
    ]
    r, g, b = palette[idx % len(palette)]
    return ColorRGBA(r=r, g=g, b=b, a=1.0)


# ── Smooth volume & centroid via spline integration ───────────────────────────
def smooth_volume_and_centroid(points, scan_step_m, upsample=4, smoothing=0.0):
    """
    Fit a bivariate spline to the cluster's (x, y, depth) grid and integrate
    over the dense upsampled surface — but MASK OUT phantom cells so that
    L/T/ring-shaped clusters don't get inflated volumes.

    Returns
    ───────
    volume_m3, cx, cy, cz   (cz is negative = below floor=0, mid-depth)
    """
    # ── Raw fallback (per-cell sum) ───────────────────────────────────────────
    cell_area = scan_step_m ** 2
    raw_vol = raw_cx = raw_cy = raw_cz = 0.0
    for (x, y, d) in points:
        v        = cell_area * d
        raw_vol += v
        raw_cx  += x * v
        raw_cy  += y * v
        raw_cz  += -(d / 2.0) * v
    if raw_vol > 0:
        raw_cx /= raw_vol
        raw_cy /= raw_vol
        raw_cz /= raw_vol

    if len(points) < 4:
        return raw_vol, raw_cx, raw_cy, raw_cz

    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    ds = np.array([p[2] for p in points])

    gxs    = np.round(xs / scan_step_m).astype(int)
    gys    = np.round(ys / scan_step_m).astype(int)
    x_idxs = np.unique(gxs)
    y_idxs = np.unique(gys)

    if len(x_idxs) < 2 or len(y_idxs) < 2:
        return raw_vol, raw_cx, raw_cy, raw_cz

    x_unique = x_idxs * scan_step_m
    y_unique = y_idxs * scan_step_m

    # Populate coarse depth grid AND a "is this cell actually part of the
    # cluster?" mask.  The mask is what saves us from the phantom-cell bug.
    depth_grid    = np.full((len(y_idxs), len(x_idxs)), np.nan)
    cluster_mask  = np.zeros((len(y_idxs), len(x_idxs)), dtype=bool)
    xi_map = {v: i for i, v in enumerate(x_idxs)}
    yi_map = {v: i for i, v in enumerate(y_idxs)}
    for xi, yi, d in zip(gxs, gys, ds):
        depth_grid[yi_map[yi], xi_map[xi]] = d
        cluster_mask[yi_map[yi], xi_map[xi]] = True

    # NaN cells (phantom) get filled by NN so the spline has a smooth surface
    # to fit, but we'll later mask them OUT of the integration.
    nan_mask = np.isnan(depth_grid)
    if nan_mask.any():
        known_yx  = np.column_stack(np.where(~nan_mask))
        known_val = depth_grid[~nan_mask]
        fill_yx   = np.column_stack(np.where(nan_mask))
        nn = NearestNDInterpolator(known_yx, known_val)
        depth_grid[nan_mask] = nn(fill_yx)

    # Choose spline order based on available data in BOTH axes
    ny, nx = depth_grid.shape
    if ny >= 4 and nx >= 4:
        kx = ky = 3
    elif ny >= 2 and nx >= 2:
        kx = ky = 1
    else:
        return raw_vol, raw_cx, raw_cy, raw_cz

    try:
        spline = RectBivariateSpline(
            y_unique, x_unique, depth_grid, kx=kx, ky=ky, s=smoothing)
    except Exception:
        return raw_vol, raw_cx, raw_cy, raw_cz

    # Dense grid evaluation
    x_dense = np.linspace(x_unique[0], x_unique[-1],
                          (len(x_unique) - 1) * upsample + 1)
    y_dense = np.linspace(y_unique[0], y_unique[-1],
                          (len(y_unique) - 1) * upsample + 1)
    Z = np.clip(spline(y_dense, x_dense), 0.0, None)

    # Build a dense-grid validity mask from the coarse cluster_mask.
    # A dense-grid point is valid iff its nearest coarse cell is in the cluster.
    # We use np.repeat to upsample the mask.  Each coarse cell becomes
    # `upsample` dense cells in each axis (plus one trailing endpoint).
    # Simple approach: nearest-neighbour upsample the mask.
    dense_mask = np.zeros_like(Z, dtype=bool)
    for iy in range(ny):
        for ix in range(nx):
            if not cluster_mask[iy, ix]:
                continue
            y_start = iy * upsample
            y_end   = (iy + 1) * upsample + (1 if iy == ny - 1 else 0)
            x_start = ix * upsample
            x_end   = (ix + 1) * upsample + (1 if ix == nx - 1 else 0)
            dense_mask[y_start:y_end, x_start:x_end] = True

    dx = (x_unique[-1] - x_unique[0]) / ((len(x_unique) - 1) * upsample)
    dy = (y_unique[-1] - y_unique[0]) / ((len(y_unique) - 1) * upsample)
    cell_area_dense = dx * dy

    XX, YY = np.meshgrid(x_dense, y_dense)

    # Mask Z to zero outside cluster footprint, then integrate
    Z_masked = np.where(dense_mask, Z, 0.0)
    vols     = Z_masked * cell_area_dense

    total_vol = float(vols.sum())
    if total_vol == 0.0:
        return raw_vol, raw_cx, raw_cy, raw_cz

    cx = float((XX * vols).sum()) / total_vol
    cy = float((YY * vols).sum()) / total_vol
    cz = float((-(Z_masked / 2.0) * vols).sum()) / total_vol

    return total_vol, cx, cy, cz


# ── Node ─────────────────────────────────────────────────────────────────────
class HoleVisualiserNode(Node):
    def __init__(self):
        super().__init__('hole_visualiser_node')

        self.declare_parameter('result_topic',       '/tof_costmap')
        self.declare_parameter('frame_id',           'tof_sensor_link')
        self.declare_parameter('hole_depth_range_m', 0.10)
        self.declare_parameter('summary_z_m',        0.15)
        self.declare_parameter('text_scale_m',       0.05)
        self.declare_parameter('publish_hz',         0.5)
        self.declare_parameter('mesh_upsample',      4)
        self.declare_parameter('mesh_smoothing',     0.0)

        self.result_topic       = self.get_parameter('result_topic').value
        self.frame_id           = self.get_parameter('frame_id').value
        self.hole_depth_range_m = float(self.get_parameter('hole_depth_range_m').value)
        self.summary_z_m        = float(self.get_parameter('summary_z_m').value)
        self.text_scale_m       = float(self.get_parameter('text_scale_m').value)
        publish_hz              = float(self.get_parameter('publish_hz').value)
        self.mesh_upsample      = int(self.get_parameter('mesh_upsample').value)
        self.mesh_smoothing     = float(self.get_parameter('mesh_smoothing').value)

        self._last_data    = None      # dict | None
        self._smooth_stats = {}        # {cluster_id: (vol_m3, cx, cy, cz)}

        self.sub = self.create_subscription(
            String, self.result_topic, self._result_cb, 10)
        self.result_pub = self.create_publisher(String, '/tof_result', 10)

        self.hole_pub    = self.create_publisher(MarkerArray, 'tof_result/hole_markers',    10)
        self.text_pub    = self.create_publisher(MarkerArray, 'tof_result/text_markers',    10)
        self.summary_pub = self.create_publisher(MarkerArray, 'tof_result/summary_markers', 10)

        period = 1.0 / max(0.1, publish_hz)
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            f'HoleVisualiserNode started\n'
            f'  Subscribing to  : {self.result_topic}\n'
            f'  Publish rate    : {publish_hz:.2f} Hz\n'
            f'  Frame           : {self.frame_id}\n'
            f'  Spline upsample : {self.mesh_upsample}× (phantom cells masked)\n'
            f'  Text scale      : {self.text_scale_m*1000:.0f} mm'
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _result_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse result JSON: {e}')
            return

        self._last_data    = data
        self._smooth_stats = {}

        clusters    = data.get('clusters', [])
        scan_step_m = float(data.get('scan_step_m', 0.01))
        baseline_m  = float(data.get('baseline_m', 0.0))

        for cluster in clusters:
            cid = cluster.get('id')
            pts = [(float(p['x']), float(p['y']), float(p['depth_m']))
                   for p in cluster.get('points', [])]

            vol, cx, cy, cz = smooth_volume_and_centroid(
                pts, scan_step_m,
                upsample=self.mesh_upsample,
                smoothing=self.mesh_smoothing,
            )
            self._smooth_stats[cid] = (vol, cx, cy, cz)

            self.get_logger().info(
                f'  Cluster #{cid}: '
                f'raw={cluster.get("volume_cm3", 0):.2f} cm³  '
                f'smooth={vol*1e6:.2f} cm³  '
                f'centroid=({cx:.4f}, {cy:.4f}, {cz:.4f}) m'
            )

        self.get_logger().info(
            f'New scan result — {len(clusters)} cluster(s), '
            f'baseline={baseline_m:.4f} m'
        )
        self._publish_result(data)

    # ──────────────────────────────────────────────────────────────────────────
    def _timer_cb(self):
        if self._last_data is None:
            return
        stamp       = self.get_clock().now().to_msg()
        clusters    = self._last_data.get('clusters', [])
        scan_step_m = float(self._last_data.get('scan_step_m', 0.01))

        self._publish_hole_markers(clusters, scan_step_m, stamp)
        self._publish_text_markers(clusters, scan_step_m, stamp)
        self._publish_summary(self._last_data, stamp)

    def _publish_result(self, data):
        clusters = []
        for cluster in data.get('clusters', []):
            cid    = cluster.get('id')
            smooth = self._smooth_stats.get(cid)
            if smooth is None:
                smooth = (
                    float(cluster.get('volume_cm3', 0.0)) / 1e6,
                    float(cluster.get('centroid', {}).get('x', 0.0)),
                    float(cluster.get('centroid', {}).get('y', 0.0)),
                    float(cluster.get('centroid', {}).get('z', 0.0)),
                )
            vol_m3, cx, cy, cz = smooth
            clusters.append({
                **cluster,
                'volume_cm3': round(vol_m3 * 1e6, 4),
                'centroid': {'x': round(cx, 4), 'y': round(cy, 4), 'z': round(cz, 4)},
                'smooth_volume_cm3': round(vol_m3 * 1e6, 4),
                'smooth_centroid': {'x': round(cx, 4), 'y': round(cy, 4), 'z': round(cz, 4)},
            })

        payload = {
            'baseline_m':  float(data.get('baseline_m', 0.0)),
            'scan_step_m': float(data.get('scan_step_m', 0.01)),
            'total_holes': len(clusters),
            'clusters':    clusters,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_hole_markers(self, clusters, scan_step_m, stamp):
        """One CUBE per hole cell + line-outline bounding box per cluster.
        DELETEALL and ADDs sent in a SINGLE MarkerArray to avoid flicker."""
        ma = MarkerArray()

        del_m = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'hole_vis'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        if not clusters:
            self.hole_pub.publish(ma)
            return

        next_id = 1

        for ci, cluster in enumerate(clusters):
            points = cluster.get('points', [])

            # ── One CUBE per hole cell ────────────────────────────────────────
            for pt in points:
                x_m     = float(pt['x'])
                y_m     = float(pt['y'])
                depth_m = float(pt['depth_m'])
                height  = max(depth_m, scan_step_m)

                m                    = Marker()
                m.header.stamp       = stamp
                m.header.frame_id    = self.frame_id
                m.ns                 = 'hole_vis'
                m.id                 = next_id
                m.type               = Marker.CUBE
                m.action             = Marker.ADD
                m.pose.orientation.w = 1.0
                m.lifetime.sec       = 0
                m.pose.position.x    = x_m
                m.pose.position.y    = y_m
                m.pose.position.z    = -(height / 2.0)
                m.scale.x            = scan_step_m
                m.scale.y            = scan_step_m
                m.scale.z            = height
                m.color              = hole_color(depth_m, self.hole_depth_range_m)
                ma.markers.append(m)
                next_id += 1

            # ── Bounding-box OUTLINE (line edges, no z-fighting) ──────────────
            if points:
                xs = [float(p['x'])       for p in points]
                ys = [float(p['y'])       for p in points]
                ds = [float(p['depth_m']) for p in points]

                x_min = min(xs) - scan_step_m / 2.0
                x_max = max(xs) + scan_step_m / 2.0
                y_min = min(ys) - scan_step_m / 2.0
                y_max = max(ys) + scan_step_m / 2.0
                z_min = -max(ds) * 1.05
                z_max = scan_step_m * 0.5

                corners = [
                    (x_min, y_min, z_min), (x_max, y_min, z_min),
                    (x_max, y_max, z_min), (x_min, y_max, z_min),
                    (x_min, y_min, z_max), (x_max, y_min, z_max),
                    (x_max, y_max, z_max), (x_min, y_max, z_max),
                ]
                # 12 edges of a cuboid
                edges = [
                    (0,1),(1,2),(2,3),(3,0),    # bottom
                    (4,5),(5,6),(6,7),(7,4),    # top
                    (0,4),(1,5),(2,6),(3,7),    # vertical
                ]

                line = Marker()
                line.header.stamp    = stamp
                line.header.frame_id = self.frame_id
                line.ns              = 'hole_vis'
                line.id              = next_id
                line.type            = Marker.LINE_LIST
                line.action          = Marker.ADD
                line.pose.orientation.w = 1.0
                line.lifetime.sec    = 0
                line.scale.x         = 0.002          # line thickness 2 mm
                line.color           = cluster_color(ci)
                line.color.a         = 0.8

                for a, b in edges:
                    pa = Point(); pa.x, pa.y, pa.z = corners[a]
                    pb = Point(); pb.x, pb.y, pb.z = corners[b]
                    line.points.append(pa)
                    line.points.append(pb)
                ma.markers.append(line)
                next_id += 1

        self.hole_pub.publish(ma)

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_text_markers(self, clusters, scan_step_m, stamp):
        ma = MarkerArray()

        del_m = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'hole_text'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        if not clusters:
            self.text_pub.publish(ma)
            return

        next_id = 1

        for ci, cluster in enumerate(clusters):
            cid   = cluster.get('id', ci + 1)
            cells = cluster.get('cells', 0)
            bc    = cluster_color(ci)

            if cid in self._smooth_stats:
                vol_m3, cx, cy, cz = self._smooth_stats[cid]
                vol_cm3 = vol_m3 * 1e6
            else:
                vol_cm3  = cluster.get('volume_cm3', 0.0)
                centroid = cluster.get('centroid', {})
                cx = float(centroid.get('x', 0))
                cy = float(centroid.get('y', 0))
                cz = float(centroid.get('z', 0))

            t                    = Marker()
            t.header.stamp       = stamp
            t.header.frame_id    = self.frame_id
            t.ns                 = 'hole_text'
            t.id                 = next_id
            t.type               = Marker.TEXT_VIEW_FACING
            t.action             = Marker.ADD
            t.pose.orientation.w = 1.0
            t.lifetime.sec       = 0
            t.pose.position.x    = cx
            t.pose.position.y    = cy
            # Anchor labels above the floor at fixed offset for consistent placement
            t.pose.position.z    = scan_step_m + self.text_scale_m * 2.0
            t.scale.z            = self.text_scale_m
            t.color              = ColorRGBA(r=bc.r, g=bc.g, b=bc.b, a=1.0)
            t.text = (
                f'Cluster #{cid}\n'
                f'Vol: {vol_cm3:.2f} cm\u00b3\n'
                f'Cells: {cells}\n'
                f'({cx:.3f}, {cy:.3f}, {cz:.3f}) m'
            )
            ma.markers.append(t)
            next_id += 1

        self.text_pub.publish(ma)

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_summary(self, data, stamp):
        ma = MarkerArray()

        del_m = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'hole_summary'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        clusters    = data.get('clusters', [])
        baseline_m  = float(data.get('baseline_m', 0.0))

        total_vol   = sum(
            self._smooth_stats[c['id']][0] * 1e6
            if c['id'] in self._smooth_stats
            else c.get('volume_cm3', 0.0)
            for c in clusters
        )
        total_cells = sum(c.get('cells', 0) for c in clusters)

        lines = [
            '\u2500\u2500\u2500 Scan Complete \u2500\u2500\u2500',
            f'Holes     : {len(clusters)}',
            f'Total vol : {total_vol:.2f} cm\u00b3',
            f'Cells     : {total_cells}',
            f'Baseline  : {baseline_m:.4f} m',
        ]
        if clusters:
            lines.append('\u2500\u2500\u2500 Per cluster \u2500\u2500\u2500')
            for c in clusters:
                cid = c['id']
                if cid in self._smooth_stats:
                    vol_m3, cx, cy, cz = self._smooth_stats[cid]
                    vol_cm3 = vol_m3 * 1e6
                else:
                    vol_cm3 = c.get('volume_cm3', 0.0)
                    ct      = c.get('centroid', {})
                    cx, cy, cz = ct.get('x', 0), ct.get('y', 0), ct.get('z', 0)
                lines.append(
                    f'  #{cid}  {vol_cm3:.2f} cm\u00b3'
                    f'  ({cx:.3f}, {cy:.3f}, {cz:.3f}) m'
                )

        s                    = Marker()
        s.header.stamp       = stamp
        s.header.frame_id    = self.frame_id
        s.ns                 = 'hole_summary'
        s.id                 = 1
        s.type               = Marker.TEXT_VIEW_FACING
        s.action             = Marker.ADD
        s.pose.orientation.w = 1.0
        s.lifetime.sec       = 0
        s.pose.position.x    = 0.0
        s.pose.position.y    = 0.0
        s.pose.position.z    = self.summary_z_m
        s.scale.z            = self.text_scale_m
        s.color              = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        s.text               = '\n'.join(lines)
        ma.markers.append(s)

        self.summary_pub.publish(ma)


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = HoleVisualiserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HoleVisualiserNode interrupted, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
