"""Validate AMCL samples and journal complete experiments independently of Qt."""

from collections import Counter, deque
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


COVARIANCE_FIELDS = ['cov_{}_{}'.format(i, j) for i in range(6) for j in range(6)]
POSE_FIELDS = ['stamp_s', 'frame_id', 'x', 'y', 'yaw'] + COVARIANCE_FIELDS
COUNT_FIELDS = [
    'input_points', 'candidate_points', 'used_points', 'used_road_points',
    'used_non_road_points', 'particles', 'evaluations',
]
CV_FIELDS = ['stamp_s', 'frame_id', 'reason', 'accepted', 'reused'] + COUNT_FIELDS + [
    'cloud_stamp', 'sync_error',
]
STREAM_FIELDS = {'pose': POSE_FIELDS, 'cv': CV_FIELDS}


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def pose_record(msg):
    """Preserve all 36 covariance entries and validate the planar pose."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid AMCL quaternion')
    x, y, z, w = (value / norm for value in (q.x, q.y, q.z, q.w))
    covariance = list(msg.pose.covariance)
    if len(covariance) != 36 or any(covariance[i] < 0 for i in (0, 7, 35)):
        raise ValueError('Invalid AMCL covariance')
    row = dict(zip(COVARIANCE_FIELDS, covariance))
    row.update(stamp_s=_stamp(msg.header), frame_id=msg.header.frame_id,
               x=p.x, y=p.y, yaw=math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z)))
    if not row['frame_id'] or not all(math.isfinite(row[key]) for key in (
            ['stamp_s', 'x', 'y', 'yaw'] + COVARIANCE_FIELDS)):
        raise ValueError('Non-finite AMCL pose or covariance')
    return row


def cv_records(msg):
    """Read actual fusion attempts, including zero usage after a rejection."""
    records = []
    for status in msg.status:
        if status.name != 'amcl/cv_fusion':
            continue
        values = {entry.key: entry.value for entry in status.values}
        row = {key: int(values[key]) for key in COUNT_FIELDS}
        for key in ('accepted', 'reused'):
            if values[key] not in ('true', 'false'):
                raise ValueError('Invalid diagnostic boolean')
            row[key] = int(values[key] == 'true')
        row.update(
            stamp_s=_stamp(msg.header), frame_id=msg.header.frame_id,
            reason=status.message, cloud_stamp=float(values['cloud_stamp']),
            sync_error=float(values['sync_error']))
        if (any(row[key] < 0 for key in COUNT_FIELDS)
                or row['used_points'] != row['used_road_points'] + row['used_non_road_points']
                or row['used_points'] != (row['candidate_points'] if row['accepted'] else 0)
                or not all(math.isfinite(row[key]) for key in (
                    'stamp_s', 'cloud_stamp', 'sync_error'))):
            raise ValueError('Invalid AMCL diagnostic counts')
        records.append(row)
    return records


class ExperimentSession:
    """Stream every sample to CSV while keeping only a bounded plot window."""

    def __init__(self, parent, settings, clock=time.monotonic):
        self.clock = clock
        self.origin = clock()
        self.started = datetime.now(timezone.utc)
        self.directory = Path(parent) / self.started.strftime('amcl_%Y%m%d_%H%M%S_%fZ')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.settings = dict(settings)
        self.history = {key: deque(maxlen=settings['max_samples']) for key in STREAM_FIELDS}
        self.counts = Counter()
        self.reasons = Counter()
        self.last_stamps = {}
        self.segments = Counter()
        self.files, self.writers = {}, {}
        self.closed = False
        try:
            for key, fields in STREAM_FIELDS.items():
                stream = (self.directory / (key + '.csv')).open(
                    'w', newline='', encoding='utf-8')
                self.files[key] = stream
                writer = csv.DictWriter(stream, fieldnames=['elapsed_s', 'segment'] + fields)
                writer.writeheader()
                stream.flush()
                self.writers[key] = writer
            self.write_metadata()
        except Exception:
            self.close()
            raise

    def append(self, key, record):
        """Keep ROS stamps and a continuous acquisition time across clock resets."""
        if self.closed:
            return False
        row = dict(record)
        if row['stamp_s'] < self.last_stamps.get(key, row['stamp_s']) - 1.0:
            self.segments[key] += 1
        self.last_stamps[key] = row['stamp_s']
        row.update(elapsed_s=self.clock() - self.origin, segment=self.segments[key])
        self.writers[key].writerow(row)
        self.files[key].flush()
        self.counts[key] += 1
        if key == 'cv':
            self.reasons[row['reason']] += 1
            self.counts['accepted'] += row['accepted']
        self.history[key].append(row)
        cutoff = row['elapsed_s'] - self.settings['history_seconds']
        for samples in self.history.values():
            while samples and samples[0]['elapsed_s'] < cutoff:
                samples.popleft()
        return True

    def snapshot(self):
        """Copy only the visible time window; CSVs retain the whole session."""
        return {key: list(samples) for key, samples in self.history.items()}

    def read_plot_data(self, max_points=5000):
        """Read a bounded full-session preview, always including the last sample."""
        result = {}
        for key, fields in STREAM_FIELDS.items():
            if not self.closed:
                self.files[key].flush()
            stride = max(1, math.ceil(self.counts[key] / max_points))
            selected, last = [], None
            with (self.directory / (key + '.csv')).open(encoding='utf-8') as stream:
                for index, row in enumerate(csv.DictReader(stream)):
                    for field in ['elapsed_s', 'segment'] + fields:
                        if field not in ('frame_id', 'reason'):
                            row[field] = float(row[field])
                    last = row
                    if index % stride == 0:
                        selected.append(row)
            if last is not None and (not selected or selected[-1] is not last):
                selected.append(last)
            result[key] = selected
        return result

    def write_metadata(self, rejected=None, complete=False, acquisition_error=''):
        """Record units, counters and interpretation alongside the raw CSVs."""
        metadata = {
            'started_utc': self.started.isoformat(), 'complete': complete,
            'duration_s': self.clock() - self.origin, 'settings': self.settings,
            'samples': dict(self.counts), 'cv_reasons': dict(self.reasons),
            'clock_resets_by_stream': dict(self.segments), 'invalid_samples': rejected or {},
            'acquisition_error': acquisition_error,
            'time_axis': 'Monotonic acquisition seconds; stamp_s preserves ROS time.',
            'points': (
                'used_points counts road and obstacle template centroids actually fused; '
                'rejected updates have zero used_points. evaluations also counts particles.'),
            'covariance': (
                'pose.csv stores the complete 6x6 ROS covariance in x,y,z,roll,pitch,yaw '
                'order. Position variances are m^2, yaw variance is rad^2. '
                'This is estimated uncertainty, not ground-truth error.'),
        }
        path = self.directory / 'metadata.json'
        temporary = path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n',
                             encoding='utf-8')
        temporary.replace(path)

    def close(self):
        """Close CSVs once, including partial sessions after an acquisition error."""
        for stream in self.files.values():
            stream.close()
        self.closed = True
