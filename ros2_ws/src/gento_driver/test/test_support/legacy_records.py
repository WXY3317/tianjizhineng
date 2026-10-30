"""Buffer hardware feedback in memory; analyze and write only after motion stops."""
import json
import os
from pathlib import Path
import tempfile
import time
import uuid


class TrajectoryDiagnostic:
    def __init__(self, plans, starts, goals, execution, config):
        self.directory = str(config.trajectory_log_dir)
        self.name = f'{time.time_ns()}-{uuid.uuid4().hex[:8]}.json'
        self.started_ns = time.monotonic_ns()
        planned_positions = {}
        for arm, (_, count) in plans.items():
            positions = []
            for index in range(count):
                t = index / (count-1) if count > 1 else 0.
                progress = t*t*t*(10+t*(-15+6*t))
                positions.append([(starts[arm][i][3] + progress*(goals[arm][i][3]-starts[arm][i][3]))
                                  * config.sdk_length_to_m for i in range(3)])
            planned_positions[str(arm)] = positions
        self.data = {
            'schema_version': 1, 'created_unix_ns': time.time_ns(),
            'execution': execution, 'sample_hz': 50,
            'units': {'joints': 'deg', 'position': 'm', 'time': 's'},
            'reference_note': 'Nominal time index is not controller playback progress. Nearest joint indices are offline estimates.',
            'config': {k: v for k, v in vars(config).items() if k in (
                'speed', 'acc', 'cartesian_speed', 'cartesian_angular_speed',
                'cartesian_joint_speed', 'cartesian_joint_acc', 'sdk_length_to_m',
                'cartesian_path_tolerance', 'publish_rate')},
            'starts_sdk': starts, 'goals_sdk': goals,
            'planned_tcp_positions_m': planned_positions,
            'planned_joints_deg': {str(a): [list(points[i*7:(i+1)*7]) for i in range(count)]
                                   for a, (points, count) in plans.items()},
            'samples': [], 'dropped_samples': 0,
        }

    @property
    def path(self):
        return str(Path(self.directory).expanduser() / self.name)

    def append(self, sample):
        sample = {**sample, 'elapsed_s': (time.monotonic_ns()-self.started_ns)/1e9,
                  'unix_ns': time.time_ns()}
        if len(self.data['samples']) < 20000:
            self.data['samples'].append(sample)
        else:
            self.data['dropped_samples'] += 1
        self.data['last_sample'] = sample

    def save(self, outcome, reason):
        # Called by a file worker, never by the SDK executor during motion.
        self.data.update(outcome=outcome, reason=reason)
        samples = list(self.data['samples'])
        last = self.data.get('last_sample')
        if last is not None and (not samples or last is not samples[-1]):
            samples.append(last)
        joint_spans = {arm: [max(q[j] for q in path)-min(q[j] for q in path) for j in range(7)]
                       for arm, path in self.data['planned_joints_deg'].items()}
        for sample in samples:
            for arm, actual in sample['arms'].items():
                path = self.data['planned_joints_deg'][arm]
                q = actual['joints_deg']
                nominal = min(len(path)-1, max(0, round(sample['elapsed_s']*50)))
                closest = min(range(len(path)), key=lambda i: sum((x-y)**2 for x, y in zip(q, path[i])))
                indices = []
                for joint in range(7):
                    span = joint_spans[arm][joint]
                    indices.append(min(range(len(path)), key=lambda i: abs(q[joint]-path[i][joint]))
                                   if span > .05 else None)
                actual.update(nominal_index=nominal, nominal_joints_deg=path[nominal],
                              nominal_error_deg=[x-y for x, y in zip(q, path[nominal])],
                              nearest_joint_vector_index=closest,
                              nearest_joint_vector_error_deg=[x-y for x, y in zip(q, path[closest])],
                              per_joint_nearest_indices=indices)
        destination = Path(self.path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix='.trajectory-', dir=destination.parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(self.data, stream, ensure_ascii=False, allow_nan=False)
            os.replace(temporary, destination)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return str(destination)
