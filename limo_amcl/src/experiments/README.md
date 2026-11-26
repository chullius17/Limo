# AMCL diagnostics

This directory contains the C++ diagnostics used to inspect mixed laser/CV
localization. The name `experiments` comes from the previous pipeline: these
files are compiled into `limo_amcl_core` and run inside the AMCL node. They do
not start experiments, record bags, save files or generate plots automatically.

## Files and topics

| File | Purpose | Default output topic |
| --- | --- | --- |
| [cv_diagnostics.cpp](cv_diagnostics.cpp) | Report each attempted CV fusion update and why it was accepted or rejected. | `/amcl/cv_diagnostics` |
| [laser_diagnostics.cpp](laser_diagnostics.cpp) | Report laser update results and the work performed by the sensor model. | `/amcl/laser_diagnostics` |
| [cv_diagnostics.hpp](../../include/limo_amcl/experiments/cv_diagnostics.hpp) | Define `CvFusionDiagnostics`, the data collected during a CV fusion attempt. | No publisher of its own. |

Both topics use `diagnostic_msgs/msg/DiagnosticArray`. Their names are private to
the node (`~/cv_diagnostics`, `~/laser_diagnostics`), so renaming or namespacing
`amcl` changes the resolved topic paths. Each message contains one status, with
the laser timestamp and the configured global frame in its header.

## CV diagnostics

The status name is `amcl/cv_fusion`. Accepted fusion has level `OK`; rejected
attempts have level `WARN`. The status message explains the result, for example
`accepted`, `no_synchronized_cloud`, `unavailable_odometry_tf`,
`insufficient_non_road_points`, or `quality_gate: ...`.

| Field | Meaning |
| --- | --- |
| `accepted` | Whether CV actually contributed to the filter weights. |
| `reused` | Whether the selected cloud was already used in an earlier fusion. This does not imply that the current attempt was accepted. |
| `input_points` | Raw points in the selected classified cloud, before filtering and voxelization. |
| `candidate_points` | Road and obstacle XY/polarity voxels produced from the cloud. |
| `used_points` | Candidate voxels if fusion was accepted; zero otherwise. |
| `used_non_road_points` | Obstacle voxels used by an accepted update. |
| `used_road_points` | Road voxels used by an accepted update. |
| `particles` | Particle count at the time of the fusion attempt, before resampling. |
| `evaluations` | Number of voxel/particle comparisons performed during CV scoring. |
| `cloud_stamp` | Selected cloud timestamp, in seconds; `-1` if none was selected. |
| `sync_error` | Absolute cloud/laser timestamp difference, in seconds; `-1` if none was selected. |

Counts reflect the processing stages reached before returning. A quality-gate
rejection can therefore report nonzero `evaluations` with zero `used_points`:
the observation was scored but did not change the filter weights. CV counts are
voxel votes, not the number of raw points contributing to their centroids.

## Laser diagnostics

The status name is `amcl/laser_update`. The status message reports results such
as `accepted`, `disabled`, `no_valid_returns`, `transform_unavailable`,
`model_update_failed`, or `no_selected_returns`. `accepted` and `disabled` have
level `OK`; the other outcomes have level `WARN`. `disabled` means that
`laser_weight_factor` is zero.

| Field | Meaning |
| --- | --- |
| `laser_weight_factor` | Configured laser exponent in the fusion formula. |
| `input_points` | Number of range entries in the incoming scan. |
| `candidate_points` | Candidate rays counted by the selected laser model. |
| `used_points` | Rays actually used by the selected laser model. |
| `particles` | Particles evaluated by the laser model. |
| `evaluations` | Ray/particle evaluations counted by the laser model. |

The beam, likelihood-field and probabilistic likelihood-field models apply their
own range/subsampling/beam-skipping rules. Their candidate and used counts can
therefore differ. If the model is skipped or cannot be reached, its counters are
zero; `input_points` still describes the incoming scan.

## Read the diagnostics

With localization running, use separate terminals:

```bash
ros2 topic echo /amcl/cv_diagnostics
ros2 topic echo /amcl/laser_diagnostics
```

Messages are built and published only while the corresponding lifecycle
publisher is active and has subscribers. The CV publisher is created only when
`cv_enabled` is true. Diagnostics follow filter update attempts, not every
incoming scan or cloud, and past events are not replayed to new subscribers.

`workload_logging_enabled` controls the CV workload log messages; setting it to
false does not disable these diagnostic topics. Reading diagnostics does not
change the fusion decisions, particle weights or resampling behavior.
