# Fleet map and unified replay

A shared 3D occupancy map built from ranging messages, and a single
time-indexed core in which **live is replay with the playhead pinned to now**.

## The message is the contract

The viewer never asks who produced a ray. If `DISTANCE_SENSOR` or
`OBSTACLE_DISTANCE` arrives it is truth, whether it came from a real lidar,
ArduPilot's proximity simulation, Gazebo, or a test harness. There are no
simulator-specific code paths, and a CI test (`no_fixture_symbols`) greps the
map and ingest sources to keep it that way.

## Sources

Four implementations of one `data_source` seam. Each yields
`(session time, vehicle, decoded frame)` and nothing more.

| source | file | notes |
| --- | --- | --- |
| live MAVLink | `src/data_source_mavlink.c` | also the tlog recorder (`--record`) |
| PX4 ULog | `src/data_source_ulog.c` | also serves `--replay` / `--ghost` |
| tlog | `src/data_source_tlog.c` | raw frames plus arrival time |
| ArduPilot `.bin` | `src/data_source_bin.c` | DataFlash; `RFND` and `PRX` |
| skynet run record | `src/skynet_manifest.c` | a manifest, not a parser |

A tlog is the only format that preserves latency, loss and ordering as the
viewer saw them, which is why it doubles as the recorder and the regression
format.

```sh
hawkeye --record flight.tlog          # live, recording what actually arrived
hawkeye --tlog flight.tlog            # replay it
hawkeye --bin  flight.bin             # ArduPilot DataFlash
hawkeye --run  run-1234.json          # a skynet run record: opens what it lists
```

## Time base

Sources disagree about time. ULog is boot-relative, MAVLink `time_usec` is
boot-relative on PX4 and mixed on ArduPilot, tlog carries wall-clock arrival,
DataFlash has its own. A fleet replay in which two vehicles sit seconds apart is
not a replay of anything, so each source declares an offset into one monotonic
session timeline, together with where that offset came from:

| provenance | source |
| --- | --- |
| `GPS UTC` | `SYSTEM_TIME.time_unix_usec` |
| `GPS raw` | `GPS_RAW_INT.time_usec`, leap-second corrected |
| `TIMESYNC` | round trip against the viewer clock |
| `arrival` | wall clock stamped by the tlog writer or the live receive path |
| `boot/manual` | boot-relative, offset set by the operator |
| `boot?` | boot-relative, offset assumed zero |

A source only ever upgrades. The provenance and the fleet spread are both on
screen, because an unlabelled alignment guess is a lie a viewer tells quietly.

## The map

An adaptive octree over an index-based node pool. Root cube 4096 m, leaves
0.25 m by default (`--map-res`), memory ceiling 256 MiB (`--map-cap`).

Per node: `int8` log-odds, `uint32` last-seen, `uint32` observer bitmask,
and saturating `agree` / `disagree` counters.

**Carving is full and hierarchical.** Free space is resolved at a coarse depth
(2 m) in the far field and at full resolution for the last 1.5 m before the
endpoint, so a wall is never eroded by a ray that grazed the corner of the cell
it lives in. A cell that already holds confident occupancy is always resolved at
full depth, whatever the distance.

**A reading at max range is a no return.** It carves free space and marks
nothing occupied. Getting this wrong builds a wall at sensor range around every
flight; the `empty` fixture asserts zero occupied cells.

**Cones, not lasers.** `horizontal_fov` / `vertical_fov` widen the occupied
endpoint with distance by choosing a coarser depth for it. A 25° sonar at 12 m
produces a 2 m cell where a 1° laser produces a 0.25 m one.

**Evidence is weighted.** `covariance` (cm²) and `signal_quality` (%) scale the
log-odds increment, so a weak return moves the map less than a clean one.

**Pruning** collapses eight agreeing sibling leaves back into their parent. It
is what makes memory plateau rather than climb, and the `endurance` fixture
asserts the plateau over 32 simulated minutes.

**Divergence** is a fleet signal. When new evidence contradicts a confident cell
*and* comes from a vehicle other than the ones already on record, `disagree` is
incremented; above a threshold the cell is contested and rendered distinctly.
Restricting it to cross-vehicle contradiction is deliberate: a single vehicle
watching an obstacle get removed should clear it, not paint it pink.

**Coverage falls out of carving for free.** Any cell that is not unknown was
observed, which answers "what did we actually look at" — usually the
operational question.

## One fleet frame

Vehicles may report different `GPS_GLOBAL_ORIGIN`, and some only ever report
local NED against their own. Every origin is pushed through ECEF into one
session ENU frame (`src/geo.c`, `src/fleet_frame.c`); the session origin freezes
before the first ray goes in, because moving it afterwards would silently shift
every cell already in the tree.

## Time-indexed core

`src/timeline.c` holds three things against session time:

* per-vehicle state in a byte-budgeted ring with a binary-searchable index;
* every ray that reached the map, so scrubbing backwards can rebuild the map as
  it stood — otherwise scrubbing back shows obstacles nobody had discovered yet;
* event marks from `STATUSTEXT`, `EVENT`, `COMMAND_ACK`, `MISSION_ITEM_REACHED`,
  `HEARTBEAT` mode changes, and the map's own ray drops.

Ring sizes are byte budgets, not durations. Periodic octree snapshots act as
keyframes so a seek restores the nearest one and replays forward rather than
from zero. The timeline shows how far back the map can honestly be rebuilt;
older than that the ray log has been evicted and the strip says so.

There is one playhead. Live pins it to the head; scrubbing unpins it; the LIVE
button re-pins. That is the whole live/replay unification, and the live view
gains rewind for free.

## Backpressure

Full carving is not free: 72 sectors × 20 Hz × 16 vehicles is a lot of tree
walking. A bounded queue sits between decode and the map with a per-frame
budget. Under overload the **oldest** pending rays are dropped and counted, and
the count is on the HUD and on the timeline as event marks — because a map that
silently falls behind looks exactly like a map of an empty room.

## Rendering

The tree is chunked at 8 m and each chunk keeps its own GPU instance buffer.
Only dirty chunks are re-extracted, chunks are frustum-culled first, and octree
depth doubles as level of detail. Four draw modes: occupancy, coverage,
divergence, contribution.

| key | action |
| --- | --- |
| `J` / `Shift+J` | cycle draw mode |
| `U` | toggle the map |
| `Shift+U` | toggle free-space cells |
| `V` / `Shift+V` | map panel / quality panel |
| `X` / `Shift+X` | mute / solo the selected vehicle's contribution |
| `Home` | re-pin the playhead to live |

## Quality overlays

All from messages that already exist. The GPS accuracy ring uses `h_acc`
(millimetres) — **not** `eph`, which is HDOP × 100 and dimensionless; HDOP and
VDOP are shown as numbers. The uncertainty ellipsoid comes from
`LOCAL_POSITION_NED_COV`, a real covariance rather than a circle. PX4
`ESTIMATOR_STATUS` innovation ratios and ArduPilot `EKF_STATUS_REPORT`
variances are normalized to their own thresholds and land on one gauge. Link
latency and sequence-gap loss, `WIND_COV`, `VIBRATION`, `TERRAIN_REPORT` /
`ALTITUDE.bottom_clearance`, and `COLLISION` — or, when nothing sends it, the
derived inter-vehicle separation matrix and its closest pair.

## Fixtures

See [fleet-map-fixtures.md](fleet-map-fixtures.md).
