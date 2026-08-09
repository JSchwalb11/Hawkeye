# Fleet map fixtures

Synthetic ray injectors that ray-cast against **known** geometry and emit
genuine `DISTANCE_SENSOR` / `OBSTACLE_DISTANCE` MAVLink messages. Because the
geometry is known, map error is a number rather than an impression.

## Rules

* They live under `test/` and are **never** linked into the viewer. There is no
  `#ifdef TEST` in the map or ingest code, and `no_fixture_symbols` enforces it.
* They emit **over the wire** — UDP, the same ports a vehicle uses — so the
  tests exercise the real ingest path instead of reaching around it into the map
  API. `fixture_wire_smoke` runs that path end to end.
* They are **deterministic**: seeded RNG, fixed timestep, fixed message
  ordering, timestamps derived only from the simulated clock. A run is
  byte-reproducible, so CI can assert exact figures.
* They **record themselves to a tlog**, so every other fixture replays in CI
  with no live process and no timing flake.
* They **publish their ground truth** beside the tlog — the geometry, the
  vehicle origins, the map configuration, the thresholds, and every ray actually
  cast — so the checker scores without re-deriving anything.

```sh
make fixtures     # record and score every fixture
make renders      # write orthographic views to docs/assets/fleet-map/

build/test/ray_injector --fixture corridor --tlog c.tlog --truth c.truth
build/test/map_checker  --truth c.truth --tlog c.tlog --render c.png
```

## The set

Each isolates one failure mode. A single "room with a box" would prove almost
nothing.

| fixture | geometry | catches |
| --- | --- | --- |
| `empty` | nothing in range; every ray returns `max_distance` | the wall-at-sensor-range bug: free space and **zero** occupied cells |
| `ground` | one horizontal plane, downward sensor | NED sign errors, and AGL against `ALTITUDE.bottom_clearance` |
| `wall` | vertical plane at a known range and bearing | range scaling and body→NED rotation |
| `corridor` | two parallel walls, 360° `OBSTACLE_DISTANCE` | sector mapping: `increment_f`, `angle_offset`, `frame` |
| `orientations` | one plane, all 40 `MAV_SENSOR_ORIENTATION` values plus a custom quaternion | `R_body←sensor` — every mount must land on the same plane |
| `moving` | translating and yawing while ranging a fixed wall | pose/timestamp association |
| `two-origins` | two vehicles, deliberately different `GPS_GLOBAL_ORIGIN`, same wall | the ECEF→ENU fleet merge |
| `disagreement` | two vehicles, one given a known position offset | divergence flagging, and that it appears only where the offset puts it |
| `clocks` | two vehicles, one with its ranging stamps 0.8 s off its pose stamps | that ranging timestamps are actually used, not just the latest attitude |
| `vanishing` | obstacle present for 30 s, then removed | that carving genuinely clears; a hits-only map can never pass |
| `cone` | wide-FOV sonar vs narrow laser, same target | endpoint widening with distance |
| `weak` | degrading `signal_quality`, rising `covariance` | evidence weighting |
| `firehose` | 72 sectors × 20 Hz × N vehicles | throughput, the ray-drop path, and that drops are **reported** |
| `endurance` | 32 simulated minutes over a bounded volume | pruning: memory must plateau, not climb |

### Why `orientations` transcribes the enum twice

The injector cannot ask the viewer where a mount points — a wrong table entry
would cancel itself out and the test would pass. So `test/orientation_basis.c`
transcribes `MAV_SENSOR_ORIENTATION` a second time, from the names, and builds
rotation matrices by explicit 3×3 multiplication rather than through
quaternions. Two separate transcriptions and two separate rotation
implementations: a mistake in either shows up as a mismatch.

### Why `clocks` skews backwards

A forward-dated ranging stamp resolves to the newest pose available and quietly
corrects itself, so it would prove nothing. Back-dating by 0.8 s makes the
viewer pair a fresh range with a genuinely stale attitude. The fixture asserts
both halves: vehicle 0 (aligned) must be clean **and** vehicle 1 (skewed) must
be visibly worse — if the code paired every range with the latest attitude,
both would smear alike and the pair would fail.

## Scored, not eyeballed

Every fixture ships thresholds that CI asserts:

* **surface RMS** — distance from occupied cells to the true surface, measured
  from the cell's nearest face so a coarse cone-widened cell is not punished for
  being big;
* **false-occupied rate** — occupied cells in known-free space, the number that
  exposes bad carving and pose lag;
* **false-free rate** — surfaces the map lost, counting a surface as represented
  when occupancy sits within one cell of it;
* **coverage completeness** — the observed fraction of the volume actually
  swept, scored against the published rays;
* **peak and steady-state memory**, and live node count before and after
  pruning;
* **sustained rays/s** and **rays dropped**, plus a check that the drops reached
  the timeline rather than being absorbed.

A map change that improves the picture but regresses false-occupied is then
visible as a number, which is the point.

## Renders

`map_checker --render` draws orthographic views straight from the octree — no
GPU, no screenshotting, reproducible in CI. Panels cover occupancy (top and
side), coverage, divergence and the focused vehicle's contribution, beside the
figures the run was scored on. Samples live in
[`docs/assets/fleet-map/`](../assets/fleet-map/).
