# Fleet map — deferred work

Items left on the table when the fleet-map work merged (#2). None of them blocks
that change; each is written up here with the measurement that motivates it, so
the next person does not have to rediscover the number before deciding whether
it is worth the effort.

Finished items move to the "Done" section at the end, with what they turned out
to be and what now guards them.

Issues are disabled on this repository, which is why these live in the tree
rather than in a tracker. If issues are turned on, each heading below is a
ready-made issue body.

Ordered by what I would do first, not by size.

---

## 1. Complete the cone sensor model, or keep the axis carve deliberately

Built, measured and **not** shipped in #2. Full reasoning is in
[`fleet-map.md`](fleet-map.md) under "Why the carve is a ray and not a cone".

The motivation: the carved volume is lace rather than a solid region. Measured
on `statue-fleet`, **26.3% of free cells' face neighbours are `UNKNOWN`** and
only **30.1%** of free cells are fully enclosed, because a 1.7° fan at 35 m
leaves roughly a metre between adjacent rays. The carve is a pencil ray along
the beam axis while the *hit* is already widened to the beam cone — an
inconsistency in the model, and the reason the renderer's free-space veil cannot
be thinned by neighbour culling.

**The trap, for anyone reimplementing it.** Realising the cone by inflating each
node's box by the cone radius before clipping the axis against it is cheap and
correct laterally — but the inflation is a cube, so it extends the swept region
*along* the beam too. A carve told to stop a hit-cell short of the surface still
reached a further cone-radius past that, into it. On `cone` — a flat wall,
viewed head-on — that alone lost 36% of the surface while the cells that
survived sat at 0.0000 m RMS. Subtracting the cone radius from the axial stop
takes the same fixture to 0.00000. Start there.

Three other explanations were measured first and each moved the number by under
0.05:

| hypothesis | result |
| --- | --- |
| per-child cone radius taken at the parent's far end | 0.479 → 0.532 (worse) |
| injector reporting axis range, not minimum-over-beam | 0.580 → 0.479 (fixed; not the cause) |
| under-sampling that minimum (13 / 33 / 65 rays per sector) | 0.532 / 0.519 / 0.530 (flat) |

Even correct, it is half a model change: the carve sweeps a whole cone free
while occupancy is still marked at a single leaf on the axis. Cone-wide free
against point-wide occupied erodes any surface whose beam footprint is much
larger than a leaf. 17 of 20 fixtures pass; the three that do not are the ones
that matter.

| | pencil carve | cone carve |
| --- | --- | --- |
| `statue-solo` shape recall | 0.9996 | 0.8337 |
| `statue-solo` false-free | 0.0005 | 0.0586 |
| `statue-fleet` shape recall | 1.0000 | 0.8654 |
| `statue-fleet` merged surface | 0.9994 | 0.9438 |
| `endurance` live nodes | passing | 87,641 against a 60,000 ceiling |

Completing it means spreading the hit across the cone footprint at reduced
weight. A wide beam knows *"something is at range R somewhere in the cone"* — a
disjunction an occupancy grid cannot hold directly, so the honest representation
is partial evidence over the footprint rather than full evidence at one cell.
That wants its own calibration, and it would legitimately make the map blobbier
for wide beams: the decimetre statue thresholds would need revisiting against
what a 1.7° beam at 35 m can actually resolve, rather than against the idealised
pencil-ray sensor they were calibrated on.

Carving the axis claims *less* than the sensor knows, which is the safe
direction to be wrong in. **Deciding to keep it that way is a legitimate way to
close this.**

---

## 2. The free-space veil is still hazier than it needs to be

After the compositing fix, **72.2%** of surface pixels still read as surface
against a free-hidden reference frame — up from 34.2%, but not 100%. Two
residuals, both measured:

**Order dependence.** Which of the nearer free cells blend before the nearest
one wins the depth test still depends on draw order, so the veil's exact shade
does too. It is a shade and not a surface — no ordering can hide geometry now
that the surfaces own the depth buffer first — but it means the veil can shift
subtly as chunks re-extract. A `GL_MAX` veil is exactly one layer and
order-independent; measured at **72.9%**, it buys nothing legibility-wise,
flattens the veil's shading, and needs `RL_BLEND_CUSTOM_SEPARATE`. Recorded so
nobody re-runs it expecting more.

**Instance count.** 273,375 instances and ~207 ms per frame under llvmpipe,
*identical across all three compositing variants* — the cost is instance
submission, not blending, so no compositing change will touch it. Culling free
cells enclosed on all six faces removed only 28%, because the carved volume is
lace (see item 1).

Thinning it further needs a different idea than face-neighbour culling. Two
worth measuring: greedy meshing of contiguous free runs into larger boxes
(exact, no information lost), or extracting free at a coarser LOD than surfaces
— which needs an honest majority-free aggregation rule, since the existing
min/max aggregator would report a mostly-unknown cell as free and quietly
overstate coverage in the one view that exists to show it.

---

## 3. A TSDF is the right representation for centimetre work

Not a defect — a structural observation, recorded because it keeps coming up.

At 7.8 mm a leaf is routinely only *partially* occupied, and a binary occupancy
grid has no way to say so. That is the root of the erosion bug fixed in #2: free
evidence from rays that merely skimmed a surface is systematically wrong, and
the fix (a range return clears stale free evidence) treats the symptom rather
than the representation.

A truncated signed distance field holds the thing the grid cannot: a ray passing
near a surface writes a positive signed distance rather than "empty", so a zero
crossing survives a grazing pass and still moves when the surface genuinely goes
away — which is the property `vanishing` exists to protect. It also resolves a
surface to roughly voxel/10, so a 5 cm TSDF matches a 5 mm binary grid at
**1/125** the cell count.

The cost is that almost everything downstream assumes log-odds cells: the
renderer's buckets, the observer bitmask, the contested-cell logic, the
keyframe snapshots and the scored fixtures' thresholds. This is a rewrite of the
map core, not a patch, and it should not start until something actually needs
centimetres in the field.

---

*Written up when #2 merged. Every figure here came from a run in this
repository; none of it is estimated.*

---

# Done

## No regression test behind the `cone_cm` width fix

**Closed** by the wide-beam case in `test/test_map_units.c`. A 25° sonar ranges
a surface 40 m ahead — an **8.87 m** cone radius, three and a half times the old
2.55 m ceiling — the map is built live, scrubbed back before the ray, then
played forward over it so the reconstruction comes out of the ray log. It
asserts the occupied cell at the surface is the same *size* both times, which is
the assertion the scored fixtures cannot make. Mutation-tested: with `cone_cm`
back to a `uint8`, live places an **8.00 m** cell and the reconstruction a
**4.00 m** one, and the case fails.

It is a unit test rather than a fixture because a fixture scores a finished map
against known geometry and never compares live against a reconstruction; the
comparison, not the scene, is the whole content of this one.

**The width is only observable at some resolutions, which is worth knowing
before trusting the fix.** `octomap_insert_ray` floors `hit_depth` at
`coarse_depth`, so the endpoint cell is never coarser than the far-field carve
cell. At the viewer's default 0.25 m leaves that cell is 2 m, and an 8.87 m cone
and a truncated 2.55 m one both measure **2.00 m** — identical, bug or no bug.
The divergence needs a carve cell coarser than 5.1 m, which under the fixed
4096 m root means leaves of 1 m or coarser (`--map-res 1.0` and up), and that is
what the test configures. The scored fixtures all run at 2 m carve cells, which
is a second reason none of them could ever have caught this.

---

## Pruning walked the whole tree on every drain once the byte cap was reached

**Fixed.** The trigger compared `octomap_bytes` — the node pool's *capacity* —
against the cap. A pool never shrinks, so once a map had grown past the pressure
fraction it stayed above it for the rest of the session no matter how much
pruning reclaimed, and a full-tree walk then ran on **every** drain. The map
stayed correct and simply crawled, which is why no fixture noticed: they all
assert what the map contains, and none asserted what it costs.

Two changes. Pressure is now measured with `octomap_live_bytes`, which discounts
the free list, so a productive pass actually relieves the pressure that
triggered it. And a pass that reclaims nothing stands the map down to the
interval schedule rather than retrying on the next drain — at the cap,
subdivision is refused, the tree stops changing shape, and every pass finds the
same unprunable nodes. That bounds the worst case at one walk per interval.

Measured on `statue-solo`, forcing the cap:

| cap | passes before | passes after | rays/s before | rays/s after |
| --- | --- | --- | --- | --- |
| 512 MiB (never reached) | 24 | 24 | 114,365 | 117,850 |
| 32 MiB | 120 | 24 | 45,164 | 119,466 |
| 16 MiB | 1,741 | 31 | 7,607 | 119,533 |

Throughput at the cap is now indistinguishable from throughput with room to
spare, and map quality is unchanged (shape recall 0.99963, IoU 0.490, memory
still pinned at the ceiling).

Guarded by the new **`pressure`** fixture: the `endurance` box, deliberately
over-resolved, run against an 8 MiB ceiling. It asserts `prune_passes_max`, a
count rather than a rate — 13,617 passes against 132 here — because a
deterministic counter has the same teeth on a slow CI runner as on a fast
workstation, and a wall-clock floor tight enough to catch a 12x collapse would
be flaky. `prune_blocks_min` is asserted alongside it so that "few passes"
cannot be bought by never pruning at all. Mutation-tested: the fixture fails
against the old trigger.
