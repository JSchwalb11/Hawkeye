# Fleet map — deferred work

Items left on the table when the fleet-map work merged (#2). None of them blocks
that change; each is written up here with the measurement that motivates it, so
the next person does not have to rediscover the number before deciding whether
it is worth the effort.

**Item 1 is done**, and so is the instance-count half of **item 3** — see the
"Done" section at the end for what each turned out to be and what now guards
it.

Issues are disabled on this repository, which is why these live in the tree
rather than in a tracker. If issues are turned on, each heading below is a
ready-made issue body.

Ordered by what I would do first, not by size.

---

## 1. No regression test behind the `cone_cm` width fix

`tl_ray_t.cone_cm` was a `uint8` in centimetres, saturating at 2.55 m. A 25°
sonar passes that at 12 m of range. The map sizes the endpoint cell from this
value, so a reconstruction from the ray log placed a **smaller** occupied cell
than live did — for exactly the wide-beam sensors where the widening matters
most. "Live is replay with the playhead pinned to now" quietly stopped being
true for them.

Widened to `uint16` in `16b3b66`. **No fixture reaches 2.55 m**, so nothing in
CI would catch it regressing.

What would close it: a fixture with a genuinely wide beam — a 25° sonar at 12 m
or more — that ranges a known surface, then asserts the live map and a
scrub-and-replay reconstruction agree on the occupied cell *size*, not just its
position. That assertion is the one the current suite cannot make, and it is the
one that fails if the field narrows again.

Small, self-contained, and it protects a fix that is currently on trust.

---

## 2. Complete the cone sensor model, or keep the axis carve deliberately

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

## 3. The free-space veil is still hazier than it needs to be

After the compositing fix, **72.2%** of surface pixels still read as surface
against a free-hidden reference frame — up from 34.2%, but not 100%. Greedy
meshing has since taken it to **74.7%** at 41% fewer instances; what follows is
the residual that remains.

**Order dependence.** Which of the nearer free cells blend before the nearest
one wins the depth test still depends on draw order, so the veil's exact shade
does too. It is a shade and not a surface — no ordering can hide geometry now
that the surfaces own the depth buffer first — but it means the veil can shift
subtly as chunks re-extract. A `GL_MAX` veil is exactly one layer and
order-independent; measured at **72.9%**, it buys nothing legibility-wise,
flattens the veil's shading, and needs `RL_BLEND_CUSTOM_SEPARATE`. Recorded so
nobody re-runs it expecting more.

**Instance count — addressed by greedy meshing; see the "Done" section.** The
remaining idea, extracting free at a coarser LOD than surfaces, is **not**
done. It still needs an honest majority-free aggregation rule, since the
existing min/max aggregator would report a mostly-unknown cell as free and
quietly overstate coverage in the one view that exists to show it. Meshing has
taken the veil to 162,203 instances; whether a further cut is worth that risk
is now a smaller question than it was.

---

## 4. A TSDF is the right representation for centimetre work

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

---

## The free-space veil cost 273,375 instances because it drew a cube per cell

**Fixed** for the instance-count half of item 3; the order-dependence half is
still open and stays written up above.

Carved cells are now merged into boxes before they are drawn. The octree has
already collapsed whatever agreed cube-wise; greedy meshing takes the runs it
cannot, because a box need not be a cube and need not sit on a power-of-two
boundary. Each chunk is rasterised into a grid at the extraction LOD, the grid
is merged x-then-y-then-z, and one instance is emitted per box. The volume is
unchanged — a re-tiling, not an approximation — and merging stops at a chunk
boundary, so a run is at most one chunk long: eight cells for the shipping
configuration, which is the ceiling on what this can buy.

Measured on `statue-fleet`, `--view left --follow-map`, converged frame at
1280x720 under llvmpipe, cropped to the map viewport. Legibility is the share
of the free-hidden reference's surface pixels that still read as surface. "Veil
px" counts every pixel the veil repaints, so a variant cannot buy legibility by
quietly drawing less carved space without that showing up here. Frame time is
the median of the last 40 frames on this machine, which is slower than the one
that recorded the 207 ms in item 3 — the 348 ms below is the same
configuration, re-measured here so every row is comparable.

| veil | legibility | veil px | instances | frame |
| --- | --- | --- | --- | --- |
| a cube per cell, interior culled (was shipping) | 72.2% | 87,388 | 273,375 | 347 ms |
| a cube per cell, interior drawn | 67.9% | 88,390 | 380,798 | 469 ms |
| merged boxes, gap 0.92 — the same seam area | 67.7% | 89,928 | 162,203 | 215 ms |
| merged boxes, gap 0.84 | 70.8% | 83,865 | 162,203 | 214 ms |
| **merged boxes, gap 0.78 (ships)** | **74.7%** | **78,273** | **162,203** | **211 ms** |
| merged boxes, gap 0.72 | 76.5% | 75,625 | 162,203 | 219 ms |
| merged boxes, one fixed cell of gap | 65.3% | 94,917 | 162,203 | 219 ms |
| merged boxes, interior drawn, gap 0.78 | 70.2% | 79,226 | 146,436 | 223 ms |
| merged boxes, interior culled on the grid, gap 0.92 | 71.3% | 91,716 | 249,760 | 325 ms |

**Conserving the seam area is not enough, and that is the part worth knowing.**
Shrinking a merged box by the per-cell 0.92 leaves exactly the gap its cells
had — 8% of the span either way — and still loses 4.5 points, because eight
small gaps sample what is behind them in eight places and one large gap samples
it in one. The veil's legibility was being bought by the *density* of its
seams, not their area, and merging spends that density. Opening the gap to 0.78
buys the density back as width; it costs 10% of the veil's painted pixels,
which is the honest price and is why the veil-px column is in the table.

Two variants are recorded as failures. Capping the merge length does not
interpolate between meshing and not meshing — rasterising the chunk destroys
the octree's own merging, and capping runs at 1 or 2 cells leaves 3,849,419 and
1,190,882 instances at 13.9% and 34.8%, far worse than either end. And culling
the interior on the grid rather than per leaf keeps almost all the legibility
(71.3%) but fragments the shell into more boxes than the leaf-level cull, so it
gives back most of the instance saving.

The leaf-level interior cull stays in front of the merge. Feeding the interior
in gives the merge more to swallow, and the result is both solider and larger:
70.2% against 74.7% for 10% fewer instances.

Extraction pays for it — 1.66 ms to 3.79 ms mean per frame, 4.73 ms to 7.10 ms
worst — against a frame that drops from 347 ms to 211 ms.

Checked in the coverage view specifically, since that is the one view whose job
is to show what was never looked at: 95.1% to 92.9%, and the same 87,388 to
78,273 veil pixels. The veil there draws less than it did, never more — meshing
cannot overstate coverage, because the merge only sets grid cells a carved leaf
already covered.
