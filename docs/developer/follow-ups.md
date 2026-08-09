# Fleet map — deferred work

Everything the fleet-map work left open when it merged (#2) has now been
resolved except one item, and two defects surfaced along the way that no fixture
had reached. The record is below: what each turned out to be, and what guards it
now. Entries are kept after they close, with their numbers, because several of
them closed by being disproved and the disproof is the useful part.

Issues are disabled on this repository, which is why this lives in the tree
rather than in a tracker.

---

## Still open

### Extract carved space at a coarser LOD than surfaces

The unpursued half of the veil work. Greedy meshing took the instance count
down by merging runs the octree could not, but a chunk bounds a run, so a merge
is at most one chunk long. Dropping the *resolution* of carved space is the
other lever and it is untouched.

The catch is the aggregation rule, and it is the reason this was not just done:
`lod_rec` reports a coarse cell as free if **any** descendant is free, so a
mostly-`UNKNOWN` cell would render as carved and quietly overstate coverage —
in `MAP_DRAW_COVERAGE`, the one view that exists to show what was never looked
at. A majority-free rule plus a coverage-view check is the work.

---

# Done

## A sub-voxel offset on occupied leaves

**Closed**, and worth less than predicted for a reason worth recording.

Each leaf now carries a three-byte offset from its own centre, a quarter-weight
running mean of the returns landing in it, reported by `om_node_surface()`. The
prediction from the TSDF spike was that this captures the field's 1.8x linear
advantage. Measured on `statue-precision`, it captures **1.25x** — surface
placement 4.4 mm from the cell centre, 3.5 mm from the offset — and 1.01–1.04x
on every other fixture.

The 1.01x is not a failure of the offset, it is the fixtures being honest: they
all fly beams whose footprint dwarfs their leaves, so their placement error
belongs to the sensor and no amount of representation recovers it. The offset
can only ever recover the part of the error that is the *cell's*.

**The argument that carries it is memory, not accuracy.** It costs 25% of the
node pool — `om_node_t` goes 16 to 20 bytes, and alignment means any addition at
all costs the full four. Against that, the alternative way to buy surface
placement:

| | node pool | centre placement | offset placement |
| --- | --- | --- | --- |
| 7.8 mm leaves | 128 MiB | 4.4 mm | — |
| 7.8 mm leaves + offset | 160 MiB | 4.4 mm | **3.5 mm** |
| 3.9 mm leaves | ~256 MiB | 3.7 mm | — |
| 3.9 mm leaves + offset | 320 MiB | 3.7 mm | 3.5 mm |

Subdivision costs four times the memory for less placement accuracy, and the
last row says the offset has already reached the sensor's floor — the extra
octree level buys nothing once it is there.

**Things measured and discarded along the way.** The weight of the running mean
does not matter: 1, 1/2, 1/4 and 1/8 all land between 1.24x and 1.26x, so it is
set to a quarter on the principle that it should settle over the same span of
evidence the occupancy does, not because a sweep chose it. Recording only the
boresight sample rather than every cone sample is *worse* (1.08x), and scaling
each update by the sample's own share of the return is worth about 0.01 —
inside the noise, kept because it costs nothing and is the more defensible rule.

**What guards it.** `statue-precision` asserts a placement gain of at least 1.15
and that 99% of scored cells carry an estimate. Three unit cases in
`test_map_units` cover what the fixture provably cannot: that the estimate beats
the centre at all, that a prune collapsing eight cells carries their estimate
into the parent's frame rather than dropping it, and that a subdivision does
*not* hand the parent's estimate to all eight children — a point inside one
child is not a point inside its seven siblings.

That last group exists because the obvious fixture threshold was written first
and then measured: reverting either the prune or the subdivide path leaves
`statue-precision` at 1.25x and 100.0%, because the cells involved are re-hit
immediately afterwards and re-establish their own estimates. The threshold's
comment says so, so the next person does not assume it covers them.

---

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

## The cone sensor model — rejected once, then landed

Rejected in #2 as "half a model change", and that judgement was right about the
half and wrong about the model. Carving the beam's cone while still marking the
return at a point on the axis erodes any surface whose footprint exceeds a leaf,
and the fixtures said so. Completing it — carve the cone *and* spread the return
across the footprint at reduced weight — reverses the verdict.

The trap worth keeping: realising the cone by inflating each node's box before
clipping the axis against it is correct laterally, but a cube inflation extends
the sweep *along* the beam too, so a carve told to stop a hit-cell short still
reached a cone-radius past that. On `cone` — a flat wall, head-on — that alone
lost 36% of the surface at 0.0000 m RMS. `t_end` is pulled back by the cone
radius as well as the hit cell.

Two calibrations that had to be measured rather than argued:

- **The evidence budget must not be conserved.** Sharing one return's increment
  across the cap leaves almost nothing above the occupancy threshold and scores
  *worse* than not spreading (statue-solo shape recall 0.5402 against 0.8932).
  Un-normalised is both best and the option with no free parameter.
- **Only the boresight sample earns the post-return grace period.** Letting each
  spread sample shield its own cell immunised the whole footprint on one reading
  and took `cone`'s false-occupied from 0.031 to 0.102.

## `corridor` was validating the map against a sensor that does not exist

Found while completing the cone model, and independent of it. MAVLink defines an
`OBSTACLE_DISTANCE` sector's distance as the **nearest** obstacle in that sector;
the injector cast one pencil ray down the axis and reported that, modelling a
sensor able to see past an obstacle filling most of its own cone.

Against a faithful sensor the pencil map fails `corridor` on two thresholds —
surface RMS 0.9654 m against a 0.80 ceiling, false-occupied 0.4619 against 0.30
— while the cone map passes at 0.2883 and 0.0613. Each map model passes only
against the sensor model that matches it, and the spec says which one is real.

(Both columns re-measured on the merged branch from one recording, the pencil
side via `git show 43b3513:src/octomap.c`. An earlier draft of this entry gave
the cone map 0.2203 and 0.0319; those came from a working branch before the
merge and do not reproduce here.)

The sector's *elevation* extent is unspecified in the message, so treating it as
a circular cone remains a modelling choice — but it is the same choice the
endpoint widening already made.

## A return shielded its cell only against evidence that arrived first

The clear-on-hit rule shipped in #2 was described there as order-independent. It
is not, and the claim should have been tested rather than asserted. It wipes free
evidence that arrived *before* a return and does nothing about the reverse, which
is just as physical: a beam terminates in a cell and, tens of milliseconds later,
the neighbouring sectors of the same sweep skim through it.

Measured on a bare cell with a hit and twelve grazing rays:

| ordering | result |
| --- | --- |
| misses first, then the hit | log-odds +17, **occupied** |
| the hit first, then misses | log-odds −70, **free** |

Four further sweeps never recovered it — each sweep contributes one hit against a
dozen misses. Surfaced by the TSDF spike, which is the one representation where
the asymmetry does not arise.

A cell cannot tell a grazing pass from a removed obstacle: both are rays passing
through and terminating elsewhere. Time separates them — a sweep is over in well
under a second, a removal keeps producing misses indefinitely — so a return now
shields its cell for `hit_grace_ms` (1 s) and no longer. Costs nothing: a spare
bit in `flags` and the `last_seen_ms` already there.

Pinned by `test_grazing_order`, since no scored fixture reaches this ordering.

## A TSDF is the right representation for centimetre work — refuted

The one item that closed by being disproved. A voxel-hashed TSDF and a binary
control grid were built on one lattice with one DDA traversal, so any difference
is representational rather than an implementation artifact; the harness was
validated by reproducing the shipped fixture to 0.00091 m against the 0.0009 it
published at the time. (`statue-precision` now reports 0.0019 m — the beam model
costs it about a millimetre. The spike's comparison is internal to itself, so
its conclusion does not move.)

| claim | verdict |
| --- | --- |
| a zero crossing survives a grazing pass | **holds** — and it is what exposed the ordering defect above |
| a removed surface still clears | **holds** — 16 cells to 0, backstop intact |
| resolves to voxel/10; 5 cm matches 5 mm at 1/125 the cells | **refuted, both numbers** |

With the wire's 1 cm quantisation removed so the sensor's floor is not mistaken
for the representation's, the field resolves to a flat cell/6 and the binary grid
to cell/3.4 — a **1.8x** linear gain, not 10x, and not hiding in the tuning.
Matching the shipped map's 3.27 mm needs ~15 mm voxels, where the field holds
6.3x more cells at **374 MiB against 38.3 MiB**. The 1/125 claim is wrong by
roughly 800x *and* in the wrong direction, because the octree carves free space
hierarchically while a uniform lattice pays full resolution for every metre of
empty air.

**Do not do the rewrite.** The accuracy that would justify it is available far
more cheaply — see "a sub-voxel offset on occupied leaves" above.
