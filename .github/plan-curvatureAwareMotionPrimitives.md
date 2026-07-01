# Plan: Curvature-aware motion primitive generation (diff drive)

Add a curvature dimension to the primitive generation loop, mirroring how `num_of_headings` drives the heading discretization today. The user specifies `num_of_curvatures` and `turning_radius`; the software builds a symmetric curvature catalog, enumerates every (start_κ, end_κ) pair alongside (start_heading, end_heading, end_point), and softly biases the diff-drive OCP so that the trajectory's **geometric** start/end curvature (as computed by `compute_curvature_samples` in `helper.py`) is near the target catalog values. The R-tree dedup key is extended to include end_κ so distinct-curvature variants survive. Output JSON keeps its current schema; `start_curvature` / `end_curvature` store the *target* catalog values (index-consistent).

Scope for this iteration: **diff only**. Omni / Ackermann remain functional with keep κ_start = κ_end = null (free curvature).

## Phase 1 — Config & curvature catalog

1. Add config keys (with defaults) in `read_config` (`generate_optimal_primitives.py`):
   - `num_of_curvatures` (int, default `null` → curvature is free, preserving current behavior)
   - `curvature_weight` (float, default e.g. 100.0) — soft-penalty weight in OCP
   - `curvature_tolerance` (float, default 0.1) — post-solve max allowed |κ_actual − κ_target| before rejecting the trajectory
2. In `OptimalGenerator.__init__`, build `self.curvatures` from `num_of_curvatures` and `turning_radius`:
   - **Scheme**: symmetric around 0, always including 0, max |κ| = `1/turning_radius`, uniform in κ.
   - `num_of_curvatures = 0 or null` → `null` → curvature is free and does not need to be constarined (current behaviour).
   - `num_of_curvatures = 1` → `[0.0]`.
   - `num_of_curvatures` odd (≥3) → `np.linspace(-κmax, +κmax, N)` (includes 0).
   - `num_of_curvatures` even → replace by odd = N+1 with a warning (rationale: keeping 0 is essential for straight motion / in-place turns / horizontal slides).
3. Expose the catalog to `TrajectoryGenerator` (constructor arg) so it can pass the current (κ_s, κ_e) target into `_create_path` and forward to the motion-model backend.

## Phase 2 — OCP soft boundary-curvature penalty (unicycle_acc.py)

Modify `unicycle_acc.py`:
1. Add two new `ocp.parameter()` inputs: `kappa_start`, `kappa_end`, and one weight `curvature_weight`.
2. Define new variable `k` reprsenting the geometric curvature of the path (`k` does not depend directly from `v`, `θ` or `w`, but only on `x` and `y`)
3. Add soft objective term expressing the boundary geometric curvature. Concretely:
   - `ocp.add_objective( curvature_weight * ( (ocp.at_t0(k) - kappa_start)**2 + (ocp.at_tf(k) - kappa_end)**2 ) )`
   - if `kappa_start` or `kappa_start` are `null` (or `None`) ignore curvature in the objective function
4. Keep hard boundary constraints on x, y, θ; keep `v(0) = 0`, `w(0) = 0` as they are today. The penalty only becomes meaningful for the terminal boundary and for the shape around t=0+ (curvature grows with v). Alternative — dropping `v(0)=0` hard constraint — listed in *Further Considerations*.
5. Extend `return_poses` signature to accept `kappa_start, kappa_end, curvature_weight`; wire values through `ocp.set_value`.
6. Extend output of `return_poses` to include `k`.
7. Mirror the change for later use in `omni_acc.py` and `bicycle_acc.py`, but call sites for those two pass `(null, null, curvature_weight)` in this iteration.

## Phase 3 — Threading κ through the generator

1. `TrajectoryGenerator._create_path` gains parameters `start_kappa`, `end_kappa` and forwards them to the motion-model backend. For diff it passes them to `unicycle_acc.return_poses`. For ackermann / omni it is accepted but ignored (or forwarded with zeros).
2. `TrajectoryGenerator._calculate_trajectory` and `generate_trajectory` gain the same two args and pass them into `_create_path`.
3. `_create_path` does not need to call `compute_curvature_samples` anymore because `k` comes directly from `return_poses`. (The call to `compute_curvature_samples` remains in place for debugging and tests purposes but is commented).
4. After `_create_path`, verify `|κ_actual − κ_target|` at the first and last samples (using `path.kappas[0]` and `path.kappas[-1]`) against `curvature_tolerance`; if either fails, return `None` (treat as infeasible). This is the acceptance filter that makes the "soft" constraint useful in practice.
5. Trajectory metadata: store `target_start_kappa` / `target_end_kappa` on `TrajectoryParameters` (extend the dataclass — remember it's `frozen=True`, add new fields with defaults).

## Phase 4 — Enumeration loop + R-tree key extension

In `OptimalGenerator._generate_minimal_spanning_set`:
1. Wrap the existing (target_point × target_heading) loop with two additional loops over `start_kappa in self.curvatures` and `end_kappa in self.curvatures`. Outer-most stays `start_heading`.
2. Store `start_kappa` in the per-start-angle bucket: change `quadrant1_end_poses[start_heading]` to key on `(start_heading, start_kappa)` and value stays a list of `(target_point, target_heading, end_kappa)` — this keeps the R-tree per (start_heading, start_kappa) which is the correct dedup scope.
3. `_is_minimal_trajectory`: extend the pose tuple stored/retrieved from the R-tree to include `end_kappa`. Reject a candidate as redundant only when a prior end pose matches in (x, y, yaw) **and** in end_kappa (within a new `CURVATURE_THRESHOLD = 0.5 * min catalog spacing`). This lets distinct-curvature variants coexist.
4. `_compute_min_trajectory_length` unchanged (heading-driven).
5. `stopping_threshold` / `max_wavefront_expansion` counters now operate per (start_heading, start_kappa) bucket.

## Phase 5 — Symmetry / flipping

1. In `_create_complete_minimal_spanning_set`, the four-way flip (X, Y, both) must invert curvature signs consistently:
   - Flip across X-axis: κ → −κ (both start and end).
   - Flip across Y-axis: κ → −κ.
   - Flip across both: κ → κ (double sign flip cancels).
2. When invoking `generate_trajectory` for a flipped variant, pass the flipped (start_kappa, end_kappa). Because the catalog is symmetric around 0, `−κ` is always in the catalog.

## Phase 6 — Output & special primitives

1. `_add_in_place_turns` and `_add_horizontal_motions` in `optimal_generator.py`: force `target_start_kappa = target_end_kappa = 0.0` on the synthesized `TrajectoryParameters`. These 0-values must be present in `self.curvatures`; add a runtime assertion.
2. `create_curvature_catalog` in `generate_optimal_primitives.py`: switch source from "read actual `path.kappas[0/-1]`" to "read `parameters.target_start_kappa` / `parameters.target_end_kappa`" so the emitted `curvatures` list is exactly `self.curvatures` from the generator, and every primitive's start/end index is guaranteed to resolve.
3. In `write_to_json`: `start_curvature`, `end_curvature`, `start_curvature_index`, `end_curvature_index` all derive from the target values. Keep `poses` including per-sample `kappa` from `compute_curvature_samples` unchanged.
4. `num_of_curvatures` in `lattice_metadata` now equals `config['num_of_curvatures']` (or `len(self.curvatures)` after the odd-count normalization).

## Relevant files

- [generate_optimal_primitives.py](generate_optimal_primitives.py) — `read_config` (new config keys, defaults), `create_curvature_catalog` (source from targets), `write_to_json` (index resolution).
- [optimal_generator.py](optimal_generator.py) — `OptimalGenerator.__init__` (build catalog), `_generate_minimal_spanning_set` (nested κ loops, per-(heading,κ) buckets & R-tree), `_is_minimal_trajectory` (extended key), `_create_complete_minimal_spanning_set` (sign-flip κ), `_add_in_place_turns` / `_add_horizontal_motions` (target κ = 0).
- [trajectory_generator.py](trajectory_generator.py) — `_create_path`, `_calculate_trajectory`, `generate_trajectory` (thread κ, tolerance filter using `path.kappas[0/-1]`).
- [unicycle_acc.py](unicycle_acc.py) — add `kappa_start`, `kappa_end`, `curvature_weight` parameters and objective term; extend `return_poses`.
- [omni_acc.py](omni_acc.py), [bicycle_acc.py](bicycle_acc.py) — signature parity only; ignore κ this iteration.
- [trajectory.py](trajectory.py) — extend `TrajectoryParameters` (`target_start_kappa`, `target_end_kappa`, defaults 0.0).
- [helper.py](helper.py) — no changes; reuse `compute_curvature_samples`.
- [config_diff.json](config_diff.json) — add `num_of_curvatures`, `curvature_weight`, `curvature_tolerance`. Default to `num_of_curvatures = null` (i.e. current behavior).
- [config_omni.json](config_omni.json), [config_ackermann.json](config_ackermann.json) — same additive defaults (`num_of_curvatures = null`).

## Verification

1. **Backward compatibility**: run `python generate_optimal_primitives.py --config ./config_diff.json` with `num_of_curvatures = null`; diff the resulting `outputs/diff_primitives/diff_primitives_wcurv.json` against the current one — the primitive count and geometry should match (curvature catalog becomes `null`).
2. **Small catalog (N=3)**: set `num_of_curvatures = 3` in `config_diff.json`. Verify:
   - `lattice_metadata.curvatures` = `[-1/R, 0, +1/R]`, `num_of_curvatures = 3`.
   - Every primitive's `start_curvature_index` and `end_curvature_index` resolve into `[0,1,2]`.
   - Straight-line primitive from (0,0,0) has both indices = 1.
   - Total primitive count is roughly ≤ 9× the N=1 count (K² × old count in the worst case, less after dedup).
3. **Larger catalog (N=5)**: set `num_of_curvatures = 5`. Spot-check figures in `outputs/diff_primitives/figures/`: primitives leaving (0,0,0) with target `κ_start = ±1/R` should visibly curve immediately, while `κ_start = 0` primitives should start straight.
4. **Actual vs target κ**: `analyze_primitives.py` (or a short ad-hoc script) — for each primitive, compare `poses[0][3]` and `poses[-1][3]` against `start_curvature` / `end_curvature`; assert the delta ≤ `curvature_tolerance` (this is exactly the acceptance filter, so should hold by construction).
5. **Symmetry**: verify that for every quadrant-1 primitive with (κ_s, κ_e), the mirror primitive in quadrant 2 has (−κ_s, −κ_e).
6. **Omni / Ackermann smoke test**: run with default configs (implicit `num_of_curvatures = null`); output JSON must still validate and match previous outputs modulo the added `start_curvature*`/`end_curvature*` fileds.

## Decisions

- Catalog is auto-generated symmetric-around-0 uniform-in-κ with max |κ| = `1/turning_radius`.
- Soft constraint uses `curvature_weight` in the OCP objective; a post-solve `curvature_tolerance` check rejects trajectories whose actual boundary curvature drifts too far — this keeps the JSON index-consistent while remaining "soft" (no infeasibility from IPOPT).
- `start_curvature` / `end_curvature` in the JSON are the **target** catalog values; per-sample `kappa` inside `poses` remains the actual geometric value.
- In-place turns and horizontal slides always take κ = 0 at both ends.
- Diff only this iteration; omni/ackermann kept working via `num_of_curvatures = 1` default.
- R-tree dedup extended to include end κ so multiple curvature variants to the same (x,y,yaw) coexist.

## Further Considerations

1. **Curvature spacing**: uniform in κ is simple but front-loads density near |κ|=1/R. Alternative: uniform in *turning radius* (values `1/R_i` with `R_i` spaced in radius or in angle) yields more values near κ=0, which planners typically prefer. Options: **A. Uniform in κ (default)** / **B. Uniform in R** / **C. Uniform in arc-turn-angle over grid_resolution**. Recommend A now; revisit after seeing N=5 figures.
2. **Even num_of_curvatures**: current plan dispaly a warn and promotes to odd.
3. **OCP formulation**: `k` reprsents the purely geometric curvature of the path. It does not depend directly on `v`, `θ` or `w`, but only on `x` and `y`. This definition extends naturally to the otehr motion models (omni and ackermann).
