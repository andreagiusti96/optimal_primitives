# Tasks Document

- [x] 1. Add curvature utilities in helper layer
  - File: helper.py
  - Implement `compute_curvature_samples(xs, ys, yaws)` for per-sample curvature derivation.
  - Implement validation helpers for finite values and sample-length consistency.
  - Purpose: Centralize curvature computation and validation as reusable, testable utilities.
  - _Leverage: existing angle normalization and geometry helper patterns in helper.py_
  - _Requirements: 1.1, 2.1, 3.3, NFR Reliability_
  - _Prompt: Role: Python Numerical Developer | Task: Implement robust curvature calculation and validation helpers in helper.py using NumPy and existing helper conventions | Restrictions: Do not change existing helper signatures used elsewhere, keep deterministic behavior, avoid adding new dependencies | Success: Curvature utility returns stable arrays aligned with pose samples and validation catches malformed data._

- [x] 2. Extend trajectory path model with curvature storage
  - File: trajectory.py
  - Extend `Path` dataclass to include `kappas` aligned with `xs`, `ys`, `yaws`.
  - Extend `Path.to_output_format(...)` to emit curvature-enabled pose samples `[x, y, yaw, curvature]` with legacy-read compatibility.
  - Purpose: Carry curvature through pipeline as first-class trajectory data.
  - _Leverage: existing Path dataclass and to_output_format serialization behavior_
  - _Requirements: 1.1, 1.2, 3.3_
  - _Prompt: Role: Python Data Model Engineer | Task: Update Path model to include curvature arrays and output formatting that supports 4-value samples | Restrictions: Preserve existing consumers as much as possible, keep rounding/normalization conventions, ensure no NaN/Inf in serialized values | Success: Path stores and exports curvature consistently with sample-aligned arrays._

- [x] 3. Compute curvature in trajectory generation flow
  - File: trajectory_generator.py
  - After solver sampling of x/y/yaw, compute curvature using helper utility.
  - Apply model-specific fallback/clamp policy where required.
  - Construct `Path` with `xs`, `ys`, `yaws`, `kappas`.
  - Purpose: Ensure each generated trajectory includes curvature data regardless of motion model.
  - _Leverage: existing sampling flow and model dispatch in trajectory_generator.py_
  - _Requirements: 1.1, 2.1, 2.2, 4.1_
  - _Prompt: Role: Motion Planning Backend Developer | Task: Integrate curvature computation into trajectory generation for Ackermann, Diff, and Omni modes | Restrictions: Do not alter solver problem setup in bicycle_acc/unicycle_acc/omni_acc beyond needed integration, keep runtime overhead low | Success: All generated Path instances include valid curvature arrays across supported models._

- [x] 4. Add curvature metadata and schema fields to JSON writer
  - File: generate_optimal_primitives.py
  - Extend header metadata with `num_of_curvatures` and `curvatures` catalog.
  - Add per-primitive fields: `start_curvature`, `end_curvature`, `start_curvature_index`, `end_curvature_index`.
  - Ensure `poses` entries are 4-value arrays in curvature-enabled output.
  - Purpose: Match exact output contract used by sample schema.
  - _Leverage: existing create_header/write_to_json flow and heading index mapping_
  - _Requirements: 1.2, 2.1, 3.1, 3.2, 4.3_
  - _Prompt: Role: Python Serialization Engineer | Task: Update JSON serialization to match sample_primitives/ackermann_with_curvature.json schema including curvature catalog and index mapping | Restrictions: Preserve existing non-curvature primitive fields, maintain deterministic ordering, avoid breaking metadata semantics | Success: Output JSON includes required curvature metadata and primitive fields with correct indexing._

- [x] 5. Implement schema consistency guards before output write
  - File: generate_optimal_primitives.py
  - Validate pose length (4 values), start/end curvature equality with first/last sample, and index-to-catalog mapping.
  - Validate `num_of_curvatures == len(curvatures)`.
  - Purpose: Prevent invalid curvature files from being written.
  - _Leverage: write_to_json control flow and helper validation utilities_
  - _Requirements: 3.3, 4.2, NFR Reliability_
  - _Prompt: Role: Defensive Programming Specialist | Task: Add strict schema validation checks and clear error messages before writing primitives.json | Restrictions: Fail fast with actionable diagnostics, do not silently repair corrupt trajectories in strict mode | Success: Invalid schema states are detected and reported before file emission._

- [x] 6. Add compatibility mode behavior for legacy primitive handling
  - Files: generate_optimal_primitives.py, README.md
  - Define and implement behavior when curvature is missing (default/infer mode vs strict mode).
  - Document strict mode failure semantics and fallback policy.
  - Purpose: Preserve migration path for existing tooling and datasets.
  - _Leverage: existing config loading/documentation patterns_
  - _Requirements: 1.3, 4.1, 4.2_
  - _Prompt: Role: Backend Compatibility Engineer | Task: Introduce documented compatibility/strict handling for curvature absence and enforce predictable behavior | Restrictions: Do not remove existing fields, keep defaults safe, ensure explicit errors in strict mode | Success: Legacy files remain consumable in compatibility mode and strict mode fails with clear messages._

- [x] 7. Add configuration keys for curvature policy and bounds
  - Files: config.json, config_ackermann.json, config_diff.json, config_omni.json
  - Add keys for curvature enablement, fallback policy, and optional model-specific bounds.
  - Purpose: Make curvature behavior configurable without source edits.
  - _Leverage: existing config schema and parsing flow_
  - _Requirements: 2.2, NFR Usability, NFR Code Architecture_
  - _Prompt: Role: Configuration Engineer | Task: Extend config files and parser assumptions with curvature policy fields compatible with current workflows | Restrictions: Keep defaults backward-compatible, avoid mandatory new keys for existing runs unless safe defaults are provided | Success: Curvature behavior is configurable and existing configs continue to work._

- [x] 8. Update documentation with explicit curvature contract
  - Files: README.md, .spec-workflow/specs/primitives-curvature/design.md (if needed for parity notes)
  - Document curvature units, sign convention, metadata fields, and primitive sample format.
  - Add example mapping to sample schema (`sample_primitives/ackermann_with_curvature.json`).
  - Purpose: Ensure data consumers can integrate without reverse engineering.
  - _Leverage: existing output structure section in README.md_
  - _Requirements: 3.1, 3.2, NFR Usability_
  - _Prompt: Role: Technical Writer for Robotics APIs | Task: Update output format docs to fully describe curvature-enabled schema and migration notes | Restrictions: Keep documentation precise and implementation-consistent, avoid ambiguous terminology between steering angle and curvature | Success: Documentation clearly defines all curvature fields and usage semantics._

- [ ] 9. Add unit tests for curvature math and serialization primitives
  - Files: new tests module (for example tests/test_curvature_utils.py, tests/test_output_schema.py) or project-consistent alternatives
  - Validate straight-line zero curvature, constant-radius arc behavior, endpoint stability, and NaN/Inf guards.
  - Validate pose sample shape and curvature index mapping logic.
  - Purpose: Protect core curvature logic from regressions.
  - _Leverage: existing script-style validation approach and sample primitive fixtures_
  - _Requirements: 1.1, 1.2, 2.1, 3.3, NFR Reliability_
  - _Prompt: Role: Python Test Engineer | Task: Add focused tests for curvature computation and schema serialization invariants | Restrictions: Tests must be deterministic and independent of external services, keep fixture data minimal and readable | Success: Tests cover critical curvature math and schema edge cases with reliable pass/fail behavior._

- [ ] 10. Add integration verification against reference curvature sample schema
  - Files: test harness script or integration test module; sample_primitives/ackermann_with_curvature.json for reference comparison
  - Generate primitives for at least ackermann config and verify presence and consistency of curvature catalog + primitive fields.
  - Compare structural parity against reference schema fields (not exact numeric equality).
  - Purpose: Guarantee produced files match required contract.
  - _Leverage: generate_optimal_primitives.py CLI and reference sample file_
  - _Requirements: 2.1, 3.1, 3.3, 4.3_
  - _Prompt: Role: Integration QA Engineer | Task: Implement an automated schema-parity check against sample_primitives/ackermann_with_curvature.json structure | Restrictions: Validate schema/consistency, not solver-specific numeric identity; include clear diagnostics for missing fields | Success: Integration checks confirm generated outputs follow required curvature schema contract._

- [ ] 11. Run end-to-end generation and performance validation
  - Files: outputs/* generated artifacts, analysis script outputs
  - Run generation for diff/ackermann/omni and confirm curvature fields exist and are valid.
  - Measure runtime delta against pre-curvature baseline and verify within target threshold.
  - Purpose: Validate production readiness across all supported models.
  - _Leverage: generate_optimal_primitives.py, analyze_primitives.py, existing output folders_
  - _Requirements: 2.1, 2.2, NFR Performance, NFR Reliability_
  - _Prompt: Role: Robotics Verification Engineer | Task: Execute full generation workflow and verify correctness/performance of curvature-enabled outputs | Restrictions: Keep benchmark setup consistent with baseline conditions, document any model-specific deviations | Success: End-to-end runs succeed for all models with valid schema and acceptable performance overhead._

- [ ] 12. Final cleanup and release-readiness review
  - Files: modified source/config/docs/test files for this spec
  - Remove temporary debug code, ensure comments and naming are consistent, and verify no unrelated regressions.
  - Confirm spec documents (requirements/design/tasks) remain aligned with implementation-ready scope.
  - Purpose: Deliver a clean, maintainable implementation plan ready for execution.
  - _Leverage: project steering docs and spec workflow artifacts_
  - _Requirements: All_
  - _Prompt: Role: Senior Maintainer | Task: Perform final quality pass on scope, consistency, and maintainability before implementation starts | Restrictions: Do not expand scope beyond approved requirements/design, avoid speculative refactors | Success: Task set is complete, coherent, and ready for implementation tracking._
