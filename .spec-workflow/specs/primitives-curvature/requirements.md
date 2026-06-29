# Requirements Document

## Introduction

The primitives-curvature feature extends generated motion primitives with an additional state dimension representing path curvature, alongside position and heading. This enables downstream planners and controllers to reason about steering continuity, turning feasibility, and smooth transitions between primitives, especially for kinematic models that are curvature-constrained.

## Alignment with Product Vision

This feature supports the project goal of producing high-quality, reusable optimal primitives for multiple vehicle models by increasing the physical expressiveness of each primitive. Adding curvature to the primitive state improves compatibility with curvature-aware motion planning, reduces ambiguity in steering interpretation, and strengthens interoperability between offline primitive generation and online trajectory selection.

## Requirements

### Requirement 1

**User Story:** As a motion-planning engineer, I want each primitive to include curvature as part of its state representation, so that planning can account for steering geometry and turning continuity.

#### Acceptance Criteria

1. WHEN a primitive is generated THEN the system SHALL include curvature values in the primitive state sequence in addition to position and heading.
2. IF a primitive is exported to JSON THEN the system SHALL persist curvature for every sampled state in a documented, deterministic field.
3. WHEN existing primitive dimensions are parsed AND curvature is present THEN the system SHALL preserve backward-compatible parsing for legacy fields without data loss.

### Requirement 2

**User Story:** As a developer maintaining multiple vehicle models, I want curvature support to behave consistently across supported primitive types, so that data consumers can rely on one common contract.

#### Acceptance Criteria

1. WHEN primitives are generated for any supported model (e.g., differential, omni, Ackermann) THEN the system SHALL output a consistent curvature field schema for all models.
2. IF a model cannot produce physically meaningful curvature THEN the system SHALL explicitly provide a defined fallback value and document the interpretation.
3. WHEN model-specific constraints apply THEN the system SHALL enforce curvature bounds according to that model’s kinematic assumptions.

### Requirement 3

**User Story:** As a data consumer of primitive files, I want clear metadata and documentation for curvature units and semantics, so that I can safely integrate the new dimension without reverse engineering.

#### Acceptance Criteria

1. WHEN curvature is written to outputs THEN the system SHALL define curvature units and sign convention in project documentation.
2. IF a primitive file includes curvature THEN the system SHALL include enough schema context to distinguish curvature from heading rate or steering angle.
3. WHEN a consumer validates output files THEN the system SHALL provide machine-checkable structure that confirms curvature is present and aligned with state samples.

### Requirement 4

**User Story:** As a user with existing primitive datasets, I want a clear migration path, so that I can adopt curvature-enabled primitives without breaking existing tooling.

#### Acceptance Criteria

1. WHEN loading legacy primitive files without curvature THEN the system SHALL either compute or assign a documented default curvature behavior without crashing.
2. IF tooling requests strict curvature-required mode THEN the system SHALL fail with a clear, actionable error message when curvature data is missing.
3. WHEN generating new primitives THEN the system SHALL produce outputs that can be versioned to indicate curvature-enabled format.

## Non-Functional Requirements

### Code Architecture and Modularity
- **Single Responsibility Principle**: Curvature computation, serialization, validation, and documentation updates SHALL be implemented in separate focused modules/functions.
- **Modular Design**: The curvature dimension SHALL be introduced via reusable state/serialization abstractions shared across model-specific generators.
- **Dependency Management**: The implementation SHALL avoid introducing tight coupling between unrelated vehicle models and keep curvature logic behind clear interfaces.
- **Clear Interfaces**: Public data contracts for primitive states and JSON output SHALL explicitly include curvature field names, type, units, and default behavior.

### Performance
- Curvature computation and serialization SHALL not increase primitive generation runtime by more than 10% for equivalent configuration and sample counts.
- Memory overhead for storing curvature SHALL scale linearly with sample count and remain bounded to one additional scalar per state sample.

### Security
- Input/config parsing related to curvature SHALL validate numeric ranges and reject malformed values to avoid corrupted outputs.
- Output serialization SHALL avoid executing dynamic code paths based on untrusted curvature data.

### Reliability
- Primitive generation SHALL be deterministic for identical inputs, including curvature values.
- Validation checks SHALL detect mismatch between state sample length and curvature sample length before writing files.
- Error handling SHALL provide explicit diagnostics for invalid curvature bounds, missing fields, or schema mismatches.

### Usability
- Documentation and output examples SHALL clearly show where curvature appears in each primitive record.
- Error and warning messages related to curvature SHALL explain the issue and the remediation in user-facing terms.
- Configuration defaults SHALL allow users to enable curvature with minimal changes to existing generation workflows.
