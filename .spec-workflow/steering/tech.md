# Technology Stack

## Project Type
Python-based robotics library and CLI workflow for offline generation and analysis of motion primitives using optimal control, built to support Navigation2 (Nav2) lattice planning workflows.

## Core Technologies

### Primary Language(s)
- **Language**: Python (tested in project README with Python 3.8 and 3.10)
- **Runtime/Compiler**: CPython runtime
- **Language-specific tools**: pip for dependency installation, virtual environment (.venv) for isolation

### Key Dependencies/Libraries
- **Navigation2 (Nav2) lattice primitive specification**: Foundational planner contract that defines the output structure and integration target for generated primitives.
- **NumPy**: Core numerical computation for geometry, interpolation, and array operations.
- **Matplotlib**: Primitive visualization and analysis plots.
- **CasADi**: Symbolic and numeric optimization backend.
- **Rockit**: Optimal control problem modeling and solving.
- **Rtree**: Spatial indexing for fast geometric lookup when pruning/selecting trajectories.
- **Python standard library**: argparse, json, pathlib, logging, datetime, enum, dataclasses, typing, collections.

### Application Architecture
Monolithic, module-oriented Python architecture with clear functional boundaries:
- Entry/CLI layer: reads config, runs generation, writes artifacts.
- Generation orchestrator: computes minimal spanning primitive set and applies filtering logic.
- Model-specific optimization modules: unicycle, bicycle/Ackermann, omni acceleration models.
- Trajectory domain layer: trajectory/path representations and helper math utilities.
- Analysis layer: post-generation inspection and plotting utilities.

### Data Storage (if applicable)
- **Primary storage**: Local file system.
- **Caching**: In-memory structures only during execution (no dedicated cache service).
- **Data formats**: JSON for configs and primitive outputs, PNG images for visualizations.

### External Integrations (if applicable)
- **APIs**: No runtime external API dependency in normal generation flow.
- **Protocols**: File I/O and local process execution.
- **Authentication**: Not applicable.
- **Planner ecosystem integration**: Navigation2 (Nav2), specifically Smac lattice planning, is the primary integration foundation and output contract target.

### Monitoring & Dashboard Technologies (if applicable)
- **Dashboard Framework**: Not part of core generator.
- **Real-time Communication**: Not applicable.
- **Visualization Libraries**: Matplotlib for static plots.
- **State Management**: Files are source of truth; runtime state is in-memory.

## Development Environment

### Build & Development Tools
- **Build System**: None required; direct Python execution.
- **Package Management**: pip with requirements.txt.
- **Development workflow**: Edit-run-validate loop via CLI scripts and generated artifacts.

### Code Quality Tools
- **Static Analysis**: No repository-enforced static analysis tool currently documented.
- **Formatting**: No repository-enforced formatter currently documented.
- **Testing Framework**: No automated test framework currently documented in repository root.
- **Documentation**: README-based manual documentation.

### Version Control & Collaboration
- **VCS**: Git.
- **Branching Strategy**: Not formally documented; recommend short-lived feature branches with pull request review.
- **Code Review Process**: Not formally documented in repository; recommend PR-based review for generator, schema, and config changes.

### Dashboard Development (if applicable)
- **Live Reload**: Not applicable.
- **Port Management**: Not applicable.
- **Multi-Instance Support**: Not applicable.

## Deployment & Distribution (if applicable)
- **Target Platform(s)**: Local engineering machines (Windows/Linux/macOS where Python dependencies are available).
- **Distribution Method**: Source repository checkout with dependency installation from requirements.txt.
- **Installation Requirements**: Python environment compatible with dependencies, solver dependencies required by CasADi/Rockit stack.
- **Update Mechanism**: Pull latest repository changes and reinstall/update dependencies as needed.

## Technical Requirements & Constraints

### Performance Requirements
- Primitive generation should complete in practical engineering time for typical configs (minutes, not hours).
- Spatial filtering and minimal-set selection should remain efficient as trajectory candidates grow.
- Visualization generation should not dominate total runtime for standard output sizes.

### Compatibility Requirements  
- **Platform Support**: Python-capable developer environments; project has documented testing on Python 3.8 and 3.10.
- **Dependency Versions**: See requirements.txt lower bounds; solver compatibility must be maintained for CasADi/Rockit.
- **Standards Compliance**: Output JSON must remain compliant with the Navigation2 lattice primitive schema and behavior expectations.

### Security & Compliance
- **Security Requirements**: Validate and sanitize config inputs before use; avoid executing untrusted code from configs.
- **Compliance Standards**: No explicit regulatory framework currently required for this repository.
- **Threat Model**: Primary risks are malformed configuration and corrupted output artifacts, not external network attacks.

### Scalability & Reliability
- **Expected Load**: Single-user, offline batch generation with potentially large candidate trajectory sets.
- **Availability Requirements**: No service uptime requirement; reliability is measured by deterministic and repeatable run outcomes.
- **Growth Projections**: Increased model complexity (for example additional state dimensions such as curvature) and larger primitive libraries over time.

## Technical Decisions & Rationale

### Decision Log
1. **Python + Scientific Stack**: Chosen for rapid iteration, readability, and mature numerical tooling; trade-off is lower raw performance versus compiled languages.
2. **CasADi/Rockit for Optimal Control**: Chosen to express and solve constrained trajectory optimization problems cleanly; trade-off is solver dependency complexity.
3. **Navigation2-First Output Contract**: Chosen to make generated primitives directly usable in Nav2/Smac planning workflows; trade-off is tighter coupling to Nav2 schema conventions.
4. **Rtree Spatial Index for Minimality Checks**: Chosen to reduce geometric search cost in pruning/coverage logic; trade-off is additional dependency and platform install considerations.
5. **File-Based Batch Workflow**: Chosen to keep generation deterministic and reproducible; trade-off is absence of real-time interactive orchestration.

## Known Limitations

- **Limited Formal Tooling**: Static analysis, formatting, and automated tests are not standardized in-repo, increasing regression risk as complexity grows.
- **Dependency Friction**: CasADi/Rockit and spatial-index dependencies can be environment-sensitive and may require careful setup.
- **Schema Evolution Risk**: As primitives evolve (for example adding curvature), backward compatibility requires explicit versioning and validation strategy.
- **Runtime Scaling Constraints**: Large parameter sweeps or very high heading/resolution combinations may increase generation time significantly.
