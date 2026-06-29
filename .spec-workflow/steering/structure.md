# Project Structure

## Directory Organization

```text
optimal_primitives/
├── generate_optimal_primitives.py   # Main CLI entry point for primitive generation
├── optimal_generator.py             # Minimal spanning set / wavefront orchestration
├── trajectory_generator.py          # Model-dispatch trajectory creation logic
├── trajectory.py                    # Core trajectory/path data structures
├── helper.py                        # Shared geometry and math helpers
├── constants.py                     # Global constants and version values
├── bicycle_acc.py                   # Ackermann/bicycle optimal control model
├── unicycle_acc.py                  # Differential-drive optimal control model
├── omni_acc.py                      # Omni model optimal control model
├── analyze_primitives.py            # Post-generation analysis and plotting
├── config.json                      # Base generation config
├── config_ackermann.json            # Ackermann-specific config
├── config_diff.json                 # Differential-drive-specific config
├── config_omni.json                 # Omni-specific config
├── requirements.txt                 # Python dependency constraints
├── README.md                        # User/developer documentation
├── outputs/                         # Generated primitive artifacts and figures
├── sample_primitives/               # Reference primitive datasets
├── .spec-workflow/                  # Spec workflow docs, templates, approvals
└── .vscode/                         # Workspace-local editor/MCP settings
```

The repository uses a mostly flat layout, where top-level modules are grouped by role (entrypoint, orchestration, model solvers, domain objects, utilities, configuration, and generated artifacts).

## Naming Conventions

### Files
- **Components/Modules**: `snake_case.py` (for example `trajectory_generator.py`, `optimal_generator.py`)
- **Services/Handlers**: Not split into dedicated service files; orchestration is in generator modules.
- **Utilities/Helpers**: `helper.py` (shared helper module in snake_case)
- **Tests**: Not currently present. Recommended future pattern: `test_<module>.py` under a `tests/` directory.

### Code
- **Classes/Types**: `PascalCase` (for example `OptimalGenerator`, `TrajectoryGenerator`, `TrajectoryParameters`)
- **Functions/Methods**: `snake_case`
- **Constants**: `UPPER_SNAKE_CASE` where module constants are used (for example thresholds), with a few legacy names preserved.
- **Variables**: `snake_case` for local/module variables

## Import Patterns

### Import Order
1. Python standard library imports
2. Third-party dependencies (`numpy`, `matplotlib`, `casadi`, `rockit`, `rtree`)
3. Internal project modules (`trajectory`, `helper`, model modules)
4. Relative imports (currently project favors same-directory absolute module imports)

### Module/Package Organization
```text
- Flat module namespace at repository root
- Internal imports are direct module imports by filename
- Entry points import orchestrator modules, which import domain/model modules
- Model-specific files are consumed through dispatch logic in trajectory generation
```

## Code Structure Patterns

### Module/Class Organization
```text
1. Imports and logger setup
2. Class/type definitions (often with internal enums)
3. Constructor reading configuration
4. Core algorithmic methods
5. Helper/internal methods for geometry and filtering
6. Public workflow method (for example run/generate path)
```

### Function/Method Organization
```text
- Parse/validate inputs from config or caller
- Perform core math/optimization logic
- Normalize/correct outputs (for example endpoint corrections)
- Return domain objects or serializable structures
- Log or print progress for long-running generation loops
```

### File Organization Principles
```text
- One primary responsibility per file (entrypoint, solver, orchestrator, data model, helper)
- Related functionality grouped together in generator modules
- Domain objects separated from solver-specific implementations
- Output serialization centralized in generation entrypoint workflow
```

## Code Organization Principles

1. **Single Responsibility**: Keep orchestration, model solving, and serialization concerns separated.
2. **Modularity**: Add new motion model logic in dedicated model files and wire via generator dispatch.
3. **Testability**: Keep pure helper/math functions isolated for future unit tests.
4. **Consistency**: Follow existing snake_case module/function patterns and established output schema.

## Module Boundaries

- **Entry Layer vs Core Logic**: `generate_optimal_primitives.py` handles CLI and file I/O, while generation internals remain in `optimal_generator.py` and `trajectory_generator.py`.
- **Orchestration vs Model-Specific Solvers**: `trajectory_generator.py` dispatches to `bicycle_acc.py`, `unicycle_acc.py`, and `omni_acc.py`.
- **Domain Data vs Algorithms**: `trajectory.py` defines trajectory/path representations used by generator and solver logic.
- **Utilities vs Business Logic**: `helper.py` provides shared math/angle utilities without owning workflow control.
- **Stable vs Evolving Artifacts**: Source modules are stable code; `outputs/` and generated JSON/figures are runtime artifacts and should not drive architecture decisions.
- **Dependency Direction**: Entry layer -> orchestrators -> model/domain/utilities. Model modules should not depend on entrypoint code.

## Code Size Guidelines

- **File size**: Target under 600 lines for most modules; split responsibilities when growth exceeds this.
- **Function/Method size**: Target under 80 lines per function, except for numerically dense routines where clarity is preserved.
- **Class/Module complexity**: Prefer fewer than 10 public methods per class/module role.
- **Nesting depth**: Keep nesting to 3 levels or fewer where practical; extract helper methods for complex branching.

## Dashboard/Monitoring Structure (if applicable)

This project does not include an internal runtime dashboard subsystem. Monitoring and inspection are file-based:
- JSON outputs in `outputs/<model>_primitives/`
- Static plots in `outputs/<model>_primitives/figures/`
- Optional analysis through `analyze_primitives.py`

## Documentation Standards
- Public-facing workflow and configuration must be documented in `README.md`.
- New configuration keys and output fields must be documented when introduced.
- Complex math/geometry blocks should include concise inline comments where intent is not obvious.
- Steering/spec docs under `.spec-workflow/` should be kept synchronized with implementation changes.
