# Product Overview

## Product Purpose
This project generates high-quality motion primitives using optimal control for grid-based robot navigation. It solves the problem of producing kinematically feasible, planner-compatible primitive sets that can be reused across motion models (differential drive, Ackermann, and omni-directional robots) while preserving trajectory quality and controllability constraints.

## Target Users
Primary users are robotics software engineers and autonomy developers who build or tune local/global planners (especially lattice-based planners such as Nav2 Smac).

Their key needs and pain points are:
- Need planner-ready primitive datasets that match robot kinematic limits.
- Need predictable output schema and metadata for integration pipelines.
- Need ability to tune generation settings (turning radius, heading bins, velocity/acceleration limits) without rewriting generator logic.
- Need consistent quality across different motion models and environments.

## Key Features

1. **Optimal-Control Primitive Generation**: Produces trajectory primitives with time-based cost under configurable kinematic constraints.
2. **Multi-Model Support**: Supports differential, Ackermann, and omni-style workflows through dedicated model logic and configuration.
3. **Planner-Compatible Output**: Exports JSON primitive sets with metadata and trajectory samples consumable by downstream lattice planners such as SMAC planner from Nav2.
4. **Config-Driven Workflow**: Uses JSON config files to define generation constraints and behavior.
5. **Analysis and Visualization Outputs**: Saves generated primitives and figures for validation and tuning.

## Success Metrics

- **Generation Success Rate**: At least 95% of configured runs complete successfully without runtime errors.
- **Integration Readiness**: 100% of generated primitive files validate against the expected planner schema and metadata contract.
- **Tuning Efficiency**: Engineers can produce an updated primitive set from config changes in under 10 minutes for standard scenarios.
- **Model Coverage**: Primitive outputs available and verified for all supported motion model families used in the project.

## Product Principles

1. **Physical Feasibility First**: Generated primitives must honor kinematic and dynamic constraints before optimization convenience.
2. **Deterministic and Reproducible Outputs**: Identical inputs should produce consistent primitive sets and metadata.
3. **Integration-Oriented Design**: Output contracts and metadata must be explicit, stable, and easy to consume by planning stacks.
4. **Configurable Without Code Changes**: Common tuning should be achieved via config parameters, not source modifications.

## Monitoring & Visibility (if applicable)

- **Dashboard Type**: File-based workflow with generated JSON outputs and figure artifacts.
- **Real-time Updates**: Not required for runtime operation; generation is batch-oriented.
- **Key Metrics Displayed**: Number of trajectories, generation completion status, metadata consistency, and model-specific primitive characteristics.
- **Sharing Capabilities**: Primitive JSON and generated figures can be shared through repository artifacts or output folders.

