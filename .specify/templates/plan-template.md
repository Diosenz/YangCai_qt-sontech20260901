# Implementation Plan: [FEATURE]

**Branch**: `[feature-branch]` | **Date**: [DATE] | **Spec**: [link]

**Input**: Feature specification from `/specs/[feature-name]/spec.md`

## Summary

[Primary requirement, baseline behavior, and proposed technical approach]

## Technical Context

**Language/Version**: C++17 or [NEEDS CLARIFICATION]
**Primary Dependencies**: CMake, Qt, and feature-specific dependencies
**Testing/Evaluation**: `tests/` CTest targets, contract/integration tests, offline replay, and shared metrics
**Target Platform**: Windows, Visual Studio 2022 / MSVC x64 or [NEEDS CLARIFICATION]
**Performance Goals**: [Measurable target or N/A]
**Constraints**: [Behavioral, hardware, latency, accuracy, threading, or compatibility constraints]
**Scale/Scope**: [Affected subsystems, datasets, runtime paths, and shared files]

## Ownership and Integration Impact

**Owning Module/Developer Area**: [Vision/Control, Localization/IBVS/DigitalTwin, or shared area]
**Shared Areas Touched**: [UI, RobotComm, Camera, CMake, tests, docs, config/data, or None]
**Coordination Required**: [Public contract/state/protocol/config/thread/CMake decisions, or None]
**Integration Target**: [Branch and required pre-merge validation]

## Change and Evaluation Design

**Variation Point**: [What implementation, algorithm, or dependency is expected to vary]
**Stable Contract**: [Inputs, outputs, parameters, state, units, validity, and failure semantics]
**Baseline**: [Existing implementation and captured behavior]
**Candidate**: [Proposed implementation or refactor]
**Shared Evaluation**: [Dataset/sample set, preprocessing, metrics, thresholds, and tests/replay harness]
**Experiment Isolation**: [Build option, configuration, or composition boundary; N/A if not experimental]

## Constitution Check

*GATE: Must pass before research. Re-check after design.*

- [ ] The boundary corresponds to an evidenced variation point; stable code is not abstracted speculatively.
- [ ] The owning module and any cross-owner/shared-area impact are explicit and coordinated.
- [ ] Baseline and candidate use the same contract, data, preprocessing, metrics, and thresholds.
- [ ] Refactor-preserved behavior and intentional behavior changes are separately identified.
- [ ] Experimental code cannot silently change the default production path and is reproducible.
- [ ] Added architectural elements have a concrete benefit and a simpler alternative was considered.
- [ ] Verification uses the shared `tests/`/CTest/replay path and matches behavioral risk.

## Project Structure

```text
specs/[feature]/
├── spec.md
├── plan.md
├── research.md
├── quickstart.md
└── tasks.md
```

**Affected Repository Paths**: [Use actual paths under Camera/, Localization/, DigitalTwin/, Vision/,
RobotComm/, Control/, IBVS/, UI/, tests/, docs/, data/, or config/]

**Structure Decision**: [Why the existing owning subsystem is correct; justify any new boundary]

## Complexity Tracking

> Fill only for Constitution violations or temporary exceptions requiring maintainer approval.

| Violation / Exception | Why Needed | Simpler Alternative Rejected Because | Approval / Removal Condition |
|-----------------------|------------|--------------------------------------|------------------------------|
| [item] | [current need] | [evidence] | [owner approval and expiry/migration condition] |
