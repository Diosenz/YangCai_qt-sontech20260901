# Feature Specification: [FEATURE NAME]

**Feature Branch**: `[feature-branch]`
**Created**: [DATE]
**Status**: Draft
**Input**: User description: "$ARGUMENTS"

## Engineering Objective *(mandatory)*

**Change Point**: [The behavior, algorithm, dependency, or strategy being refactored or optimized]
**Owning Module/Area**: [Primary module and developer responsibility area]
**Shared Impact**: [Public types, UI, RobotComm, CMake, tests, docs, config/data, or None]
**Behavior to Preserve**: [Observable behavior that MUST remain unchanged, or N/A with rationale]
**Intentional Behavior Changes**: [Explicitly approved changes, or None]
**Out of Scope**: [Adjacent modules, another developer's ownership area, or speculative work excluded]

## Scenarios & Acceptance *(mandatory)*

### Scenario 1 - [Primary engineering outcome] (Priority: P1)

[Describe the coherent behavior or engineering capability]

**Why this priority**: [Value and risk]
**Independent Verification**: [Shared test, replay, comparison, or inspection that proves the outcome]

**Acceptance Scenarios**:

1. **Given** [baseline/input], **When** [change is exercised], **Then** [measurable result]
2. **Given** [failure/boundary input], **When** [change is exercised], **Then** [explicit outcome]

### Edge Cases

- [Boundary, invalid input, missing dependency, or skipped-data behavior]
- [State, timing, precision, coordinate/physical-unit, threading, or compatibility condition]

## Requirements *(mandatory)*

### Functional and Contract Requirements

- **FR-001**: The system MUST [specific capability or preserved behavior].
- **FR-002**: Inputs and outputs MUST define [units, validity, error, ownership, and state semantics].
- **FR-003**: Cross-module changes MUST [state coordination and compatibility requirements].
- **FR-004**: Experimental selection MUST [remain isolated from the default path, if applicable].

### Evaluation Requirements *(mandatory for algorithm comparison or optimization)*

- **ER-001 Baseline**: [Existing implementation/version and baseline capture method]
- **ER-002 Dataset**: [Representative data/sample scope and provenance]
- **ER-003 Common Path**: [Shared preprocessing, parameters, contract, and `tests/`/replay harness]
- **ER-004 Metrics**: [Correctness/quality metrics; performance/resource metrics when relevant]
- **ER-005 Thresholds**: [Acceptance, equivalence, improvement, and regression thresholds]
- **ER-006 Reproducibility**: [Build configuration, dependencies, parameters, seed/environment]

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: [Measurable behavior or quality outcome]
- **SC-002**: [No-regression threshold against the baseline]
- **SC-003**: [Performance outcome, or N/A with rationale]
- **SC-004**: [Integration outcome for shared contracts/areas, or N/A]

## Assumptions and Dependencies

- [Reasonable assumption]
- [Required dataset, hardware, SDK, model, service, configuration, or collaborator decision]
