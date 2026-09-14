---
description: "Outcome-oriented task template for collaborative refactors and algorithm experiments"
---

# Tasks: [FEATURE NAME]

**Input**: Design documents from `/specs/[feature-name]/`

**Organization**: Tasks are grouped by meaningful behavior, capability, dependency, ownership,
or risk boundary. Do not create one task per file, class, interface, DTO, or test file.

## Format: `[ID] [P?] [Outcome] Description`

- **[P]**: Can run in parallel without shared-file, ownership, or dependency conflicts
- **[Outcome]**: Maps the task to a coherent specification outcome
- Include exact repository paths and the evidence that completes the task

## Phase 1: Baseline, Ownership, and Evaluation Foundation

- [ ] T001 Identify the owning module and record cross-owner/shared-area coordination needs
- [ ] T002 Capture preserved behavior and known deviations using existing `tests/` or replay evidence
- [ ] T003 Define the shared dataset, preprocessing, metrics, thresholds, and result format
- [ ] T004 Record build configuration, dependencies, parameters, and environment needed for reproduction

**Checkpoint**: Ownership, baseline, and comparison method are reviewable before implementation.

## Phase 2: Stable Boundary *(include only when justified)*

- [ ] T005 [Outcome] Establish or adapt the smallest stable input/output/parameter/state contract
- [ ] T006 [Outcome] Keep UI, hardware, file loading, transport, and implementation details outside the core contract
- [ ] T007 [Outcome] Confirm shared contract changes with affected maintainers and document compatibility

**Checkpoint**: Baseline behavior still passes through the selected boundary.

## Phase 3: Candidate Refactor or Algorithm

- [ ] T008 [Outcome] Implement the candidate within the owning existing subsystem
- [ ] T009 [Outcome] Connect selection only at the configuration, composition, or pipeline boundary
- [ ] T010 [Outcome] Preserve compatibility mappings or explicitly implement approved behavior changes

**Checkpoint**: Candidate and baseline can be selected and evaluated without unrelated changes.

## Phase 4: Comparative Verification

- [ ] T011 [P] [Outcome] Run shared correctness/quality evaluation for baseline and candidate
- [ ] T012 [P] [Outcome] Run performance/resource evaluation when optimization is a stated goal
- [ ] T013 [Outcome] Validate failure, skip, boundary, threading, and cross-subsystem behavior by risk
- [ ] T014 [Outcome] Record comparable results and resolve regressions against acceptance thresholds

**Checkpoint**: Evidence distinguishes pass, failure, and missing-data/dependency skips.

## Phase 5: Integration and Handoff

- [ ] T015 [Outcome] Confirm experimental code cannot silently alter the default production path
- [ ] T016 [Outcome] Update focused docs/spec artifacts and record shared-area/API impacts
- [ ] T017 [Outcome] Run relevant Visual Studio build and shared CTest/replay targets
- [ ] T018 [Outcome] Sync the integration target, resolve semantic conflicts, and record validation/risks before push

## Dependencies and Parallel Work

- Ownership and shared-impact decisions precede changes in another maintainer's area.
- Baseline and evaluation definitions precede candidate comparison claims.
- A new contract, when justified, precedes implementations that consume it.
- Candidate code and independent evaluation-fixture preparation may run in parallel when paths do not conflict.
- Tasks touching public types, UI, RobotComm, CMake, config, state machines, or threading are not marked [P]
  unless coordination and file ownership are explicitly resolved.

## Notes

- Prefer existing `tests/` CTest and replay tools when they provide sufficient evidence.
- Add validation for uncovered required behavior, equivalence, regression risk, or high-risk paths.
- A task MAY modify multiple related files when they jointly deliver one meaningful outcome.
- Do not add abstractions, cross-owner edits, or adjacent cleanup without a current documented need.
