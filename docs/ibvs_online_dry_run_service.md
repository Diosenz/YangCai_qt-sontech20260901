# IBVS Online Dry-run Service

## Summary

`IbvsOnlineService` turns the existing IBVS pure algorithms into a start/stop
service boundary for UI and later runtime orchestration. The current online path
is dry-run only: it computes fused desired features, unified velocity and a
robot command draft, but it does not enqueue RobotComm messages.

## Boundaries

- IBVS service inputs are plain value objects, not QWidget, Camera SDK or
  RobotComm objects.
- UI may create simulated `IbvsOnlineFrameInput` for dry-run diagnostics.
- A future camera adapter should convert real image detections into the same
  `IbvsOnlineFrameInput` shape before calling IBVS.
- A future robot adapter may consume `dryRunCommand` or `velocityMps`, but real
  sending must remain behind an explicit safety gate.

## Current Flow

1. UI loads `config/IBVS/ibvs_runtime_defaults.txt` and
   `config/IBVS/workpiece_profiles.txt` through the service.
2. UI starts IBVS dry-run.
3. UI single-step creates a simulated frame input from the latest frame id.
4. `IbvsOnlineService` runs sensor fusion and unified velocity solving.
5. UI displays velocity, feature error, row count, limit status and command
   draft.

## Deferred Work

- Real camera continuous acquisition and frame queue integration.
- Real RobotComm command enqueueing.
- Robot pose based position-constraint current value injection.
- Safety interlocks, operator confirmation, emergency stop and hardware mode
  validation.
