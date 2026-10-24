# Recovery evidence and transition (ticket 07)

The same public `replay.py --evidence` entry applies recovery after loss.
Candidate configuration (not field calibrated):

- `recovery_confirmation_frames`: positive fresh-exposure count, default 2.
- `max_reacquisition_distance_pixel`: identity corridor around the historical relative prediction; default null rejects displaced automatic recovery.
- `max_recovery_step_pixel`: maximum estimated transition step; default null rejects automatic nonzero jumps from an available previous estimate.

A duplicate or missing/rejected exposure breaks confirmation. Out-of-corridor geometry is not adopted even when the production image branch accepted it.
Confirmed raw geometry is retained in `recovery_measurement_pixel`, separately from `final_pixel`.
Transition output has source `recovery_transition`, no independent validation or spatial/control qualification; it cannot refresh the anchor or old budget.
The old time/distance limits also apply to transitions. When that budget expires, the transition stops. Fresh confirmed independent geometry may subsequently reestablish a measured anchor after the unavailable interval; this is not prediction renewing itself.
A distant target requires an explicit new session/reestablishment decision; this implementation does not silently reset identity or choose a far branch.

Tests: `python3 tests/diagnostics/online_detector_replay/test_recovery.py`.
Four public image-replay scenarios cover bounded transition, duplicate confirmation, budget expiration and distant candidates.
The raw production detector still runs with its original training semantics until the separate ticket 06 transaction is enabled. This ticket does not claim to roll back private tracker training.
