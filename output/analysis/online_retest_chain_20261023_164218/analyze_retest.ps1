param([string]$RepositoryRoot = 'H:\YangCai_Code\YangCai_qt-sontech20260901')
$ErrorActionPreference = 'Stop'
$runRelative = 'output/online_correction/20261023_164218_138'
$localRelative = 'output/online_local_registration/20261023_084218_096_session_1'
$evidenceDirectory = Join-Path $RepositoryRoot 'output/analysis/online_retest_chain_20261023_164218'
$rows = @(Import-Csv (Join-Path $RepositoryRoot "$runRelative/vision_diagnostics.tsv") -Delimiter "`t")
$online = @($rows | Where-Object phase -eq 'online_running')
$robotRows = @(Import-Csv (Join-Path $RepositoryRoot "$runRelative/actual_robot_path.txt") -Delimiter "`t")
$timed = @(foreach ($row in $rows) {
    [pscustomobject]@{
        frame = [long]$row.frame_id; phase = $row.phase; source = $row.point_source
        expanded = $row.image_search_expanded; reinitialized = $row.template_reinitialized
        process_ms = [long]$row.processing_finished_ms - [long]$row.processing_started_ms
        geometry_ms = [double]$row.geometry_elapsed_ms
        frozen_state_age_at_finish_ms = [long]$row.processing_finished_ms - [long]$row.robot_state_received_ms
        finish_to_record_ms = [long]$row.record_timestamp_ms - [long]$row.processing_finished_ms
    }
})
$groups = @(foreach ($group in $timed | Group-Object expanded) {
    $sorted = @($group.Group | Sort-Object process_ms)
    [pscustomobject]@{
        expanded = $group.Name; count = $sorted.Count
        process_mean_ms = ($sorted | Measure-Object process_ms -Average).Average
        process_p50_ms = $sorted[[int][math]::Floor(($sorted.Count - 1) * 0.5)].process_ms
        process_p95_ms = $sorted[[int][math]::Floor(($sorted.Count - 1) * 0.95)].process_ms
        process_max_ms = $sorted[-1].process_ms
        geometry_mean_ms = ($sorted | Measure-Object geometry_ms -Average).Average
    }
})
$trigger = $rows | Where-Object frame_id -eq '9243' | Select-Object -First 1
$cancelTimeMs = 1792745207076L
$latestRecordedRobotState = $robotRows | Where-Object {[long]$_.timestamp_ms -le $cancelTimeMs} | Select-Object -Last 1
$after = @($online | Where-Object {[long]$_.record_timestamp_ms -ge $cancelTimeMs})
$attempts = @(foreach ($file in Get-ChildItem (Join-Path $RepositoryRoot $localRelative) -File | Where-Object {$_.Name -match '^local_icp_1_\d+_[^.]+\.json$'}) {
    $candidate = Get-Content -LiteralPath $file.FullName -Raw | ConvertFrom-Json
    [pscustomobject]@{file=$file.Name;frame=$candidate.frame_id;accepted=$candidate.accepted_before_archive;surface_only=$candidate.surface_only}
})
$selectedIds = @('3246','3259','7075','9030','9241','9243','9258','10691','15179','16873')
$selected = @($rows | Where-Object frame_id -in $selectedIds | Select-Object frame_id,phase,result_valid,point_source,camera_timestamp_ms,robot_state_received_ms,processing_started_ms,processing_finished_ms,record_timestamp_ms,geometry_elapsed_ms,image_search_expanded,template_reinitialized,workpiece_pose_revision,before_fusion_x_px,before_fusion_y_px,final_x_px,final_y_px,sim_x_px,sim_y_px)
$productionPaths = @('Vision/src/track/LegacyOnlineKcfDetector.cpp','Vision/src/feature/OnlineSeamRefiner.cpp','UI/src/RuntimeLocalRegistration.cpp','UI/src/RuntimeCoordinator.cpp','UI/src/RuntimeWorkerAdapters.cpp','UI/src/UiController.cpp','UI/src/MainWidget.cpp','Control/src/OnlineCorrectionController.cpp','Control/src/OnlineCorrectionPathFollower.cpp')
$sourceHashes = @(foreach ($path in $productionPaths) {[pscustomobject]@{path=$path;sha256=(Get-FileHash (Join-Path $RepositoryRoot $path) -Algorithm SHA256).Hash}})
$summary = [ordered]@{
    scope='Read-only diagnosis; no production edits, builds, device operations or Computer Use.'
    run=$runRelative; local_registration=$localRelative; all_results=$rows.Count; online_results=$online.Count
    online_sources=@($online | Group-Object point_source | Select-Object Name,Count)
    timing_all_phases=$groups
    expanded_frames=@($timed | Where-Object expanded -eq '1').Count
    local_frames_over_100ms=@($timed | Where-Object {$_.expanded -eq '0' -and $_.process_ms -ge 100}).Count
    trigger_frame=$trigger.frame_id
    trigger_frozen_state_age_at_capture_ms=[long]$trigger.camera_timestamp_ms-[long]$trigger.robot_state_received_ms
    trigger_frozen_state_age_at_finish_ms=[long]$trigger.processing_finished_ms-[long]$trigger.robot_state_received_ms
    latest_recorded_robot_state_before_cancel=$latestRecordedRobotState
    latest_recorded_robot_state_age_ms=$cancelTimeMs-[long]$latestRecordedRobotState.timestamp_ms
    cancellation_timestamp_ms=$cancelTimeMs
    after_cancellation_results=$after.Count
    after_cancellation_sources=@($after | Group-Object point_source | Select-Object Name,Count)
    candidate_count=$attempts.Count; accepted_candidates=@($attempts | Where-Object accepted).Count
    writeback_count=@(Get-ChildItem (Join-Path $RepositoryRoot $localRelative) -Filter '*.writeback.json').Count
    verified_count=@(Get-ChildItem (Join-Path $RepositoryRoot $localRelative) -Filter '*.verified.json').Count
    selected_frames=$selected; candidates=$attempts; current_source_hashes=$sourceHashes
    confirmed_boundaries=@('Full expanded image search runs synchronously before KCF fallback identity acceptance.','Frozen exposure RobotState age is checked against processing wall time; >500ms resets pending verification and sets the session fault latch.','Fresh later RobotState does not clear that latch.','UI marker visibility is overwritten with current result.valid, while coordinate panel retains the previous accepted final business point.')
    evidence_gaps=@('No per-proposal/local-search failure timing or local rejection status distinct from final expanded result.','No UI receive/paint duration timestamps.','No persisted verification samples/decision for cancelled9030 transaction.','Five-second timeout callback delivery and retainRequested value are not recorded.','Not every camera frame has a saved raw image; the optical cause of each local rejection cannot be reconstructed.')
    confirmed_display_requirement='User selected: always show final business point only; do not add raw KCF response display.'
}
[IO.File]::WriteAllText((Join-Path $evidenceDirectory 'evidence.json'),($summary | ConvertTo-Json -Depth 9),[Text.UTF8Encoding]::new($false))
$selected | Export-Csv (Join-Path $evidenceDirectory 'selected_frames.tsv') -Delimiter "`t" -NoTypeInformation -Encoding utf8
$applicationLog='C:/Users/admin/AppData/Local/i2r_rlc/logs/i2r_rlc.log'
$logLines=Get-Content -LiteralPath $applicationLog
$excerpt=for($i=0;$i -lt $logLines.Count;$i++) {if($logLines[$i] -match '^\[2026-10-23T16:46:(41|46|47)\.') { '{0}: {1}' -f ($i+1),$logLines[$i] }}
[IO.File]::WriteAllText((Join-Path $evidenceDirectory 'application_log_excerpt.txt'),($excerpt -join "`r`n"),[Text.UTF8Encoding]::new($false))
[pscustomobject]@{all=$rows.Count;online=$online.Count;after_cancel=$after.Count;trigger_frame=$trigger.frame_id;frozen_age_ms=$summary.trigger_frozen_state_age_at_finish_ms;live_state_age_ms=$summary.latest_recorded_robot_state_age_ms;evidence=$evidenceDirectory} | ConvertTo-Json
