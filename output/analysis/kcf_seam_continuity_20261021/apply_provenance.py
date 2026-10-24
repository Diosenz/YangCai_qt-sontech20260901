from pathlib import Path
b=chr(92)
def edit(name,pairs):
 p=Path(name);s=p.read_text(encoding='utf-8-sig')
 for a,c in pairs:
  a=a.replace('@T',b+'t').replace('@N',b+'n');c=c.replace('@T',b+'t').replace('@N',b+'n')
  assert a in s,(name,a[:100]);s=s.replace(a,c)
 p.write_bytes((chr(65279)+s.replace(chr(13)+chr(10),chr(10)).replace(chr(10),chr(13)+chr(10))).encode('utf-8'))
edit('UI/src/OnlineCorrectionRunRecorder.cpp', [
 ('"response@Tmessage@N"))','"response@Tmessage@Testimated@Tpoint_source@Tdegraded_frames@N"))'),
 ('"gap_width_mm@Tseam_index@N"))','"gap_width_mm@Tseam_index@Testimated@N"))'),
 ('degraded_frames@Ttransition_limited@N','degraded_frames@Ttransition_limited@Testimated@N'),
 ('QStringLiteral("icp_diagnostics_schema"), QStringLiteral("1")','QStringLiteral("icp_diagnostics_schema"), QStringLiteral("2")'),
 ('sanitizeField(result.message)\n    };','sanitizeField(result.message), boolText(result.sample.estimated),\n        QString::number(static_cast<int>(result.pointSource)), QString::number(result.degradedFrames)\n    };'),
 ('<< boolText(algorithm.continuity.transitionLimited);','<< boolText(algorithm.continuity.transitionLimited) << boolText(result.sample.estimated);'),
 ('const std::optional<float> &gap, const QString &seam) {','const std::optional<float> &gap, const QString &seam, const QString &estimated) {'),
 ('boolText(point.valid), optionalNumberText(gap), seam','boolText(point.valid), optionalNumberText(gap), seam, estimated'),
 ('batch.fitPoints[index], {}, {});','batch.fitPoints[index], {}, {}, {});'),
 ('batch.modelPoints[index], {}, {});','batch.modelPoints[index], {}, {}, {});'),
 ('sample.gapWidthMm, QString::number(sample.simulatedSeamIndex));','sample.gapWidthMm, QString::number(sample.simulatedSeamIndex), boolText(sample.estimated));'),
 ("    return appendData(fileName, fields.join('@T').toUtf8() + '@N');","""    // Non-vision events have no sample provenance; keep the appended columns empty.
    if (fileName == QString::fromLatin1(kRealtimeCorrectionFileName) && fields.size() == 23) {
        auto row = fields;
        row << QString() << QString() << QString();
        return appendData(fileName, row.join('@T').toUtf8() + '@N');
    }
    return appendData(fileName, fields.join('@T').toUtf8() + '@N');""")
])
edit('tests/test_runtime_tracking_flow.cpp', [('&& first.independentWeldPointInParent_mm.valid,','&& first.independentWeldPointInParent_mm.valid && !first.sample.estimated,'),('&& predicted.pointSource == vision::OnlineSeamPointSource::SimulationPrediction','&& predicted.sample.estimated\n                && predicted.pointSource == vision::OnlineSeamPointSource::SimulationPrediction')])
edit('tests/test_online_seam_continuity_replay.cpp', [('    std::ifstream in(root / "output/online_correction/20261021_224020_914/vision_diagnostics.tsv");','''    fs::path recording;
    const auto recordings = root / "output/online_correction";
    if (fs::exists(recordings)) {
        for (const auto &entry : fs::directory_iterator(recordings)) {
            if (entry.is_directory()
                && entry.path().filename().wstring().find(L"20261021_224020_914") == 0) {
                if (!recording.empty()) { std::cerr << "Ambiguous incident recording@N"; return 1; }
                recording = entry.path() / "vision_diagnostics.tsv";
            }
        }
    }
    std::ifstream in(recording);''')])
edit('tests/test_online_correction_run_recorder.cpp', [
 ('fallback.releasedSamples = fallback.pendingSamples;', 'fallback.pendingSamples[0].estimated = true;\n    fallback.releasedSamples = fallback.pendingSamples;'),
 ('    if (targets.size() == 2) {','''    if (points.size() == 9) {
        ok &= expect(points[0].split('@T').last() == "estimated"
            && points[7].split('@T').last() == "1"
            && points[8].split('@T').last() == "1",
            "ICP pending and released samples retain estimated provenance");
    }
    if (targets.size() == 2) {'''),
 ('    tracking.frameId = 44;\n    tracking.diagnostic = diagnosticSnapshot;', '''    tracking.frameId = 44;
    tracking.diagnostic = diagnosticSnapshot;
    tracking.valid = true;
    tracking.sample.point.valid = true;
    tracking.sample.estimated = true;
    tracking.pointSource = vision::OnlineSeamPointSource::SimulationPrediction;
    tracking.degradedFrames = 360;
    tracking.diagnostic->algorithm.refinement.valid = false;'''),
 ('        ok &= expect(value(capped, "image_request_status") == "pair_limit",','''        ok &= expect(value(capped, "estimated") == "1"
            && value(capped, "point_source") == "3"
            && value(capped, "degraded_frames") == "360"
            && value(capped, "final_y_px") == "205.000000"
            && value(capped, "geometry_y_px").isEmpty(),
            "prediction retains business coordinates without fabricating measured geometry");
        ok &= expect(value(capped, "image_request_status") == "pair_limit",'''),
 ('    const auto fresh = readRows(recorder.sessionDirectory(), "online_icp_batches.txt");','''    const auto sourceRows = readRows(recorder.sessionDirectory(), "realtime_correction.txt");
    const auto sourceHeader = sourceRows.front().split('@T');
    const auto sourceRow = sourceRows.back().split('@T');
    const int estimatedIndex = sourceHeader.indexOf("estimated");
    const int sourceIndex = sourceHeader.indexOf("point_source");
    ok &= expect(sourceRow.size() == sourceHeader.size()
        && estimatedIndex >= 0 && sourceIndex >= 0
        && sourceRow[estimatedIndex] == "1" && sourceRow[sourceIndex] == "3",
        "business log preserves provenance even with diagnostics disabled");
    const auto fresh = readRows(recorder.sessionDirectory(), "online_icp_batches.txt");''')])
print('Applied provenance serialization, assertions, recording discovery')
