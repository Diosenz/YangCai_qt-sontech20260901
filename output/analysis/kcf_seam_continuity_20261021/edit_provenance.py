from pathlib import Path
root=Path('.')
def edit(name, pairs):
 p=root/name
 s=p.read_text(encoding='utf-8-sig')
 for a,b in pairs:
  assert a in s,(name,a[:100])
  s=s.replace(a,b)
 p.write_bytes(('\ufeff'+s.replace('\r\n','\n').replace('\n','\r\n')).encode('utf-8'))
edit('UI/src/OnlineCorrectionRunRecorder.cpp', [
 ('"response\tmessage\n"))','"response\tmessage\testimated\tpoint_source\tdegraded_frames\n"))'),
 ('"gap_width_mm\tseam_index\n"))','"gap_width_mm\tseam_index\testimated\n"))'),
 ('degraded_frames\ttransition_limited\n','degraded_frames\ttransition_limited\testimated\n'),
 ('QStringLiteral("icp_diagnostics_schema"), QStringLiteral("1")','QStringLiteral("icp_diagnostics_schema"), QStringLiteral("2")'),
 ('sanitizeField(result.message)\n    };','sanitizeField(result.message), boolText(result.sample.estimated),\n        QString::number(static_cast<int>(result.pointSource)), QString::number(result.degradedFrames)\n    };'),
 ('<< boolText(algorithm.continuity.transitionLimited);','<< boolText(algorithm.continuity.transitionLimited) << boolText(result.sample.estimated);'),
 ('const std::optional<float> &gap, const QString &seam) {','const std::optional<float> &gap, const QString &seam, const QString &estimated) {'),
 ('boolText(point.valid), optionalNumberText(gap), seam','boolText(point.valid), optionalNumberText(gap), seam, estimated'),
 ('batch.fitPoints[index], {}, {});','batch.fitPoints[index], {}, {}, {});'),
 ('batch.modelPoints[index], {}, {});','batch.modelPoints[index], {}, {}, {});'),
 ('sample.gapWidthMm, QString::number(sample.simulatedSeamIndex));','sample.gapWidthMm, QString::number(sample.simulatedSeamIndex), boolText(sample.estimated));'),
 ("    return appendData(fileName, fields.join('\t').toUtf8() + '\n');","    // 非视觉事件没有样本来源，保留空列以与追加后的表头对齐。\n    if (fileName == QString::fromLatin1(kRealtimeCorrectionFileName) && fields.size() == 23) {\n        auto row = fields;\n        row << QString() << QString() << QString();\n        return appendData(fileName, row.join('\t').toUtf8() + '\n');\n    }\n    return appendData(fileName, fields.join('\t').toUtf8() + '\n');")
])
