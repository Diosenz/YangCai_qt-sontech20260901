from pathlib import Path
import re

p = Path('tests/test_online_local_surface_verification.cpp')
s = p.read_text(encoding='utf-8-sig')
s = s.replace('#include <cmath>', '#include <algorithm>\n#include <cmath>', 1)
marker = 'bool testNewFrameStreakAndTerminalState()'
helper = '''// 旧断言保留双向绝对门禁；配对证据按显式差值构造，不用于生产计算。
Decision observe(Verification &verification, qint64 frameId, bool valid, const Metrics &metrics)
{
    const auto &old = verification.baselineMetrics();
    const double worsening = std::max(metrics.realToVirtualResidualMm - old.realToVirtualResidualMm,
        metrics.virtualToRealResidualMm - old.virtualToRealResidualMm);
    return verification.observe(frameId, valid, metrics,
        {true, 0.0, std::max(0.0, worsening), 1.0});
}

'''
# 已有断言专注帧去重、终态、覆盖和配置，新行为另设直接配对证据反例。
s = re.sub(r'(\w+)\.observe\(', r'observe(\1, ', s)
s = s.replace(marker, helper + marker, 1)
new_test = '''bool testMovingSliceAndSameFrameCounterfactual()
{
    const Metrics frozen{1.3061662919, 0.4767739274, 0.9449, 1.0};
    Verification moving(frozen, Verification::Options{});
    const Metrics fresh[] = {
        {1.4962744289, 0.4475627589, 0.8928, 1.0},
        {1.9867825967, 0.6365605719, 0.8384, 0.9534},
        {1.7706495806, 0.6137708139, 0.8482, 0.9534}
    };
    // 现场切片指标只复现旧误拒触发；配对指标为合成，不能声称现场位姿正确。
    for (int index = 0; index < 3; ++index) {
        moving.observe(2980 + index, true, fresh[index], {true, 0.8, 0.6, 0.9});
    }
    bool ok = expect(moving.decision() == Decision::Verified,
        "new slice statistics may change while same-frame paired surfaces improve");
    Verification wrong(frozen, Verification::Options{});
    for (int index = 0; index < 3; ++index) {
        wrong.observe(10 + index, true, fresh[index], {true, 0.4, 0.6, 0.9});
    }
    ok &= expect(wrong.decision() == Decision::Rollback,
        "genuine same-frame worsening still rolls back despite absolute slice limits passing");
    Verification unsupported(frozen, Verification::Options{});
    for (int index = 0; index < 3; ++index) {
        unsupported.observe(20 + index, true, fresh[index], {false, 0.8, 0.6, 0.9});
    }
    ok &= expect(unsupported.decision() == Decision::Rollback,
        "no frozen local support cannot be accepted by absolute slice metrics alone");
    return ok;
}

'''
s = s.replace('} // namespace', new_test + '} // namespace', 1)
s = s.replace('ok &= testInvalidEvidenceAndConfiguration();',
    'ok &= testInvalidEvidenceAndConfiguration();\n    ok &= testMovingSliceAndSameFrameCounterfactual();', 1)
p.write_text(s, encoding='utf-8-sig', newline='\r\n')
