const fs=require('fs');const p='UI/src/OnlineLocalRegistrationWorker.cpp';
let s=fs.readFileSync(p,'utf8').replace(/^\uFEFF/,'').replace(/\r\n/g,'\n');
let st=s.indexOf('        surfaceRequest.candidateVirtualToReal = result.correction;');
let en=s.indexOf('\n    } catch (const std::exception &error)',st);
if(st<0||en<0)throw Error('surface block missing');
let b=s.slice(st,en);
b=b.replace('        result.originalSurfaceCorrection = result.correction;\n','');
let a=b.indexOf('        // 缺失观测允许表面支路工作');
let e=b.indexOf('        result.baseSafetyAccepted = result.accepted;',a);
if(a<0||e<0)throw Error('surface veto missing');
b=b.slice(0,a)+`        // 相同原图焊点逐帧核对；当前几何不确定时保留旧点门禁，候选无交点不能回退。
        std::vector<COnlineLocalRegistrationStability::Observation> current;
        result.accepted = evaluateIndependentWindow(frames, result, current) && result.accepted;
`+b.slice(e);
a=b.indexOf('            std::vector<COnlineLocalRegistrationStability::Observation> current;');
e=b.indexOf('            if (!current.empty())',a);
if(a<0||e<0)throw Error('surface stability missing');
b=b.slice(0,a)+b.slice(e);
// This loop applies only after the training solver; no heldout data changes the solve or information subspace.
let repl=`        result.originalSurfaceCorrection = result.correction;
        const auto fullCorrection = result.correction;
        // 仅沿训练云得到的共同可观测修正取三个有限步长；不重新拟合留出帧或焊点。
        // 每个候选重新走全部门禁，不能靠焊点改善豁免表面失败。
        for (double stepScale : {1.0, 0.5, 0.25}) {
            if (solveTimer.elapsed() > m_options.maximumSolveTimeMs) {
                result.accepted = false;
                result.message = QStringLiteral("候选评价超过统一耗时预算，拒绝更新。");
                break;
            }
            result.candidateStepScale = stepScale;
            result.correction = fullCorrection;
            if (stepScale != 1.0) {
                const auto &full = fullCorrection.rowMajor;
                const double yaw = std::atan2(full[4], full[0]) * stepScale;
                auto &scaled = result.correction.rowMajor;
                scaled[0] = scaled[5] = std::cos(yaw);
                scaled[1] = -std::sin(yaw);
                scaled[4] = std::sin(yaw);
                scaled[3] *= stepScale;
                scaled[7] *= stepScale;
                scaled[11] *= stepScale;
            }
            result.stabilityEvaluated = false;
            result.stabilityEvaluation = {};
`+b.split('\n').map(l=>'    '+l).join('\n')+`
            result.solveElapsedMs = solveTimer.elapsed();
            if (result.solveElapsedMs > m_options.maximumSolveTimeMs) {
                result.accepted = false;
                result.message = QStringLiteral("候选完整评价超过统一耗时预算，拒绝更新。");
                break;
            }
            if (result.accepted) { break; }
        }`;
s=s.slice(0,st)+repl+s.slice(en);
fs.writeFileSync(p,'\uFEFF'+s.replace(/\n/g,'\r\n'),'utf8');console.log('bounded candidate validation integrated');