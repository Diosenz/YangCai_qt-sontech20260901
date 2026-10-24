const fs=require('fs');
function read(p){return fs.readFileSync(p,'utf8').replace(/^\uFEFF/,'').replace(/\r\n/g,'\n');}
function write(p,s){fs.writeFileSync(p,'\uFEFF'+s.replace(/\r?\n/g,'\r\n'),'utf8');}
function edit(p,changes){let s=read(p);for(const[a,b]of changes){if(!s.includes(a))throw Error(p+' missing '+a.slice(0,80));s=s.replace(a,b);}write(p,s);}
edit('UI/include/OnlineLocalRegistrationWorker.h',[[`        std::optional<digital_twin::OnlineSeamGeometryRequest> candidateGeometry;
    };`,`        std::optional<digital_twin::OnlineSeamGeometryRequest> candidateGeometry;
        std::optional<digital_twin::OnlineSeamSceneBinding> candidateGeometryBinding;
    };`]]);
edit('UI/src/OnlineLocalRegistrationWorker.cpp',[[`    reset();
}

void COnlineLocalRegistrationWorker::reset()`,`    m_candidateGeometry.clear();
    m_candidateGeometryBindingKey.clear();
    m_candidateGeometryLoadMessage.clear();
    reset();
}

void COnlineLocalRegistrationWorker::reset()`],[`    validation.evidenceContext = surface.evidenceContext;`,`    validation.evidenceContext = surface.evidenceContext;
    if (!m_options.candidateGeometrySceneFile.isEmpty()) {
        validation.candidateGeometryBinding = digital_twin::OnlineSeamSceneBinding{
            m_options.candidateGeometrySceneFile, input.workpieceId, input.parentId, input.sensorId};
    }
    // 与表面投影使用同一次曝光的机器人状态/标定，禁止读取处理时的最新姿态。
    const auto built = vision::CWeldProjectionContextBuilder::build(
        input.trackingOptions.sensorCalibration, input.trackingOptions.positionerCalibration, input.robotState);
    if (input.confirmedWorkpiecePose.has_value() && built.valid) {
        const auto &context = built.context;
        const auto &parent = context.robotBaseToPositionerTcp;
        const auto &tool = context.toolInRobotBase;
        const auto &camera = context.calibration.cameraInTool;
        const Eigen::Matrix3d rotation = parent.rotation * tool.rotation * camera.rotation;
        const Eigen::Vector3d translation = parent.translationMm + parent.rotation
            * (tool.translationMm + tool.rotation * camera.translationMm);
        const Eigen::Vector3d normal = rotation * Eigen::Vector3d(
            context.calibration.laserPlaneA, context.calibration.laserPlaneB, context.calibration.laserPlaneC);
        const double norm = normal.norm();
        if (rotation.allFinite() && translation.allFinite() && normal.allFinite() && norm > 1e-12) {
            digital_twin::OnlineSeamGeometryRequest snapshot;
            snapshot.parentLaserPlane = {normal.x() / norm, normal.y() / norm, normal.z() / norm,
                (1000.0 - normal.dot(translation)) / norm};
            snapshot.parentFromWorkpiece = *input.confirmedWorkpiecePose;
            snapshot.referenceToleranceMm = m_options.candidateGeometryReferenceToleranceMm;
            validation.candidateGeometry = snapshot;
        }
    }`],[`    const bool independentUsable = usableIndependent(validation.independentObservation, frameId);`,`    if (validation.candidateGeometryBinding.has_value()) {
        const auto &binding = *validation.candidateGeometryBinding;
        const QString key = QStringLiteral("%1|%2|%3|%4").arg(binding.sceneFile)
            .arg(binding.workpieceId).arg(binding.parentId).arg(binding.sensorId);
        if (key != m_candidateGeometryBindingKey) {
            const auto loaded = m_candidateGeometry.load(binding);
            m_candidateGeometryBindingKey = key;
            m_candidateGeometryLoadMessage = loaded.message;
        }
    }
    const bool independentUsable = usableIndependent(validation.independentObservation, frameId);`],[`    frame.evidenceContext = validation.evidenceContext;`,`    frame.evidenceContext = validation.evidenceContext;
    frame.candidateGeometry = validation.candidateGeometry;`],[`        {QStringLiteral("schema_version"), 4},`,`        {QStringLiteral("schema_version"), 4},
        {QStringLiteral("candidate_geometry_policy_version"), 1},
        {QStringLiteral("candidate_geometry"), result.candidateGeometryEvidence},
        {QStringLiteral("candidate_step_scale"), result.candidateStepScale},
        {QStringLiteral("candidate_geometry_scene_file"), m_options.candidateGeometrySceneFile},
        {QStringLiteral("candidate_geometry_load_message"), m_candidateGeometryLoadMessage},`]]);
let p='UI/src/OnlineLocalRegistrationWorker.cpp',s=read(p);
const at=s.indexOf('bool COnlineLocalRegistrationWorker::archiveAttempt(');
if(at<0)throw Error('archive marker missing');
s=s.slice(0,at)+`bool COnlineLocalRegistrationWorker::evaluateIndependentWindow(const std::deque<CloudFrame> &frames,
    Result &result, std::vector<COnlineLocalRegistrationStability::Observation> &current)
{
    bool accepted = true;
    result.beforeIndependentErrorMm = 0.0;
    result.afterIndependentErrorMm = 0.0;
    for (const auto &frame : frames) {
        const auto &independent = frame.independentObservation;
        if (!usableIndependent(independent, frame.frameId)) { continue; }
        const auto moved = localization::pipeline_math::transformPoints(
            {independent.virtualWeldPointInParent_mm}, result.correction);
        auto beforePoint = independent.virtualWeldPointInParent_mm;
        auto afterPoint = moved.front();
        digital_twin::OnlineSeamGeometryResult geometry;
        if (frame.candidateGeometry.has_value()) {
            auto request = *frame.candidateGeometry;
            request.correction = result.correction;
            request.currentVirtualWeld = independent.virtualWeldPointInParent_mm;
            request.realWeld = independent.realWeldPointInParent_mm;
            geometry = m_candidateGeometry.evaluate(request);
        }
        if (geometry.valid) {
            beforePoint = geometry.beforeIntersection;
            afterPoint = geometry.afterIntersection;
        }
        const double before = pointDistance(independent.realWeldPointInParent_mm, beforePoint);
        const double after = pointDistance(independent.realWeldPointInParent_mm, afterPoint);
        const bool candidateValid = !(geometry.referenceVerified && !geometry.valid);
        const bool frameAccepted = candidateValid && std::isfinite(before) && std::isfinite(after)
            && after <= m_options.maximumIndependentErrorMm
            && after <= before + m_options.maximumIndependentWorseningMm;
        result.beforeIndependentErrorMm = std::max(result.beforeIndependentErrorMm, before);
        result.afterIndependentErrorMm = std::max(result.afterIndependentErrorMm, after);
        auto observation = COnlineLocalRegistrationStability::Observation{
            independent.realWeldPointInParent_mm, beforePoint, frame.frameId};
        if (geometry.valid) { observation.candidateVirtualPoint = afterPoint; }
        current.push_back(observation);
        QJsonObject record{
            {QStringLiteral("frame_id"), QString::number(frame.frameId)},
            {QStringLiteral("candidate_step_scale"), result.candidateStepScale},
            {QStringLiteral("reference_verified"), geometry.referenceVerified},
            {QStringLiteral("geometry_verified"), geometry.valid},
            {QStringLiteral("status"), static_cast<int>(geometry.status)},
            {QStringLiteral("seam_edge_id"), geometry.seamEdgeId},
            {QStringLiteral("evaluation_kind"), geometry.valid ? QStringLiteral("fixed_exposure_plane_cad_reintersection")
                : (geometry.referenceVerified ? QStringLiteral("candidate_geometry_rejected")
                    : QStringLiteral("legacy_point_veto_fallback"))},
            {QStringLiteral("before_intersection_parent_mm"), pointJson(beforePoint)},
            {QStringLiteral("after_intersection_parent_mm"), pointJson(afterPoint)},
            {QStringLiteral("before_error_mm"), before}, {QStringLiteral("after_error_mm"), after},
            {QStringLiteral("accepted"), frameAccepted}, {QStringLiteral("message"), geometry.message}};
        if (frame.candidateGeometry.has_value()) {
            const auto &snapshot = *frame.candidateGeometry;
            const auto &plane = snapshot.parentLaserPlane;
            QJsonArray pose;
            for (double value : snapshot.parentFromWorkpiece.rowMajor) { pose.append(value); }
            record.insert(QStringLiteral("parent_from_workpiece_row_major"), pose);
            record.insert(QStringLiteral("parent_laser_plane"), QJsonArray{plane.nx, plane.ny, plane.nz, plane.dMm});
        }
        result.candidateGeometryEvidence.append(record);
        if (!frameAccepted) {
            accepted = false;
            result.message = QStringLiteral("独立焊点候选验收拒绝：frame=%1，前=%2 mm，后=%3 mm，几何状态=%4。")
                .arg(frame.frameId).arg(before).arg(after).arg(static_cast<int>(geometry.status));
        }
    }
    return accepted;
}

`+s.slice(at);
// Replace duplicated historical point veto with one evidence-aware routine.
let start=s.indexOf(`        if (validation.requireIndependentObservation) {
            // KCF`);
let end=s.indexOf(`
    } catch (const std::exception &error)`,start);
if(start<0||end<0)throw Error('legacy veto block missing');
s=s.slice(0,start)+`        if (validation.requireIndependentObservation) {
            std::vector<COnlineLocalRegistrationStability::Observation> current;
            result.accepted = evaluateIndependentWindow(attemptedFrames, result, current) && result.accepted;
        }`+s.slice(end);
// Current stability observations must use the same checked CAD result instead of re-vetoing transformed old intersections.
start=s.indexOf(`        for (const auto &item : attemptedFrames) {
            const auto &observation`);
end=s.indexOf('        result.stabilityEvaluated',start);
s=s.slice(0,start)+`        const bool independentAccepted = evaluateIndependentWindow(attemptedFrames, result, current);
        result.accepted = independentAccepted && result.accepted;
`+s.slice(end);
// Surface window block is replaced in next edit with bounded candidate search.
write(p,s);
console.log('worker snapshot and CAD evidence channel integrated');