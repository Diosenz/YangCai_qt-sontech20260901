/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_runtime_live_camera_flow.cpp
 * @brief       runtime live camera flow 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>
#include <memory>

#include <QCoreApplication>
#include <QEventLoop>
#include <QImage>
#include <QTimer>

#include "RuntimeWorkerAdapters.h"

namespace {

class CFakeStreamingCamera final : public i2r_rlc::camera::ICameraDevice {
public:
    void start() override
    {
        m_started = true;
    }

    void stop() override
    {
        m_started = false;
    }

    i2r_rlc::camera::CameraFrame latestFrame() const override
    {
        i2r_rlc::camera::CameraFrame frame;
        if (!m_started) {
            return frame;
        }
        frame.cameraId = i2r_rlc::camera::CameraId::VimbaTracking;
        frame.frameId = m_frameId;
        frame.frameNumber = m_frameId++;
        frame.image = QImage(64, 48, QImage::Format_Grayscale8);
        frame.image.fill(80);
        frame.previewImage = frame.image;
        frame.description = QStringLiteral("fake live camera frame");
        return frame;
    }

    QString stateText() const override
    {
        return m_started
            ? QStringLiteral("fake camera running")
            : QStringLiteral("fake camera stopped");
    }

    bool setExposureTimeUs(double exposureTimeUs, QString*) override
    {
        m_lastExposureUs = exposureTimeUs;
        ++m_exposureSetCount;
        return m_started;
    }

    bool getExposureTimeUs(double* exposureTimeUs, QString*) const override
    {
        if (exposureTimeUs == nullptr) {
            return false;
        }
        *exposureTimeUs = m_lastExposureUs;
        return true;
    }

    int exposureSetCount() const
    {
        return m_exposureSetCount;
    }

    double lastExposureUs() const
    {
        return m_lastExposureUs;
    }

private:
    mutable qint64 m_frameId = 0;
    mutable bool m_started = false;
    double m_lastExposureUs = 0.0;
    int m_exposureSetCount = 0;
};

bool expect(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    auto camera = std::make_unique<CFakeStreamingCamera>();
    CFakeStreamingCamera* cameraRecord = camera.get();
    i2r_rlc::ui::CCameraWorkerAdapter worker(std::move(camera), 5);

    bool received = false;
    i2r_rlc::camera::CameraFrame lastFrame;
    QObject::connect(
        &worker,
        &i2r_rlc::ui::CCameraWorkerAdapter::frameReady,
        &app,
        [&](const i2r_rlc::camera::CameraFrame &frame) {
            received = true;
            lastFrame = frame;
            app.quit();
        });

    worker.start();
    worker.setExposureTimeUs(4321);
    QTimer::singleShot(1000, &app, &QCoreApplication::quit);
    app.exec();
    worker.stop();

    if (!expect(received, "camera worker should publish live frames")
        || !expect(!lastFrame.image.isNull(), "published frame should contain image")
        || !expect(
            lastFrame.cameraId == i2r_rlc::camera::CameraId::VimbaTracking,
            "camera role should be preserved")
        || !expect(cameraRecord->exposureSetCount() == 1,
                   "camera worker should forward exposure requests")
        || !expect(cameraRecord->lastExposureUs() == 4321.0,
                   "camera worker should preserve exposure value in us")) {
        return 1;
    }

    std::cout << "PASS: runtime live camera flow tests\n";
    return 0;
}
