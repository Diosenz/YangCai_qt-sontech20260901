/*******************************************************************************
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        test_offline_image_camera_device.cpp
 * @brief       offline image camera device 测试入口
 * @details     验证对应模块的迁移行为和关键边界，测试断言保持原有语义。
 ******************************************************************************/
#include <iostream>

#include <QColor>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QImage>

#include "OfflineImageCameraDevice.h"

namespace {

using i2r_rlc::camera::OfflineImageCameraDevice;

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }

    return true;
}

bool writeImage(const QString& path, const QColor& color)
{
    QImage image(8, 6, QImage::Format_RGB32);
    image.fill(color);
    if (!image.save(path)) {
        std::cerr << "FAIL: cannot write image " << path.toStdString() << '\n';
        return false;
    }

    return true;
}

QString makeTestRoot(const QString& caseName)
{
    QDir root(QDir::current().filePath(QStringLiteral("Testing/Temporary/i2r_rlc_offline_camera_tests/%1").arg(caseName)));
    if (root.exists()) {
        root.removeRecursively();
    }
    if (!QDir().mkpath(root.absolutePath())) {
        std::cerr << "FAIL: cannot create test root " << root.absolutePath().toStdString() << '\n';
        return {};
    }
    return root.absolutePath();
}

bool testInvalidDirectoryFails()
{
    OfflineImageCameraDevice device;
    QString errorMessage;
    const bool ok = device.setImageDirectory(QStringLiteral("Z:/this/path/should/not/exist"), &errorMessage);
    return expectTrue(!ok, "invalid image directory should fail")
        && expectTrue(errorMessage.contains(QStringLiteral("不存在")), "invalid directory should report structured error");
}

bool testEmptyDirectoryFails()
{
    const QString tempPath = makeTestRoot(QStringLiteral("empty_directory"));
    if (!expectTrue(!tempPath.isEmpty(), "temporary dir should be valid")) {
        return false;
    }

    OfflineImageCameraDevice device;
    QString errorMessage;
    const bool ok = device.setImageDirectory(tempPath, &errorMessage);
    return expectTrue(!ok, "empty image directory should fail")
        && expectTrue(errorMessage.contains(QStringLiteral("没有支持的图片")), "empty directory should report no supported images");
}

bool testSortedFrameReadAndReset()
{
    const QString tempPath = makeTestRoot(QStringLiteral("sorted_frame_read"));
    if (!expectTrue(!tempPath.isEmpty(), "temporary dir should be valid")) {
        return false;
    }

    QDir dir(tempPath);
    bool ok = true;
    ok &= writeImage(dir.filePath(QStringLiteral("002_second.png")), QColor(0, 255, 0));
    ok &= writeImage(dir.filePath(QStringLiteral("001_first.bmp")), QColor(255, 0, 0));
    if (!ok) {
        return false;
    }

    OfflineImageCameraDevice device;
    QString errorMessage;
    ok &= expectTrue(device.setImageDirectory(tempPath, &errorMessage), "valid image directory should load");
    ok &= expectTrue(device.imageCount() == 2, "two images should be discovered");
    ok &= expectTrue(device.hasNextFrame(), "device should have first frame");

    const auto first = device.nextFrame();
    const auto second = device.nextFrame();
    ok &= expectTrue(first.ok, "first frame should read");
    ok &= expectTrue(second.ok, "second frame should read");
    ok &= expectTrue(QFileInfo(first.frame.sourcePath).fileName() == QStringLiteral("001_first.bmp"), "first sorted file should be 001_first.bmp");
    ok &= expectTrue(QFileInfo(second.frame.sourcePath).fileName() == QStringLiteral("002_second.png"), "second sorted file should be 002_second.png");
    ok &= expectTrue(!first.frame.image.isNull(), "first frame image should not be null");
    ok &= expectTrue(first.frame.frameId == 0, "first frameId should be 0");
    ok &= expectTrue(second.frame.frameId == 1, "second frameId should be 1");

    device.reset();
    const auto afterReset = device.nextFrame();
    ok &= expectTrue(afterReset.ok, "frame after reset should read");
    ok &= expectTrue(afterReset.frame.frameId == 0, "frameId should restart after reset");
    ok &= expectTrue(QFileInfo(afterReset.frame.sourcePath).fileName() == QStringLiteral("001_first.bmp"), "reset should return to first sorted file");
    return ok;
}

} // namespace

/**
 * @brief 离线图像相机轻量级测试入口。
 *
 * 该测试只验证目录扫描、排序和 QImage 读取，不依赖 Qt Widgets，不访问旧工程目录。
 */
int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    bool ok = true;
    ok &= testInvalidDirectoryFails();
    ok &= testEmptyDirectoryFails();
    ok &= testSortedFrameReadAndReset();

    if (!ok) {
        return 1;
    }

    std::cout << "PASS: offline image camera device tests\n";
    return 0;
}
