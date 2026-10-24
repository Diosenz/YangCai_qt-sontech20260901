// Isolated headless rendering of our local report; no mouse or desktop-app control.
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const { pathToFileURL } = require('url');
const { chromium } = require('C:/Users/admin/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');

(async () => {
    const browser = await chromium.launch({
        executablePath: 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
        headless: true,
        args: ['--allow-file-access-from-files']
    });
    try {
        const page = await browser.newPage({ viewport: { width: 1440, height: 1080 }, deviceScaleFactor: 1 });
        const errors = [];
        page.on('pageerror', error => errors.push(error.message));
        await page.goto(pathToFileURL(path.join(__dirname, 'report.html')).href);
        await page.waitForFunction(() => window.reportReady === true);
        const result = await page.evaluate(async () => {
            const selector = document.getElementById('frame-select');
            const ids = [...selector.options].map(option => option.value);
            const pristine = window.reportDataSnapshot();
            let validGeometryFrames = 0;
            for (const id of ids) {
                selector.value = id;
                selector.dispatchEvent(new Event('change'));
                const image = document.getElementById('source-image');
                await new Promise((resolve, reject) => {
                    const check = new Image();
                    check.onload = () => check.naturalWidth > 0 ? resolve() : reject(Error('empty image'));
                    check.onerror = () => reject(Error('source image failed'));
                    check.src = image.getAttribute('href');
                });
                const metadata = document.getElementById('frame-meta').textContent;
                assertMetadata(metadata.includes('帧 ' + id), 'frame label mismatch');
                const expectedGeometry = metadata.includes('几何有效：是');
                const displayedGeometry = document.querySelector('#overlays [data-layer="geometry"]') !== null;
                assertMetadata(expectedGeometry === displayedGeometry, 'invalid geometry presented as measurement');
                validGeometryFrames += expectedGeometry ? 1 : 0;
            }
            for (const input of document.querySelectorAll('[data-layer]')) {
                input.checked = false;
                input.dispatchEvent(new Event('change'));
            }
            assertMetadata(document.getElementById('overlays').children.length === 0, 'overlay hide failed');
            for (const input of document.querySelectorAll('[data-layer]')) {
                input.checked = true;
                input.dispatchEvent(new Event('change'));
            }
            selector.value = '17861';
            selector.dispatchEvent(new Event('change'));
            assertMetadata(document.getElementById('frame-meta').textContent.includes('仿真辅助预测'), 'prediction source mislabelled');
            assertMetadata(pristine === window.reportDataSnapshot(), 'coordinates changed while toggling');
            assertMetadata(pristine === window.reportPristine, 'initial snapshot mismatch');
            return {
                frameIds: ids,
                validGeometryFrames,
                frameSwitch: 'passed',
                imageLoads: 'passed',
                overlaySwitches: 'passed',
                coordinateImmutability: 'passed',
                desktopOverflow: document.documentElement.scrollWidth > innerWidth,
                links: [...document.querySelectorAll('a[href]')].map(link => link.getAttribute('href'))
            };
            function assertMetadata(value, message) { if (!value) throw Error(message); }
        });
        assert.strictEqual(result.desktopOverflow, false);
        assert.deepStrictEqual(errors, []);
        for (const href of result.links) {
            assert(!/^(https?:|javascript:)/i.test(href), 'unexpected external dependency');
            assert(fs.existsSync(path.join(__dirname, href)), 'missing local link: ' + href);
        }
        await page.screenshot({ path: path.join(__dirname, 'report_preview.png'), fullPage: false });
        await page.locator('section').nth(1).screenshot({ path: path.join(__dirname, 'frame_preview.png') });
        await page.locator('section').nth(2).screenshot({ path: path.join(__dirname, 'counterexample_preview.png') });
        const report = { status: 'passed', browser: 'isolated Edge headless', mouseUsed: false,
            robotOrSceneAccess: false, controlsVerifiedBy: 'DOM change events', errors, ...result };
        fs.writeFileSync(path.join(__dirname, 'browser_verification.json'), JSON.stringify(report, null, 2));
        process.stdout.write(JSON.stringify(report));
    } finally {
        await browser.close();
    }
})().catch(error => { process.stderr.write(error.stack); process.exitCode = 1; });
