import pathlib, subprocess,shlex,sys
root=pathlib.Path.cwd();out=root/'output/analysis/kcf_production_integration_20261025'
header=(root/'UI/include/RuntimeWorkerAdapters.h').read_text(encoding='utf-8-sig')
begin=header.index('class CVisionTrackingWorkerAdapter ');end=header.index('\n};',begin)+3
partial=out/'VisionWorkerMoc.h';partial.write_text('#include "RuntimeWorkerAdapters.h"\nnamespace i2r_rlc::ui {\n'+header[begin:end]+'\n}\n')
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','Qt6Core','Qt6Gui','eigen3'],text=True))
flags+=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs-only-L','opencv4'],text=True))+['-lopencv_imgproc','-lopencv_imgcodecs','-lopencv_core']
includes=[arg for m in ['UI','Camera','Vision','Control','Localization','DigitalTwin','RobotComm','IBVS'] for arg in ['-I',str(root/(m+'/include'))]]
moc=out/'moc_VisionWorker.cpp'
run=subprocess.run(['/opt/homebrew/share/qt/libexec/moc','-i',str(partial),'-o',str(moc)],capture_output=True,text=True)
if run.returncode: print(run.stderr);sys.exit(run.returncode)
moc.write_text('#include "RuntimeWorkerAdapters.h"\n'+moc.read_text())
sources=['tests/test_online_kcf_evidence_worker.cpp','UI/src/RuntimeWorkerAdapters.cpp','Control/src/TrackingTriggerPolicy.cpp','Vision/src/geometry/WeldProjectionContextBuilder.cpp','Vision/src/geometry/PositionerKinematics.cpp','Vision/src/stripe/StegerStripeDetector.cpp','Vision/src/stripe/StripeLineRepairTools.cpp','Vision/src/stripe/StripeResponseEvaluationTools.cpp','Vision/src/geometry/LineGeometryTools.cpp']
sys.path.insert(0,str(root/'tests/diagnostics/online_detector_replay'));from replay import SOURCES
sources+=['Vision/src/'+name+'.cpp' for name in SOURCES]
sources+=['Vision/third_party/libicp/matrix.cpp','tests/diagnostics/online_detector_replay/GapDisabledAdapter.cpp',str(moc)]
command=['clang++','-std=c++17','-O1','-ffunction-sections','-fdata-sections','-DI2R_RLC_HAS_OPENCV',*sources,*includes,'-I',str(root/'Vision/third_party'),*flags,'-Wl,-dead_strip','-o',str(out/'worker_tests')]
run=subprocess.run(command,capture_output=True,text=True);(out/'worker_build.log').write_text(run.stdout+run.stderr)
print('worker build',run.returncode,flush=True)
if run.returncode:print(run.stderr[-5000:]);sys.exit(run.returncode)
run=subprocess.run([str(out/'worker_tests')],capture_output=True,text=True);(out/'worker_tests.log').write_text(run.stdout+run.stderr);print(run.stdout+run.stderr);sys.exit(run.returncode)
