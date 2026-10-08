"""检查实际 MSVC/Ninja 产物的头文件依赖和入口/构造函数布局，不启动设备。"""

import argparse
import ctypes as ct
import os
from pathlib import Path
import re
import struct
import subprocess


class Symbol(ct.Structure):
    _fields_ = [
        ("SizeOfStruct", ct.c_ulong), ("TypeIndex", ct.c_ulong),
        ("Reserved", ct.c_ulonglong * 2), ("Index", ct.c_ulong),
        ("Size", ct.c_ulong), ("ModBase", ct.c_ulonglong),
        ("Flags", ct.c_ulong), ("Value", ct.c_ulonglong),
        ("Address", ct.c_ulonglong), ("Register", ct.c_ulong),
        ("Scope", ct.c_ulong), ("Tag", ct.c_ulong),
        ("NameLen", ct.c_ulong), ("MaxNameLen", ct.c_ulong),
        ("Name", ct.c_char * 2048),
    ]


class StackFrame(ct.Structure):
    _fields_ = [
        ("InstructionOffset", ct.c_ulonglong), ("ReturnOffset", ct.c_ulonglong),
        ("FrameOffset", ct.c_ulonglong), ("StackOffset", ct.c_ulonglong),
        ("BackingStoreOffset", ct.c_ulonglong), ("FuncTableEntry", ct.c_ulonglong),
        ("Params", ct.c_ulonglong * 4), ("Reserved", ct.c_ulonglong * 5),
        ("Virtual", ct.c_int),
    ]


def check_dependencies(build, probe):
    cache = (build / "CMakeCache.txt").read_text(encoding="utf-8")
    ninja = re.search(r"^CMAKE_MAKE_PROGRAM:[^=]+=(.+)$", cache, re.M)[1]
    source = Path(re.search(r"^CMAKE_HOME_DIRECTORY:[^=]+=(.+)$", cache, re.M)[1])
    target = "UI/CMakeFiles/i2r_rlc.dir/src/main.cpp.obj"
    output = subprocess.check_output(
        [ninja, "-t", "deps", target], cwd=build, text=True, encoding="utf-8"
    )
    normalized = output.replace("\\", "/").lower()
    headers = ["UI/include/UiController.h", "UI/include/RuntimeCoordinator.h",
               "UI/include/RuntimeTrackingTypes.h"]
    missing = [header for header in headers if header.lower() not in normalized]
    assert not missing, "main.cpp.obj misses header dependencies: " + ", ".join(missing)
    if probe:
        # 只临时改变 mtime 并 dry-run，finally 恢复；要求本构建目录没有并发生成。
        def dry_run():
            return subprocess.check_output(
                [ninja, "-n", target], cwd=build, text=True, encoding="utf-8"
            ).replace("\\", "/")

        expected = "Building CXX object " + target
        assert expected not in dry_run(), "build main.cpp before probing header invalidation"
        header = source / headers[-1]
        stamp = header.stat()
        try:
            future = max(stamp.st_mtime_ns, (build / target).stat().st_mtime_ns) + 2_000_000_000
            os.utime(header, ns=(stamp.st_atime_ns, future))
            assert expected in dry_run(), "changing an indirect header did not schedule main.cpp"
        finally:
            os.utime(header, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        assert expected not in dry_run(), "restored header unexpectedly dirties main.cpp"
    print("PASS T1: actual main.cpp tracks direct/indirect layout headers"
          + (" and header-only change triggers recompilation" if probe else ""))


def check_layout(build):
    """只查当前函数作用域中的活动类型；PDB 可保留无关的历史类型记录。"""
    exe = build / "UI/i2r_rlc.exe"
    image = exe.read_bytes()
    pe = struct.unpack_from("<I", image, 0x3C)[0]
    image_size = struct.unpack_from("<I", image, pe + 24 + 56)[0]
    dbg = ct.WinDLL("dbghelp.dll", use_last_error=True)
    kernel = ct.WinDLL("kernel32.dll")
    kernel.GetCurrentProcess.restype = ct.c_void_p
    process = kernel.GetCurrentProcess()
    base = 0x10000000
    callback_type = ct.WINFUNCTYPE(ct.c_int, ct.POINTER(Symbol), ct.c_ulong, ct.c_void_p)
    dbg.SymInitialize.argtypes = [ct.c_void_p, ct.c_char_p, ct.c_int]
    dbg.SymCleanup.argtypes = [ct.c_void_p]
    dbg.SymLoadModuleEx.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_char_p,
        ct.c_char_p, ct.c_ulonglong, ct.c_ulong, ct.c_void_p, ct.c_ulong]
    dbg.SymLoadModuleEx.restype = ct.c_ulonglong
    dbg.SymFromName.argtypes = [ct.c_void_p, ct.c_char_p, ct.POINTER(Symbol)]
    dbg.SymSetContext.argtypes = [ct.c_void_p, ct.POINTER(StackFrame), ct.c_void_p]
    dbg.SymEnumSymbols.argtypes = [ct.c_void_p, ct.c_ulonglong, ct.c_char_p,
                                  callback_type, ct.c_void_p]
    dbg.SymGetTypeInfo.argtypes = [ct.c_void_p, ct.c_ulonglong, ct.c_ulong,
                                 ct.c_int, ct.c_void_p]
    dbg.SymSetOptions(0x216)  # undecorate + deferred symbols + lines + no error dialogs
    assert dbg.SymInitialize(process, os.fsencode(exe.parent), 0), "SymInitialize failed"
    try:
        assert dbg.SymLoadModuleEx(process, None, os.fsencode(exe), None,
                                   base, image_size, None, 0), "load EXE/PDB failed"

        def local_size(function_name, local_name, pointer):
            function = Symbol()
            function.SizeOfStruct = Symbol.Name.offset + 4  # sizeof SYMBOL_INFO = 88 on Win64
            function.MaxNameLen = 2048
            assert dbg.SymFromName(process, function_name.encode(), ct.byref(function)), function_name
            frame = StackFrame()
            frame.InstructionOffset = function.Address + min(32, max(0, function.Size - 1))
            assert dbg.SymSetContext(process, ct.byref(frame), None), "set function scope failed"
            found = []

            @callback_type
            def collect(info, unused_size, unused_context):
                value = info.contents
                if value.Name.decode() == local_name:
                    type_id = value.TypeIndex
                    if pointer:
                        pointee = ct.c_ulong()
                        if not dbg.SymGetTypeInfo(process, base, type_id, 4, ct.byref(pointee)):
                            return 0
                        type_id = pointee.value
                    length = ct.c_ulonglong()
                    if dbg.SymGetTypeInfo(process, base, type_id, 2, ct.byref(length)):
                        found.append(length.value)
                return 1

            assert dbg.SymEnumSymbols(process, 0, None, collect, None), "enumerate active locals failed"
            assert len(found) == 1 and found[0] > 0, "missing unique active local: " + local_name
            return found[0]

        allocated = local_size("main", "controller", False)
        constructed = local_size("i2r_rlc::ui::UiController::UiController", "this", True)
        assert allocated == constructed, (
            f"UiController layout mismatch: main allocates {allocated}, constructor uses {constructed} bytes"
        )
        print(f"PASS T2: main and UiController constructor agree on {allocated} bytes")
    finally:
        dbg.SymCleanup(process)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    parser.add_argument("--probe-header", action="store_true",
                        help="dry-run an indirect header mtime change, restoring it afterwards")
    args = parser.parse_args()
    failures = 0
    for name, test in [("T1", lambda: check_dependencies(args.build.resolve(), args.probe_header)),
                       ("T2", lambda: check_layout(args.build.resolve()))]:
        try:
            test()
        except (AssertionError, OSError, subprocess.SubprocessError) as error:
            failures += 1
            print(f"FAIL {name}: {error}")
    raise SystemExit(bool(failures))
