if(NOT VCPKG_TARGET_IS_WINDOWS OR NOT VCPKG_TARGET_ARCHITECTURE STREQUAL "x64")
    message(FATAL_ERROR "${PORT} overlay currently supports only Windows x64.")
endif()

if(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
    message(FATAL_ERROR "${PORT} overlay installs the official dynamic Windows x64 package only.")
endif()

set(ONNXRUNTIME_VERSION "1.23.2")
set(ONNXRUNTIME_ARCHIVE_NAME "onnxruntime-win-x64-${ONNXRUNTIME_VERSION}.zip")

vcpkg_download_distfile(ONNXRUNTIME_ARCHIVE
    URLS "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_ARCHIVE_NAME}"
    FILENAME "${ONNXRUNTIME_ARCHIVE_NAME}"
    SHA512 3f3aecea57a38fca987401306e72cde4058828a41ee57e16cc9ff3363eaaba16c43ae65545a280f0fd485da535d6e18541838cf5ccd12c13374f8f0a66704d0d
)

vcpkg_extract_source_archive(
    SOURCE_PATH
    ARCHIVE "${ONNXRUNTIME_ARCHIVE}"
)

set(PACKAGE_DIR "${SOURCE_PATH}")
if(NOT EXISTS "${PACKAGE_DIR}/include/onnxruntime_c_api.h")
    set(PACKAGE_DIR "${SOURCE_PATH}/onnxruntime-win-x64-${ONNXRUNTIME_VERSION}")
endif()

if(NOT EXISTS "${PACKAGE_DIR}/include/onnxruntime_c_api.h")
    message(FATAL_ERROR "Unexpected ONNX Runtime package layout: ${PACKAGE_DIR}")
endif()

file(INSTALL "${PACKAGE_DIR}/include/" DESTINATION "${CURRENT_PACKAGES_DIR}/include")

file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime.lib" DESTINATION "${CURRENT_PACKAGES_DIR}/lib")
file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime.dll" DESTINATION "${CURRENT_PACKAGES_DIR}/bin")
file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime_providers_shared.lib" DESTINATION "${CURRENT_PACKAGES_DIR}/lib")
file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime_providers_shared.dll" DESTINATION "${CURRENT_PACKAGES_DIR}/bin")

file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime.lib" DESTINATION "${CURRENT_PACKAGES_DIR}/debug/lib")
file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime.dll" DESTINATION "${CURRENT_PACKAGES_DIR}/debug/bin")
file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime_providers_shared.lib" DESTINATION "${CURRENT_PACKAGES_DIR}/debug/lib")
file(INSTALL "${PACKAGE_DIR}/lib/onnxruntime_providers_shared.dll" DESTINATION "${CURRENT_PACKAGES_DIR}/debug/bin")

file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/share/${PORT}")
file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/onnxruntimeConfig.cmake" [=[
include_guard(GLOBAL)

add_library(onnxruntime::onnxruntime SHARED IMPORTED)

get_filename_component(_ONNXRUNTIME_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set_target_properties(onnxruntime::onnxruntime PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_ONNXRUNTIME_IMPORT_PREFIX}/include"
    IMPORTED_CONFIGURATIONS "Debug;Release;RelWithDebInfo;MinSizeRel"
    IMPORTED_IMPLIB_DEBUG "${_ONNXRUNTIME_IMPORT_PREFIX}/debug/lib/onnxruntime.lib"
    IMPORTED_LOCATION_DEBUG "${_ONNXRUNTIME_IMPORT_PREFIX}/debug/bin/onnxruntime.dll"
    IMPORTED_IMPLIB_RELEASE "${_ONNXRUNTIME_IMPORT_PREFIX}/lib/onnxruntime.lib"
    IMPORTED_LOCATION_RELEASE "${_ONNXRUNTIME_IMPORT_PREFIX}/bin/onnxruntime.dll"
    IMPORTED_IMPLIB_RELWITHDEBINFO "${_ONNXRUNTIME_IMPORT_PREFIX}/lib/onnxruntime.lib"
    IMPORTED_LOCATION_RELWITHDEBINFO "${_ONNXRUNTIME_IMPORT_PREFIX}/bin/onnxruntime.dll"
    IMPORTED_IMPLIB_MINSIZEREL "${_ONNXRUNTIME_IMPORT_PREFIX}/lib/onnxruntime.lib"
    IMPORTED_LOCATION_MINSIZEREL "${_ONNXRUNTIME_IMPORT_PREFIX}/bin/onnxruntime.dll"
)

unset(_ONNXRUNTIME_IMPORT_PREFIX)
]=])

file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/onnxruntimeConfigVersion.cmake" [=[
set(PACKAGE_VERSION "1.23.2")

if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
        set(PACKAGE_VERSION_EXACT TRUE)
    endif()
endif()
]=])

vcpkg_install_copyright(FILE_LIST
    "${PACKAGE_DIR}/LICENSE"
    "${PACKAGE_DIR}/ThirdPartyNotices.txt"
)
