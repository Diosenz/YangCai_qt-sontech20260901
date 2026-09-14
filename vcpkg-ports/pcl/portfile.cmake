vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO PointCloudLibrary/pcl
    REF pcl-1.8.1
    SHA512 9e7c87fb750a176712f08d215a906012c9e8174b687bbc8c08fa65de083b4468951bd8017b10409015d5eff0fc343885d2aae5c340346118b1a251af7bdd5cd7
    HEAD_REF master
    PATCHES
        cmakelists.patch
        config.patch
        config_install.patch
        find_flann.patch
        boost_endian_compat.patch
        boost_filesystem_compat.patch
        disable_device_grabbers.patch
        disable_legacy_frustum_culling.patch
        disable_legacy_bilateral_upsampling.patch
        msvc_log2f_compat.patch
        boost_uuid_random_generator_compat.patch
        cmake4_policy_compat.patch
        boost_checked_delete_noexcept.patch
        cpp17_binary_function_compat.patch
)

string(COMPARE EQUAL "${VCPKG_LIBRARY_LINKAGE}" "dynamic" PCL_SHARED_LIBS)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_2d=ON
        -DBUILD_apps=OFF
        -DBUILD_common=ON
        -DBUILD_examples=OFF
        -DBUILD_features=ON
        -DBUILD_filters=ON
        -DBUILD_geometry=ON
        -DBUILD_global_tests=OFF
        -DBUILD_io=ON
        -DBUILD_kdtree=ON
        -DBUILD_keypoints=ON
        -DBUILD_ml=ON
        -DBUILD_octree=ON
        -DBUILD_outofcore=OFF
        -DBUILD_people=OFF
        -DBUILD_recognition=OFF
        -DBUILD_registration=ON
        -DBUILD_sample_consensus=ON
        -DBUILD_search=ON
        -DBUILD_segmentation=ON
        -DBUILD_simulation=OFF
        -DBUILD_stereo=OFF
        -DBUILD_surface=ON
        -DBUILD_surface_on_nurbs=OFF
        -DBUILD_tools=OFF
        -DBUILD_tracking=OFF
        -DBUILD_visualization=OFF
        -DBUILD_CUDA=OFF
        -DBUILD_GPU=OFF
        -DPCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32=${PCL_SHARED_LIBS}
        -DPCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32=${PCL_SHARED_LIBS}
        -DPCL_BUILD_WITH_QHULL_DYNAMIC_LINKING_WIN32=${PCL_SHARED_LIBS}
        -DPCL_SHARED_LIBS=${PCL_SHARED_LIBS}
        -DWITH_CUDA=OFF
        -DWITH_LIBUSB=OFF
        -DWITH_OPENNI=OFF
        -DWITH_OPENNI2=OFF
        -DWITH_PCAP=OFF
        -DWITH_PNG=OFF
        -DWITH_QHULL=OFF
        -DWITH_QT=OFF
        -DWITH_VTK=OFF
    MAYBE_UNUSED_VARIABLES
        PCL_BUILD_WITH_QHULL_DYNAMIC_LINKING_WIN32
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH share/pcl)
vcpkg_copy_pdbs()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE.txt")
