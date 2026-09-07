message(STATUS "--- Loading Project Dependencies ---")
include(FetchContent)
# ==========================================
# 1. System & Pre-built Packages
# ==========================================

find_package(Eigen3 REQUIRED)

if(MIOS_BUILD_LEGACY_PANDA)
    # The direct Core container deliberately uses the same system libfranka
    # package as the separately validated transport preflight.  Keep the old
    # self-contained downloader as an explicit fallback for legacy builds.
    if(MIOS_USE_SYSTEM_LIBFRANKA)
        find_package(Franka 0.20.5 EXACT REQUIRED)
    else()
        # --- AUTOMATED LOCAL LIBFRANKA INSTALL ---
        set(FRANKA_VERSION "0.21.1")

        # 1. Dynamically detect the Ubuntu codename
        execute_process(
            COMMAND lsb_release -cs
            OUTPUT_VARIABLE UBUNTU_CODENAME
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        message(STATUS "Detected Ubuntu release: ${UBUNTU_CODENAME}")

        if(NOT UBUNTU_CODENAME MATCHES "^(focal|jammy|noble)$")
            message(WARNING "Unrecognized Ubuntu version '${UBUNTU_CODENAME}'. Defaulting to 'jammy'.")
            set(UBUNTU_CODENAME "jammy")
        endif()

        # 2. Construct the exact filenames and URLs
        set(FRANKA_DEB_NAME "libfranka_${FRANKA_VERSION}_${UBUNTU_CODENAME}_amd64.deb")
        set(FRANKA_DEB_URL "https://github.com/frankarobotics/libfranka/releases/download/${FRANKA_VERSION}/${FRANKA_DEB_NAME}")
        set(FRANKA_SHA_URL "${FRANKA_DEB_URL}.sha256")

        set(FRANKA_LOCAL_DIR "${CMAKE_SOURCE_DIR}/local_deps/franka")
        set(FRANKA_DEB_FILE "${FRANKA_LOCAL_DIR}/${FRANKA_DEB_NAME}")
        set(FRANKA_SHA_FILE "${FRANKA_LOCAL_DIR}/libfranka.sha256")

        # 3. Download, verify, and extract (only if not already done)
        if(NOT EXISTS "${FRANKA_LOCAL_DIR}/usr")
            message(STATUS "Preparing to install libfranka ${FRANKA_VERSION} for ${UBUNTU_CODENAME}...")
            file(MAKE_DIRECTORY "${FRANKA_LOCAL_DIR}")

            # STEP A: Download the SHA256 file
            message(STATUS "Fetching SHA256 checksum...")
            file(DOWNLOAD "${FRANKA_SHA_URL}" "${FRANKA_SHA_FILE}" STATUS sha_status)
            list(GET sha_status 0 sha_code)
            if(NOT sha_code EQUAL 0)
                message(FATAL_ERROR "Failed to download SHA256 file!\nURL: ${FRANKA_SHA_URL}")
            endif()

            # STEP B: Read the file and extract the 64-character hash
            file(READ "${FRANKA_SHA_FILE}" RAW_SHA_CONTENT)
            string(SUBSTRING "${RAW_SHA_CONTENT}" 0 64 EXPECTED_HASH)
            message(STATUS "Expected SHA256: ${EXPECTED_HASH}")

            # STEP C: Download the .deb file and let CMake verify the hash automatically
            message(STATUS "Downloading and verifying libfranka package...")
            file(DOWNLOAD "${FRANKA_DEB_URL}" "${FRANKA_DEB_FILE}"
                 EXPECTED_HASH SHA256=${EXPECTED_HASH}
                 STATUS deb_status
                 SHOW_PROGRESS)

            list(GET deb_status 0 deb_code)
            list(GET deb_status 1 deb_msg)
            if(NOT deb_code EQUAL 0)
                message(FATAL_ERROR "Download or Hash Verification failed: ${deb_msg}")
            endif()

            # STEP D: Extract the verified package
            message(STATUS "Extracting verified libfranka into local_deps...")
            execute_process(
                COMMAND dpkg -x "${FRANKA_DEB_FILE}" "${FRANKA_LOCAL_DIR}"
                RESULT_VARIABLE extract_result
            )
            if(NOT extract_result EQUAL 0)
                message(FATAL_ERROR "Failed to extract libfranka.deb!")
            endif()

            message(STATUS "Successfully installed local libfranka.")
        endif()

        # 4. Point CMake to the dynamically downloaded package
        list(APPEND CMAKE_PREFIX_PATH "${FRANKA_LOCAL_DIR}/usr")
        find_package(Franka REQUIRED)
    endif()
else()
    message(STATUS "MIOS legacy Panda/libfranka transport disabled.")
endif()



# 3. Tell FetchContent to skip the download step and just use our manually cloned folder
FetchContent_Declare(
    mirmi_cpp_utils
    GIT_REPOSITORY https://github.com/SchneiderROS/mirmi_utils
    GIT_TAG v1.7.4
    GIT_SHALLOW FALSE)
# set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE INTERNAL "")


# Build the downloaded source
FetchContent_MakeAvailable(mirmi_cpp_utils)

# mirmi_utils v1.7.4 uses std::numeric_limits in its files component without
# including <limits>.  The ROS-only Core build does not inherit the legacy PCH
# that used to hide this upstream omission.  Keep the compatibility include
# confined to that third-party target and to compilers that support -include.
if(TARGET files AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(files PRIVATE -include limits)
endif()
