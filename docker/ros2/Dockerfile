FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# Building libfranka and the hardware plugin concurrently can exceed the
# memory available to a desktop Docker daemon. Keep the production image build
# deterministic; users can override this at build time if they have more RAM.
ENV CMAKE_BUILD_PARALLEL_LEVEL=2
ARG MIOS_BUILD_TESTING=ON
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-pip python3-rosdep python3-vcstool libpoco-dev nlohmann-json3-dev \
      ros-jazzy-controller-manager ros-jazzy-ros2-control \
      ros-jazzy-joint-state-broadcaster ros-jazzy-xacro

# Keep apt indices for rosdep below. They are cleaned after dependency
# resolution rather than before rosdep invokes apt-get.

# Pin only the upstream packages required for the hardware path. The complete
# dependency.repos also brings Gazebo, ZED, and vision packages, which are
# deliberately excluded from this no-MoveIt/no-simulation image.
# The upstream Noble libfranka package matches the Franka Jazzy dependency pin
# and avoids building libfranka's test targets inside this runtime image.
RUN mkdir -p src \
    && git clone https://github.com/frankarobotics/franka_ros2.git src/franka_ros2 \
    && git -C src/franka_ros2 checkout ee1b79b77e473751852b4f884d1f7ce86041dfff \
    && git clone --branch 2.8.1 --depth 1 https://github.com/frankarobotics/franka_description.git src/franka_description \
    # franka_bringup is also the real-hardware launch package, but upstream
    # declares optional simulation, mobile, and example-controller packages
    # as workspace dependencies.  Remove those optional dependencies so the
    # FR3 hardware path can be built without Gazebo or MoveIt 2.
    && sed -i -E '/franka_(gazebo|mobile|example|vision_and_manipulation_kit)/d' src/franka_ros2/franka_bringup/package.xml \
    && curl --fail --location --output /tmp/libfranka.deb \
         https://github.com/frankarobotics/libfranka/releases/download/0.20.5/libfranka_0.20.5_noble_amd64.deb \
    && dpkg --install /tmp/libfranka.deb \
    && rm /tmp/libfranka.deb \
    && rosdep update \
    && rosdep install --ignore-src --rosdistro jazzy -y \
         --skip-keys="ament_cmake_clang_format xacro libfranka" \
         --from-paths src/franka_ros2/franka_hardware \
                      src/franka_ros2/franka_msgs \
                      src/franka_ros2/franka_semantic_components \
                      src/franka_description \
    && rm -rf /var/lib/apt/lists/*

COPY mios/docker/ros2/franka_combined_control.patch /tmp/franka_combined_control.patch
RUN git -C /ws/src/franka_ros2 apply --check /tmp/franka_combined_control.patch \
    && git -C /ws/src/franka_ros2 apply /tmp/franka_combined_control.patch \
    && rm /tmp/franka_combined_control.patch

COPY mios/ros2_ws/src/mios_msgs /ws/src/mios_msgs
COPY mios/ros2_ws/src/mios_ros2_control /ws/src/mios_ros2_control
COPY mios/ros2_ws/src/mios_ros2_runtime /ws/src/mios_ros2_runtime
# The runtime uses this contract to consume commands from the original MIOS
# algorithms. It is data-only and contains no FCI/libfranka dependency.
COPY mios/src/utils/include/mios/control /ws/src/mios_control_types/include/mios/control
# These are small, neutral Core interface contracts only. They contain no
# PandaBody implementation, libfranka include, or FCI ownership.
COPY mios/src/panda/include/mios/panda/robot_backend.hpp /ws/src/mios_core_contract/include/mios/panda/robot_backend.hpp
COPY mios/src/utils/include/mios/utils/types.hpp /ws/src/mios_core_contract/include/mios/utils/types.hpp

# Build only the MIOS plugin here. The upstream sources remain pinned in
# /ws/src for the hardware-integration build, but must not shadow Jazzy's
# binary ros2_control packages during this focused controller build.
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --executor sequential --parallel-workers 1 \
         --packages-select \
           franka_description \
           franka_msgs \
           franka_semantic_components \
           franka_hardware \
           franka_robot_state_broadcaster \
           franka_gripper \
           franka_bringup \
           mios_msgs \
           mios_ros2_control \
           mios_ros2_runtime \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF

# Build MIOS-only CTest executables without enabling the upstream Franka test
# suites. These tests run with --network none in the validation command.
RUN if [ "${MIOS_BUILD_TESTING}" = "ON" ]; then \
      source /opt/ros/jazzy/setup.bash \
      && source /ws/install/setup.bash \
      && colcon build --symlink-install --executor sequential --parallel-workers 1 \
           --packages-select mios_ros2_control mios_ros2_runtime \
           --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON; \
    fi

# Build the original MIOS Core as a ROS-owned process. Only its neutral Core
# implementation and ROS adapter are compiled: PandaBody and MIOS-owned
# libfranka/FCI are explicitly disabled. The ROS transport was installed above
# and is imported through CMAKE_PREFIX_PATH.
COPY mios/CMakeLists.txt mios/conanfile.txt /opt/mios/
COPY mios/cmake /opt/mios/cmake
COPY mios/include /opt/mios/include
COPY mios/src /opt/mios/src
RUN apt-get update && apt-get install -y --no-install-recommends fping \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --target /opt/mios-conan conan==1.66.0
RUN --mount=type=cache,target=/root/.conan \
    source /opt/ros/jazzy/setup.bash \
    && source /ws/install/setup.bash \
    && export PATH=/opt/mios-conan/bin:${PATH} PYTHONPATH=/opt/mios-conan:${PYTHONPATH} \
    && cmake -S /opt/mios -B /opt/mios/build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/opt/mios/install \
         '-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib' \
         -DMIOS_BUILD_LEGACY_PANDA=OFF \
         -DMIOS_BUILD_LEGACY_MAIN=OFF \
         -DMIOS_BUILD_ROS2_CORE_RUNTIME=ON \
         -DENABLE_TESTING=ON \
         -DENABLE_PCH=OFF \
    && cmake --build /opt/mios/build \
         --target mios_ros2_core_runtime test_joint_position_pipeline --parallel 2 \
    && ctest --test-dir /opt/mios/build --output-on-failure \
         --tests-regex '^test_joint_position_pipeline$' \
    && cmake --install /opt/mios/build \
    && install -d /opt/mios/install/lib \
    && cp -a /opt/mios/build/lib/boost/libboost_*.so* /opt/mios/install/lib/

# Core loads the retained MIOS controller and safety plugins dynamically. They
# are versioned prebuilt plugin artifacts under the source tree, so place them
# beside the executable's installed runtime libraries.
RUN find /opt/mios/src/plugins -type f -name 'lib*.so' \
    -exec cp -a {} /opt/mios/install/lib/ \;

# The ROS-owned Core retains MIOS's Portal API.  Include the small existing
# Python WebSocket helper so an operator can submit an original MIOS task
# through that API without publishing a low-level ROS command directly.
COPY mios/python/utils/ws_client.py /opt/mios/python/utils/ws_client.py
# The teaching entry point belongs in this motion image as well: it must use
# the local controller manager to establish the pre-configured effort session
# before it starts a Portal HandGuiding task. The learning-service image has
# no ROS 2 CLI and is deliberately unable to claim the robot controller.
COPY mios/python/mios_examples.py /opt/mios/python/mios_examples.py
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-websockets \
    && rm -rf /var/lib/apt/lists/*

COPY mios/docker/ros2/entrypoint.sh /entrypoint.sh
COPY mios/docker/ros2/start_model_broadcaster.sh /usr/local/bin/start_model_broadcaster.sh
RUN chmod +x /entrypoint.sh
RUN chmod +x /usr/local/bin/start_model_broadcaster.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
