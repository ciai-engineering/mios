ARG BASE_IMAGE=mios-ros2-jazzy:bridge-test
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /opt/mios-direct

# Build the original MIOS Core with PandaBody as the only FCI owner. The base
# image supplies the exact libfranka 0.20.5 package that passed the isolated
# 90-second direct transport preflight; do not download a second libfranka.
COPY mios /opt/mios-direct
RUN --mount=type=cache,target=/root/.conan \
    source /opt/ros/jazzy/setup.bash \
    && export PATH=/opt/mios-conan/bin:${PATH} PYTHONPATH=/opt/mios-conan:${PYTHONPATH} \
    && cmake -S /opt/mios-direct -B /opt/mios-direct/build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/opt/mios-direct/install \
         -DMIOS_BUILD_LEGACY_PANDA=ON \
         -DMIOS_BUILD_LEGACY_MAIN=ON \
         -DMIOS_USE_SYSTEM_LIBFRANKA=ON \
         -DMIOS_BUILD_ROS2_CORE_RUNTIME=OFF \
         -DENABLE_TESTING=ON \
         -DENABLE_PCH=OFF \
    && cmake --build /opt/mios-direct/build --target mios test_joint_position_pipeline --parallel 2 \
    && ctest --test-dir /opt/mios-direct/build --output-on-failure \
         --tests-regex '^test_joint_position_pipeline$' \
    && cmake --install /opt/mios-direct/build \
    && cp /opt/mios-direct/install/bin/mios /opt/mios-direct/mios/bin/mios

# The base entrypoint merely sources ROS. It does not start franka_hardware;
# this image is the exclusive FCI client when it is running.
WORKDIR /opt/mios-direct/mios/bin
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "./mios.sh"]
