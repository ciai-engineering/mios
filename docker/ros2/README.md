# ROS-only MIOS deployment

This deployment uses `franka_hardware` as the **only** FCI client.  The MIOS
Core runs separately with `Ros2CoreRobotBackend`, communicates through ROS 2
topics/actions, and retains the existing MIOS Portal at port `12000`.  It never
constructs `PandaBody` or a `franka::Robot` object.

Do not start `docker/direct` while this stack is running.

## Build and state-only start

From the repository root:

```bash
docker compose -f docker/ros2/docker-compose.runtime.yml build
docker compose -f docker/ros2/docker-compose.runtime.yml up -d
docker compose -f docker/ros2/docker-compose.runtime.yml logs -f mios_ros2_control
```

The default start enables only Franka state broadcasters and the read-only
MIOS model broadcaster.  It does not start the Core Portal or claim an MIOS
arm command interface.

## Real-time CPU placement

The FR3 FCI controller requires a 1 kHz communication loop.  The
controller-manager update thread itself is pinned to CPU 6 in
`mios_controllers.yaml`; all other threads in the FCI container inherit
`MIOS_CONTROL_WORKER_CPUS` (`0-5,8-19` by default). CPU 7 is the SMT sibling
of the reserved controller CPU 6 and is intentionally left idle; Core/model
containers use the same non-real-time set. On this commissioning host CPU 6
is reserved for the Franka Ethernet interrupt. Pin the actual NIC IRQ at boot
as well (the helper discovers the IRQ dynamically, so it does not rely on an
unstable PCI MSI number):

```bash
sudo install -m 0755 tools/pin_franka_nic_irq.sh /usr/local/sbin/mios-pin-franka-nic-irq
sudo install -m 0644 docker/ros2/mios-franka-nic-irq.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mios-franka-nic-irq.service
```

### Isolate the controller's physical CPU core

On this host CPU 7 is the SMT sibling of CPU 6, so reserve the whole physical
core. Add the following to `GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub`,
run `sudo update-grub`, and reboot:

```text
isolcpus=managed_irq,domain,6-7 nohz_full=6-7 rcu_nocbs=6-7 irqaffinity=0-5,8-19
```

`irqaffinity` keeps ordinary IRQs off the reserved core. The
`mios-franka-nic-irq` service then pins only the Franka NIC IRQ back to CPU 6.
After boot, verify `cat /sys/devices/system/cpu/isolated` reports `6-7` and
that the NIC IRQ's `/proc/irq/<number>/smp_affinity_list` reports `6`.

If the Franka interface or reserved CPU changes, update the controller
manager's `cpu_affinity`, `MIOS_CONTROL_WORKER_CPUS`, `MIOS_NONRT_CPUS`, and
the IRQ service environment values together. Do **not** apply a Docker CPU
set to the whole FCI container: it would force libfranka's several FIFO-99
worker threads onto one CPU and can starve the controller-manager loop during
a mode switch. The Compose `taskset` wrapper deliberately excludes only the
reserved CPU from ordinary FCI workers; `controller_manager` then assigns its
single update thread back to that CPU.
Do not raise the controller rate above 1 kHz or change controller priorities
while commissioning physical motion.

## Enable the ROS Core Portal for a supervised task

Only with the workspace clear, FCI enabled, the gripper available, and an
operator supervising the robot, explicitly enable every Core task gate:

```bash
MIOS_LOAD_GRIPPER=true \
MIOS_ENABLE_CORE_SCHEDULER=true \
MIOS_ENABLE_CORE_TASK_EXECUTION=true \
MIOS_ALLOW_CONTROLLER_OWNED_MOVE_MODE=true \
docker compose -f docker/ros2/docker-compose.runtime.yml --profile core up -d
```

`MIOS_ALLOW_ROBOT_PARAMETER_APPLICATION` must remain **false** for a
supervised task.  Franka parameter services (load, TCP, collision behaviour,
and stiffness) are not valid while an arm controller has the FCI in Move
mode.  Provision those values in Desk, or in a separately supervised Idle
configuration session before activating an arm command controller.

Before a learning task, prepare the post-task ROS position-hold controller in
the `mios-ros2-control` container.  This is a physical controller activation;
use it only during a supervised commissioning session:

```bash
docker exec -it mios-ros2-control bash -lc '
  ros2 control load_controller mios_joint_position_controller || true
  ros2 control set_controller_state mios_joint_position_controller inactive
  ros2 control set_controller_state mios_joint_position_controller active
'
```

Teach through the Portal while the ROS effort controller is the FCI command
owner:

```bash
docker exec -it mios-ros2-control python3 -u /opt/mios/python/mios_examples.py \
  --object samuelnew --grasp-width 0.02501746080815792 \
  --grasp-speed 0.01 --accept-contact-width --ros-handguiding --execute
```

Then the learning service uses the same ROS Core Portal and performs its
controller handoff through `mios-ros2-control`:

```bash
docker exec -it mios-ml-service python3 -u /mios_mls/example_learning.py
```

Stop the ROS-only stack before returning to the direct deployment:

```bash
docker compose -f docker/ros2/docker-compose.runtime.yml --profile core down
```
