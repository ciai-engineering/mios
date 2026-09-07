#!/usr/bin/env bash
# Pin every interrupt belonging to the network interface used for the Franka
# robot to the CPU reserved for the ROS 2 FCI container.  This script is safe
# to run repeatedly and deliberately discovers the route/IRQ at runtime: PCI
# MSI interrupt numbers are not stable across host reboots.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run as root (for example: sudo $0)." >&2
  exit 1
fi

robot_ip=${ROBOT_IP:-192.168.4.100}
control_cpu=${MIOS_CONTROL_CPU:-6}

if ! [[ ${control_cpu} =~ ^[0-9]+$ ]]; then
  echo "MIOS_CONTROL_CPU must be one CPU number; received '${control_cpu}'." >&2
  exit 1
fi

interface=$(ip route get "${robot_ip}" | awk '
  $1 == target {
    for (field = 1; field <= NF; ++field) {
      if ($field == "dev" && field < NF) {
        print $(field + 1)
        exit
      }
    }
  }
' target="${robot_ip}")

if [[ -z ${interface} || ${interface} == lo ]]; then
  echo "Could not determine a non-loopback route to Franka robot ${robot_ip}." >&2
  exit 1
fi

mapfile -t irq_numbers < <(awk -v interface="${interface}" '
  $0 ~ ("(^|[[:space:]])" interface "([[:space:]]|$)") {
    irq = $1
    sub(":$", "", irq)
    if (irq ~ /^[0-9]+$/) print irq
  }
' /proc/interrupts)

if (( ${#irq_numbers[@]} == 0 )); then
  echo "No IRQs found for Franka interface ${interface}." >&2
  exit 1
fi

for irq in "${irq_numbers[@]}"; do
  affinity_file="/proc/irq/${irq}/smp_affinity_list"
  if [[ ! -w ${affinity_file} ]]; then
    echo "Cannot write ${affinity_file}." >&2
    exit 1
  fi
  printf '%s\n' "${control_cpu}" > "${affinity_file}"
  printf 'Pinned IRQ %s (%s) to CPU %s.\n' "${irq}" "${interface}" "${control_cpu}"
done
