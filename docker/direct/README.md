# Direct libfranka MIOS Core

This deployment uses `PandaBody` and `franka::Robot::control()` as the sole
FCI owner. Do not run `franka_bringup`, `mios-ros2-move-preflight`, or any
other libfranka client while `mios_direct` is running.

Build and start the Core after FCI is enabled:

```bash
docker compose -f docker/direct/docker-compose.yml up -d --build
docker logs -f mios_direct
```

Run teaching from the learning-service container only after the Core reports
`System is ready`:

```bash
docker exec -it mios-ml-service python3 -u /mios_mls/mios_examples.py \
  --object samuelnew --grasp-width 0.02501746080815792 \
  --grasp-speed 0.01 --accept-contact-width --direct-handguiding --execute
```

After a successful complete teaching baseline, run one supervised learning
candidate through the same direct Core:

```bash
docker exec -it mios-ml-service python3 -u /mios_mls/example_learning.py --direct-fci
```

If FCI faults, stop `mios_direct` first, clear the fault and re-enable FCI in
Desk, then start only `mios_direct` again. Do not run the learning command
until the teaching baseline has completed successfully.
