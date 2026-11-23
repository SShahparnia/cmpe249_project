# ARiADNE log (Module 7)

Owners: Joey, Aaron. Hard deadline: **end of Week 4.** If Step A or Step C fails, switch to `roadmap` and record it in [baseline_provenance.md](baseline_provenance.md). Do not keep debugging ARiADNE in Week 5.

## Hours

| Date | Who | Hours | What |
|------|-----|-------|------|
| 2026-11-23 | Aaron | TODO | Code reading: ROS version, interfaces, checkpoint, training vs. deployment params. Wrote `harness/ariadne_adapter` and the timing patch (not built or run yet). |

## Step A: does the model work? (authors' Python environment)

Findings from reading the repos (2026-11-23):

- The pinned `methods/ARiADNE` is the **`new_version`** branch. It has training code (`driver.py`, `runner.py`, `worker.py`, `env.py`) and 5,663 training maps in `maps/`, but **no test script and no checkpoint**.
- The only checkpoint is in the ROS planner: `methods/ARiADNE-ROS-Planner/src/rl_planner/rl_planner/model/checkpoint.pth` (51 MB, key `policy_model`).
- The README of `new_version` says the original implementation is on the `main` branch. The test script the instructions expect may be there. **TODO:** check `main` for a test driver and test set, and decide which code and checkpoint pair Step A uses. The pair has to match: the ROS checkpoint may not load into the `main` branch model.
- Python requirements (README): PyTorch, `scikit-image`, `matplotlib`, `ray`, `tensorboard`; the ROS planner README uses Python 3.10.12 and the CPU torch wheel.

| Item | Value |
|------|-------|
| Python / PyTorch versions | TODO |
| Test episodes | TODO |
| Our mean trajectory length | TODO |
| Paper's mean trajectory length | TODO (full text) |
| Within ~10%? | TODO |

## Step B: does it run on the Jetson?

| Item | Value |
|------|-------|
| PyTorch wheel for our JetPack | TODO (VERIFY against `docs/jetson_setup.md` JetPack version) |
| Checkpoint load time | TODO |
| Inference time per step | TODO (`/runtime` topic or `PlanningCycle`) |
| Memory used (jtop) | TODO |

Known blocker, already handled in our launch: the authors' `rl_planner.launch.py` preloads `/usr/lib/x86_64-linux-gnu/libstdc++.so.6`, which does not exist on aarch64.

## Step C: does it run in our ROS 2 stack?

- [x] ROS version: **ROS 2 Humble** (`humble` branch). No port needed.
- [x] Inputs and outputs listed (see [baseline_provenance.md](baseline_provenance.md)).
- [x] Adapter written: `harness/ariadne_adapter` (**not built or run yet**).
- [ ] Map-resolution decision recorded (0.4 m downsample vs. raw 0.05 m).
- [ ] Run on the default TB4 world until the map is mostly covered.

## Decision

| Date | Decision | Reason | Who |
|------|----------|--------|-----|
| TODO | ARiADNE / roadmap | | |
