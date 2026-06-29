# System ID

## Scope

System identification estimates UR3e actuator behavior for simulation transfer:
stiffness, damping, effective inertia, friction and latency surrogates.

## Source Of Truth

- `docs/Robot_Control/ur3e_programme_identification_gains.md`
- `docs/Robot_Control/ur3e_resultats_identification_gains.md`
- `docs/Robot_Control/ur3e_parametres_actionneur_reference.md`
- `src/ur3e_sysid/`
- `ur3e_actuator_identified.yaml`

## Current State From Docs

- The program document defines safe excitation and offline fitting.
- The result document records measured K/D values and validation notes.
- Wrist values were identified without the hoop, so treat them as initial
  values for simulation rather than final mounted-system truth.
- `shoulder_lift` is called out as the least clean fit.
- `src/ur3e_sysid/` exists locally, but the docs note it is not tracked by Git
  in the current workspace state.

## Main Code Areas

- `src/ur3e_sysid/ur3e_sysid/run_sweep.py`
- `src/ur3e_sysid/ur3e_sysid/fit_gains.py`
- `src/ur3e_sysid/ur3e_sysid/estimator.py`
- `src/ur3e_sysid/ur3e_sysid/excitation.py`
- `src/ur3e_sysid/ur3e_sysid/recorder.py`
- `scripts/sysid_frf_check.py`

## Verification

```bash
cd src/ur3e_sysid
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

## Related Notes

- [[Sim_to_Real]]
- [[Robot_Control]]
- [[Current_Status]]
