# UR3e Actuator Identification

> Sources: system-id program, 2026-06-29; system-id results, 2026-06-29; actuator parameter reference, 2026-06-29
> Raw: [System-id program](../../docs/Robot_Control/ur3e_programme_identification_gains.md); [System-id results](../../docs/Robot_Control/ur3e_resultats_identification_gains.md); [Actuator parameter reference](../../docs/Robot_Control/ur3e_parametres_actionneur_reference.md)

## Overview

System identification estimates surrogate actuator behavior for Isaac/UR3e
transfer: stiffness, damping, effective inertia, friction and delay. The docs
separate the measurement program from the measured results.

## Main Code

- `src/ur3e_sysid/ur3e_sysid/run_sweep.py`
- `src/ur3e_sysid/ur3e_sysid/fit_gains.py`
- `src/ur3e_sysid/ur3e_sysid/estimator.py`
- `src/ur3e_sysid/ur3e_sysid/excitation.py`
- `src/ur3e_sysid/ur3e_sysid/recorder.py`
- `scripts/sysid_frf_check.py`

## Caveats

- Wrist measurements were taken without the hoop, so treat them as initial
  simulation values.
- `shoulder_lift` is the least clean fit.
- The package is present locally but is noted as untracked in the current
  workspace state.

## See Also

- [Policy Transfer And Action Semantics](../sim-to-real/policy-transfer-and-action-semantics.md)
- [UR3e Control Stack](../robot-control/ur3e-control-stack.md)
