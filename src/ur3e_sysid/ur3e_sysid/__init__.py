"""UR3e actuator system identification (system-id).

Two stages (see docs/Robot_Control/ur3e_programme_identification_gains.md §6):
  - online  ``run_sweep``  — excite one joint + record /joint_states to CSV;
  - offline ``fit_gains``  — fit (wn, zeta, L), inertia, friction -> YAML.

The math-only modules (:mod:`signals`, :mod:`estimator`, :mod:`inertia`,
:mod:`excitation`, :mod:`recorder`) avoid importing rclpy so they stay
unit-testable off-robot.
"""
