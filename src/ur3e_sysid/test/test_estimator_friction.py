import numpy as np

from ur3e_sysid import estimator


def test_recover_coulomb_and_viscous():
    fc, fv = 0.8, 0.35
    v = np.array([-1.0, -0.5, -0.2, 0.2, 0.5, 1.0, 1.5])
    rng = np.random.default_rng(0)
    tau = fc * np.sign(v) + fv * v + rng.normal(0.0, 1e-3, size=v.shape)
    fc_fit, fv_fit, r2 = estimator.fit_friction(v, tau)
    assert abs(fc_fit - fc) < 0.05
    assert abs(fv_fit - fv) < 0.05
    assert r2 > 0.99


def test_reconcile_prefers_chirp_and_warns_on_gap():
    wn, zeta, warns = estimator.reconcile(10.0, 0.4, 10.5, 0.42)
    assert (wn, zeta) == (10.0, 0.4)
    assert warns == []
    _, _, warns2 = estimator.reconcile(10.0, 0.4, 20.0, 0.9)
    assert warns2  # large disagreement flagged
