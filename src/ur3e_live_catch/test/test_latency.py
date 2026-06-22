"""LatencyStats accumulator (archi §10, step 8)."""

import pytest

from ur3e_live_catch.latency import LatencyStats


def test_empty_stats_are_zero():
    s = LatencyStats()
    assert s.count == 0
    assert s.mean == 0.0 and s.min == 0.0 and s.max == 0.0
    assert s.percentile(95) == 0.0
    assert "no samples" in s.format_ms("x")


def test_basic_aggregates():
    s = LatencyStats()
    for v in [0.001, 0.002, 0.003, 0.004]:
        s.add(v)
    assert s.count == 4
    assert s.mean == pytest.approx(0.0025)
    assert s.min == pytest.approx(0.001)
    assert s.max == pytest.approx(0.004)


def test_nearest_rank_percentiles():
    s = LatencyStats()
    for i in range(1, 101):     # 1..100 ms
        s.add(i / 1000.0)
    assert s.percentile(50) == pytest.approx(0.050)
    assert s.percentile(95) == pytest.approx(0.095)
    assert s.percentile(99) == pytest.approx(0.099)
    assert s.percentile(100) == pytest.approx(0.100)
    assert s.percentile(0) == pytest.approx(0.001)


def test_window_bounds_percentile_memory_but_not_count():
    s = LatencyStats(window=10)
    for i in range(1000):
        s.add(float(i))
    # running count/sum/min/max are exact over the whole session
    assert s.count == 1000
    assert s.min == pytest.approx(0.0)
    assert s.max == pytest.approx(999.0)
    # percentiles only see the last 10 samples (990..999)
    assert s.percentile(0) == pytest.approx(990.0)
    assert s.percentile(100) == pytest.approx(999.0)


def test_bad_args():
    with pytest.raises(ValueError):
        LatencyStats(window=0)
    with pytest.raises(ValueError):
        LatencyStats().percentile(101)
