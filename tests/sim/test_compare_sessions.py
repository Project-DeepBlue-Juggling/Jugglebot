"""Tests for sim/analysis/compare_sessions.py — session comparison."""

import os

import pytest

from sim.analysis.compare_sessions import (
    _pct_change,
    _delta_str,
    _count_flags,
    _verdict,
    compare_sessions,
)


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

class TestPctChange:
    def test_positive_change(self):
        assert _pct_change(10.0, 15.0) == 50.0

    def test_negative_change(self):
        assert _pct_change(10.0, 5.0) == -50.0

    def test_no_change(self):
        assert _pct_change(10.0, 10.0) == 0.0

    def test_zero_baseline(self):
        assert _pct_change(0.0, 5.0) is None

    def test_near_zero_baseline(self):
        assert _pct_change(1e-12, 5.0) is None


class TestDeltaStr:
    def test_improvement_lower_is_better(self):
        result = _delta_str(10.0, 5.0, 'mm', lower_is_better=True)
        assert 'down' in result
        assert 'improved' in result

    def test_regression_lower_is_better(self):
        result = _delta_str(5.0, 10.0, 'mm', lower_is_better=True)
        assert 'up' in result
        assert 'worse' in result

    def test_improvement_higher_is_better(self):
        result = _delta_str(5.0, 10.0, '', lower_is_better=False)
        assert 'up' in result
        assert 'improved' in result

    def test_unchanged(self):
        result = _delta_str(10.0, 10.0, 'mm')
        assert result == 'unchanged'

    def test_nearly_unchanged(self):
        result = _delta_str(10.0, 10.001, 'mm')
        assert result == 'unchanged'


class TestCountFlags:
    def test_count_errors(self):
        flags = [
            {'severity': 'error', 'message': 'a'},
            {'severity': 'warning', 'message': 'b'},
            {'severity': 'error', 'message': 'c'},
        ]
        assert _count_flags(flags, 'error') == 2
        assert _count_flags(flags, 'warning') == 1
        assert _count_flags(flags, 'info') == 0

    def test_empty_flags(self):
        assert _count_flags([], 'error') == 0


class TestVerdict:
    def test_pass(self):
        assert _verdict([]) == 'PASS'
        assert _verdict([{'severity': 'info'}]) == 'PASS'

    def test_needs_attention(self):
        assert _verdict([{'severity': 'warning'}]) == 'NEEDS_ATTENTION'

    def test_fail(self):
        assert _verdict([{'severity': 'error'}]) == 'FAIL'
        # Error takes precedence over warning
        flags = [{'severity': 'warning'}, {'severity': 'error'}]
        assert _verdict(flags) == 'FAIL'


# ---------------------------------------------------------------------------
# Full comparison (using real CSV data if available)
# ---------------------------------------------------------------------------

class TestCompareSessionsIntegration:
    @pytest.fixture
    def csv_paths(self):
        """Find two real CSV files to compare."""
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        logs_dir = os.path.join(repo_root, 'temp', 'logs')

        if not os.path.isdir(logs_dir):
            pytest.skip('temp/logs directory not found')

        csvs = sorted(
            f for f in os.listdir(logs_dir)
            if f.startswith('mpc_') and f.endswith('.csv')
        )

        if len(csvs) < 2:
            pytest.skip('Need at least 2 CSV files for comparison test')

        return (
            os.path.join(logs_dir, csvs[0]),
            os.path.join(logs_dir, csvs[1]),
        )

    def test_compare_returns_valid_structure(self, csv_paths):
        result = compare_sessions(csv_paths[0], csv_paths[1])
        assert 'error' not in result
        assert 'label_a' in result
        assert 'label_b' in result
        assert 'rows' in result
        assert len(result['rows']) > 0

    def test_compare_rows_have_required_fields(self, csv_paths):
        result = compare_sessions(csv_paths[0], csv_paths[1])
        for row in result['rows']:
            assert 'metric' in row
            assert 'before' in row
            assert 'after' in row
            assert 'delta' in row

    def test_compare_with_labels(self, csv_paths):
        result = compare_sessions(
            csv_paths[0], csv_paths[1],
            labels=('before', 'after'),
        )
        assert result['label_a'] == 'before'
        assert result['label_b'] == 'after'

    def test_compare_includes_verdict(self, csv_paths):
        result = compare_sessions(csv_paths[0], csv_paths[1])
        verdict_rows = [r for r in result['rows'] if r['metric'] == 'Verdict']
        assert len(verdict_rows) == 1
        assert verdict_rows[0]['before'] in ('PASS', 'NEEDS_ATTENTION', 'FAIL')
        assert verdict_rows[0]['after'] in ('PASS', 'NEEDS_ATTENTION', 'FAIL')
