"""Round-trip contract for the canonical StepRecord CSV loader.

Regression tests for a 2026-07-31 review finding: sim/analysis/compare.py
carried a duplicate CSV loader that hard-coded solve_status as the only
string column, so any current-schema CSV (which includes throw_phase: str,
default "") crashed every /diagnose, compare and interactive-plot entry
point with ``ValueError: could not convert string to float: ''``.  A second,
quieter defect in TelemetryLogger.load() returned int-annotated fields
(ball0_held, catches_total, fk_iterations, ...) as str.

controller.telemetry.load_records() is now the single typed loader; these
tests pin TYPES, not just values, so neither defect can silently return.
"""

import dataclasses

from controller.telemetry import StepRecord, TelemetryLogger, load_records
from analysis.compare import load_csv


_INT_FIELDS = [f.name for f in dataclasses.fields(StepRecord) if f.type == 'int']
_STR_FIELDS = [f.name for f in dataclasses.fields(StepRecord) if f.type == 'str']


def _sample_records() -> list:
    """One default record (the historical crash case: throw_phase == '')
    plus one with every non-float type exercised at a non-default value."""
    populated = StepRecord(
        time=0.025,
        solve_time_ms=1.25,
        solve_status='optimal',
        throw_phase='bb_throw',
        ball0_held=1,
        ball1_held=0,
        catches_total=3,
        fk_iterations=7,
        ipopt_iter=12,
    )
    return [StepRecord(), populated]


def _write_csv(tmp_path) -> str:
    path = str(tmp_path / 'roundtrip.csv')
    logger = TelemetryLogger(path)
    for rec in _sample_records():
        logger.append(rec)
    logger.flush()
    return path


def test_load_records_roundtrips_values_exactly(tmp_path):
    path = _write_csv(tmp_path)
    assert load_records(path) == _sample_records()


def test_load_records_restores_annotated_types(tmp_path):
    path = _write_csv(tmp_path)
    for rec in load_records(path):
        for name in _INT_FIELDS:
            assert type(getattr(rec, name)) is int, name
        for name in _STR_FIELDS:
            assert type(getattr(rec, name)) is str, name
        assert type(rec.time) is float


def test_default_record_with_empty_throw_phase_loads(tmp_path):
    # The exact crash case: throw_phase defaults to '' and the old
    # compare.py loader called float('') on it.
    path = _write_csv(tmp_path)
    first = load_records(path)[0]
    assert first.throw_phase == ''
    assert first == StepRecord()


def test_compare_load_csv_delegates_to_canonical_loader(tmp_path):
    path = _write_csv(tmp_path)
    assert load_csv(path) == load_records(path)


def test_logger_load_uses_typed_loader(tmp_path):
    path = str(tmp_path / 'logger.csv')
    logger = TelemetryLogger(path)
    for rec in _sample_records():
        logger.append(rec)
    loaded = logger.load()
    assert loaded == _sample_records()
    assert type(loaded[1].catches_total) is int


def test_int_converter_tolerates_float_formatted_cells(tmp_path):
    # Historical CSVs may carry '1.0'-style cells in int columns.
    path = _write_csv(tmp_path)
    with open(path) as f:
        header, first, second = f.read().splitlines()
    cols = header.split(',')
    row = second.split(',')
    row[cols.index('ball0_held')] = '1.0'
    with open(path, 'w') as f:
        f.write('\n'.join([header, first, ','.join(row)]) + '\n')
    rec = load_records(path)[1]
    assert rec.ball0_held == 1
    assert type(rec.ball0_held) is int


def test_every_steprecord_field_has_a_csv_converter():
    # The hard-fail contract: adding a StepRecord field with an annotation
    # outside float/int/str must be a loud error, never a silent str.
    types = {f.type if isinstance(f.type, str) else f.type.__name__
             for f in dataclasses.fields(StepRecord)}
    assert types <= {'float', 'int', 'str'}, (
        f"StepRecord grew a field type outside the CSV round-trip contract: "
        f"{types - {'float', 'int', 'str'}} - extend _CSV_CONVERTERS and "
        f"these tests together"
    )
