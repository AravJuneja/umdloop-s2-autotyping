from typist.e2e_check import _record_latest, _result_row


def test_record_latest_replaces_stale_result():
    stale = object()
    fresh = object()
    received = {}

    _record_latest(received, stale)
    _record_latest(received, fresh)

    assert received['r'] is fresh


def test_result_row_reports_outcome_and_counts():
    class Result:
        seed = 4
        target = 'MARS'
        exact_match = True
        presses_attempted = 4
        presses_accepted = 4
        rejection_reasons = ['NO_KEY']
        rejection_counts = [1]
        elapsed = 12.5

    row = _result_row('MARS', Result(), wall_elapsed=13.0, failure='')

    assert row['code'] == 'MARS'
    assert row['exact_match'] == 'True'
    assert row['accepted'] == '4'
    assert row['rejections'] == 'NO_KEY:1'
    assert row['failure'] == ''
