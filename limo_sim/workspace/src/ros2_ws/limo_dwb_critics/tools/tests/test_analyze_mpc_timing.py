"""Offline timing-report regression tests; run with python3 -m unittest discover."""

import contextlib
import csv
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest


TOOL_PATH = Path(__file__).resolve().parents[1] / "analyze_mpc_timing.py"
SPEC = importlib.util.spec_from_file_location("analyze_mpc_timing", TOOL_PATH)
timing = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(timing)


def sample(cycle, start=None, **changes):
    row = dict.fromkeys(timing.LEGACY_COLUMNS + timing.CADENCE_COLUMNS + timing.ASYNC_COLUMNS, 0)
    row.update(session=1, cycle=cycle, ros_time_ns=cycle * 50_000_000,
               steady_time_ns=cycle * 50_000_000, backend="SMAC", success=1,
               budget_ms=50, cycle_ms=10, cpu_ms=5, prepare_ms=3,
               solve_ms=6, scoring_ms=4, debug_ms=1,
               plan_epoch=1, replan_hz=2, generated=20, evaluated=19, rejected=1)
    if start is not None:
        row.update(plan_called=1, detouring=1, plan_start_steady_ns=start,
                   plan_ms=100, grid_updated=1, grid_ms=2)
    row.update(changes)
    return row


class TimingReportTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "mpc_timing_test.csv"

    def write(self, rows, path=None, legacy=False, async_columns=True):
        path = path or self.path
        header = timing.LEGACY_COLUMNS if legacy else timing.LEGACY_COLUMNS + timing.CADENCE_COLUMNS
        if async_columns and not legacy:
            header += timing.ASYNC_COLUMNS
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=header, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        return path

    def report(self, rows, **kwargs):
        return timing.analyze([self.write(rows, **kwargs)])

    def test_plan_statistics_exclude_idle_and_cache_rows_and_include_failures(self):
        report = self.report([
            sample(1, 1_000_000_000, plan_ms=100),
            sample(2, detouring=1),
            sample(3, 1_500_000_000, plan_ms=300, plan_failed=1, success=0),
            sample(4, success=0, grid_updated=1, deadline_missed=1),
        ])
        backend = report["backends"][0]
        self.assertEqual(backend["attempts"], 2)
        self.assertEqual(backend["plan_failures"], 1)
        self.assertEqual(backend["cached_cycles"], 1)
        self.assertEqual(backend["waiting_cycles"], 1)
        self.assertEqual(backend["failed_cycles"], 2)
        self.assertEqual(backend["deadline_misses"], 1)
        self.assertEqual(backend["metrics_ms"]["plan_ms"], {
            "count": 2, "mean": 200, "median": 200, "p95": 300, "max": 300})
        self.assertEqual(backend["metrics_ms"]["grid_ms"]["count"], 3)

    def test_cadence_uses_start_timestamps_not_variable_finish_or_logged_interval(self):
        report = self.report([
            sample(1, 1_000_000_000, steady_time_ns=1_400_000_000, plan_ms=400),
            sample(2, detouring=1, steady_time_ns=1_450_000_000),
            sample(3, 1_500_000_000, steady_time_ns=1_501_000_000,
                   plan_ms=1, plan_interval_valid=1, plan_interval_ms=101),
        ])
        result = report["backends"][0]["cadence"]
        self.assertEqual(result["status"], "VERIFIED")
        self.assertEqual(result["checked_intervals"], 1)
        self.assertEqual(result["intervals"]["min"], 500)
        self.assertEqual(result["intervals"]["observed_hz"], 2)

    def test_async_worker_time_is_separate_from_control_budget_and_running_cycles(self):
        report = self.report([
            sample(1, plan_async=1, plan_running=1, success=0, grid_updated=1),
            sample(2, plan_async=1, plan_running=1, detouring=1),
            sample(3, 1_000_000_000, plan_async=1, plan_ms=750),
            sample(4, plan_async=1, plan_running=1, detouring=1, plan_epoch=2),
            sample(5, 1_500_000_000, plan_async=1, plan_ms=300, plan_discarded=1),
        ])
        backend = report["backends"][0]
        self.assertEqual(backend["attempts"], 2)
        self.assertEqual(backend["async_completions"], 2)
        self.assertEqual(backend["discarded_completions"], 1)
        self.assertEqual(backend["async_cycles"], 5)
        self.assertEqual(backend["in_flight_cycles"], 3)
        self.assertEqual(backend["waiting_cycles"], 1)
        self.assertEqual(backend["cached_cycles"], 2)
        self.assertEqual(backend["plan_failures"], 0)
        self.assertEqual(backend["deadline_misses"], 0)
        self.assertEqual(backend["metrics_ms"]["cycle_ms"]["max"], 10)
        self.assertEqual(backend["metrics_ms"]["plan_ms"]["mean"], 525)
        self.assertEqual(backend["plan_mode_metrics_ms"]["async_worker"]["count"], 2)
        self.assertEqual(backend["plan_mode_metrics_ms"]["synchronous"]["count"], 0)
        self.assertEqual(backend["cadence"]["intervals"]["min"], 500)
        self.assertEqual(backend["cadence"]["status"], "VERIFIED")
        text = timing.render_text(report)
        self.assertIn("2 completamenti consumati, 1 scartati", text)
        self.assertIn("3 cicli con ricerca in volo", text)
        self.assertIn("non fa parte del tempo del ciclo di controllo", text)

    def test_noncompletion_rows_do_not_double_count_failure_or_discard_flags(self):
        backend = self.report([
            sample(1, 1_000_000_000, plan_async=1, plan_failed=1, plan_discarded=1),
            sample(2, plan_async=1, plan_running=1, plan_failed=1, plan_discarded=1),
            sample(3, plan_async=1, plan_running=1),
        ])["backends"][0]
        self.assertEqual(backend["attempts"], 1)
        self.assertEqual(backend["plan_failures"], 1)
        self.assertEqual(backend["discarded_completions"], 1)
        self.assertEqual(backend["in_flight_cycles"], 2)
        self.assertEqual(backend["metrics_ms"]["plan_ms"]["count"], 1)

    def test_sync_and_async_durations_remain_distinct_in_mixed_files(self):
        backend = self.report([
            sample(1, 1_000_000_000, plan_ms=20),
            sample(2, 1_500_000_000, plan_async=1, plan_ms=600),
        ])["backends"][0]
        self.assertEqual(backend["metrics_ms"]["plan_ms"]["mean"], 310)
        self.assertEqual(backend["plan_mode_metrics_ms"]["synchronous"]["mean"], 20)
        self.assertEqual(backend["plan_mode_metrics_ms"]["async_worker"]["mean"], 600)

    def test_waiting_count_excludes_failed_mpc_with_active_cache_and_other_backends(self):
        report = self.report([
            sample(1, success=0, grid_updated=1),
            sample(2, success=0, grid_updated=1, detouring=1),
            sample(3, success=0, grid_updated=0),
            sample(4, success=0, grid_updated=1, backend="DIJKSTRA", replan_hz=0),
        ])
        backends = {row["backend"]: row for row in report["backends"]}
        self.assertEqual(backends["SMAC"]["waiting_cycles"], 1)
        self.assertEqual(backends["DIJKSTRA"]["waiting_cycles"], 0)

    def test_exact_boundary_and_one_nanosecond_tolerance(self):
        report = self.report([
            sample(1, 1_000_000_000),
            sample(2, 1_500_000_000),
            sample(3, 1_999_999_999),
            sample(4, 2_499_999_997),
        ])
        result = report["backends"][0]["cadence"]
        self.assertEqual(result["status"], "VIOLATION")
        self.assertEqual(result["checked_intervals"], 3)
        self.assertEqual(len(result["violations"]), 1)
        self.assertEqual(result["violations"][0]["cycle"], 4)
        self.assertEqual(result["flag_disagreements"], 1)

    def test_new_epoch_short_interval_is_reported_separately(self):
        report = self.report([
            sample(1, 1_000_000_000),
            sample(2, 1_500_000_000),
            sample(3, 1_550_000_000, plan_epoch=2, cadence_reset=1),
            sample(4, 2_050_000_000, plan_epoch=2),
        ])
        result = report["backends"][0]["cadence"]
        self.assertEqual(result["status"], "VERIFIED")
        self.assertEqual(result["epochs"], 2)
        self.assertEqual(result["checked_intervals"], 2)
        self.assertEqual(len(result["reset_short_intervals"]), 1)
        self.assertEqual(result["reset_short_intervals"][0]["interval_ms"], 50)
        self.assertEqual(result["reset_cycles"], 1)
        self.assertIn("eccezioni tra epoche", timing.render_text(report))

    def test_only_first_call_in_each_epoch_cannot_pass_limit_check(self):
        report = self.report([
            sample(1, 1_000_000_000), sample(2, 1_050_000_000, plan_epoch=2),
        ])
        result = report["backends"][0]["cadence"]
        self.assertEqual(result["status"], "INSUFFICIENT_DATA")
        self.assertEqual(len(result["reset_short_intervals"]), 1)

    def test_zero_or_one_attempt_is_insufficient_data(self):
        for rows in ([sample(1)], [sample(1, 1_000_000_000)]):
            with self.subTest(calls=sum(row["plan_called"] for row in rows)):
                result = self.report(rows)["backends"][0]["cadence"]
                self.assertEqual(result["status"], "INSUFFICIENT_DATA")
                self.assertIsNone(result["intervals"]["observed_hz"])

    def test_legacy_csv_keeps_stats_but_cannot_verify_cadence(self):
        report = self.report([
            sample(1, 1_000_000_000), sample(2, 1_500_000_000),
        ], legacy=True)
        backend = report["backends"][0]
        self.assertEqual(backend["metrics_ms"]["plan_ms"]["mean"], 100)
        self.assertEqual(backend["cadence"]["status"], "UNVERIFIABLE")
        self.assertEqual(backend["cadence"]["missing_start_timestamps"], 2)
        self.assertIn("NON VERIFICABILE", timing.render_text(report))

    def test_previous_thirty_column_csv_preserves_cadence_without_async_metadata(self):
        report = self.report([
            sample(1, 1_000_000_000), sample(2, 1_500_000_000),
        ], async_columns=False)
        backend = report["backends"][0]
        self.assertEqual(backend["cadence"]["status"], "VERIFIED")
        self.assertEqual(backend["async_cycles"], 0)
        self.assertEqual(backend["in_flight_cycles"], 0)
        self.assertEqual(backend["discarded_completions"], 0)
        self.assertEqual(backend["plan_mode_metrics_ms"]["synchronous"]["count"], 2)
        self.assertEqual(report["files"][0]["missing_async_columns"], sorted(timing.ASYNC_COLUMNS))

    def test_individual_async_columns_are_optional(self):
        header = timing.LEGACY_COLUMNS + timing.CADENCE_COLUMNS + ("plan_async",)
        with self.path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=header, extrasaction="ignore")
            writer.writeheader()
            writer.writerow(sample(1, 1_000_000_000, plan_async=1, plan_ms=500))
        report = timing.analyze([self.path])
        backend = report["backends"][0]
        self.assertEqual(backend["async_completions"], 1)
        self.assertEqual(backend["in_flight_cycles"], 0)
        self.assertEqual(backend["discarded_completions"], 0)
        self.assertEqual(report["files"][0]["missing_async_columns"],
                         ["plan_discarded", "plan_running"])

    def test_zero_start_timestamp_does_not_yield_false_verification(self):
        report = self.report([
            sample(1, 1_000_000_000), sample(2, 1_500_000_000),
            sample(3, 0),
        ])
        result = report["backends"][0]["cadence"]
        self.assertEqual(result["status"], "UNVERIFIABLE")
        self.assertEqual(result["checked_intervals"], 1)

    def test_unmeasured_attempt_breaks_interval_adjacency(self):
        result = self.report([
            sample(1, 1_000_000_000), sample(2, 0), sample(3, 1_500_000_000),
        ])["backends"][0]["cadence"]
        self.assertEqual(result["status"], "UNVERIFIABLE")
        self.assertEqual(result["checked_intervals"], 0)

    def test_large_nanosecond_timestamps_keep_boundary_precision(self):
        start = 123_456_789_123_456_789
        result = self.report([
            sample(1, start), sample(2, start + 499_999_998),
        ])["backends"][0]["cadence"]
        self.assertEqual(result["status"], "VIOLATION")
        self.assertEqual(len(result["violations"]), 1)

    def test_backends_sessions_and_files_never_form_cross_scope_intervals(self):
        first = self.write([
            sample(1, 1_000_000_000),
            sample(2, 1_050_000_000, backend="DIJKSTRA", replan_hz=0),
            sample(3, 1_500_000_000),
            sample(4, 1_550_000_000, session=2),
        ])
        other = self.write([
            sample(1, 1_600_000_000), sample(2, 2_100_000_000),
        ], path=Path(self.directory.name) / "other.csv")
        report = timing.analyze([first, other])
        backends = {entry["backend"]: entry for entry in report["backends"]}
        self.assertEqual(backends["SMAC"]["cadence"]["checked_intervals"], 2)
        self.assertEqual(backends["SMAC"]["cadence"]["status"], "VERIFIED")
        self.assertEqual(backends["DIJKSTRA"]["cadence"]["status"], "NOT_LIMITED")
        self.assertEqual(len(report["groups"]), 4)

    def test_configurable_rate_is_used_instead_of_hard_coded_two_hz(self):
        result = self.report([
            sample(1, 1_000_000_000, replan_hz=4),
            sample(2, 1_250_000_000, replan_hz=4),
        ])["backends"][0]["cadence"]
        self.assertEqual(result["status"], "VERIFIED")
        self.assertEqual(result["configured_hz"], [4])

    def test_unterminated_last_record_is_ignored_even_if_numeric_prefix_is_valid(self):
        self.write([sample(1, 1_000_000_000)])
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write("1,2,123,456,SMAC,1,0,1")
        report = timing.analyze([self.path])
        self.assertEqual(report["files"][0]["incomplete_rows_skipped"], 1)
        self.assertEqual(report["backends"][0]["cycles"], 1)
        self.assertIn("riga finale incompleta", timing.render_text(report))

    def test_complete_last_record_without_newline_is_conservatively_ignored(self):
        self.write([sample(1), sample(2)])
        self.path.write_text(self.path.read_text().rstrip("\n"), encoding="utf-8")
        report = timing.analyze([self.path])
        self.assertEqual(report["files"][0]["cycles"], 1)
        self.assertEqual(report["files"][0]["incomplete_rows_skipped"], 1)

    def test_malformed_complete_record_fails_with_file_line_and_column(self):
        self.write([sample(1, plan_ms="broken")])
        with self.assertRaisesRegex(timing.TimingDataError, r"mpc_timing_test.csv:2: plan_ms='broken'"):
            timing.analyze([self.path])

    def test_nonfinite_timing_and_invalid_boolean_are_rejected(self):
        for changes in ({"cpu_ms": "nan"}, {"success": 2}, {"plan_running": 2}):
            with self.subTest(changes=changes):
                self.write([sample(1, **changes)])
                with self.assertRaises(timing.TimingDataError):
                    timing.analyze([self.path])

    def test_completed_short_record_is_not_silently_ignored(self):
        self.write([sample(1)])
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write("1,2,3\n")
        with self.assertRaisesRegex(timing.TimingDataError, "attese 33 colonne, trovate 3"):
            timing.analyze([self.path])

    def test_missing_header_field_is_an_actionable_error(self):
        self.path.write_text("session,cycle\n1,1\n", encoding="utf-8")
        with self.assertRaisesRegex(timing.TimingDataError, "mancano colonne obbligatorie"):
            timing.analyze([self.path])

    def test_backwards_same_epoch_start_timestamp_is_rejected(self):
        self.write([sample(1, 2_000_000_000), sample(2, 1_000_000_000)])
        with self.assertRaisesRegex(timing.TimingDataError, "torna indietro"):
            timing.analyze([self.path])

    def test_latest_cli_and_json_use_modification_time_and_deduplicate_paths(self):
        old = self.write([sample(1)])
        new = self.write([sample(1, 1_000_000_000), sample(2, 1_500_000_000)],
                         path=Path(self.directory.name) / "mpc_timing_new.csv")
        os.utime(old, ns=(1_000_000_000, 1_000_000_000))
        os.utime(new, ns=(2_000_000_000, 2_000_000_000))
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(timing.main([str(new), "--latest", "--directory",
                                          self.directory.name, "--json"]), 0)
        report = json.loads(output.getvalue())
        self.assertEqual(len(report["files"]), 1)
        self.assertEqual(report["files"][0]["path"], str(new.resolve()))
        self.assertEqual(report["backends"][0]["cadence"]["status"], "VERIFIED")

    def test_empty_latest_directory_returns_clear_cli_error(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as caught:
            timing.main(["--latest", "--directory", self.directory.name])
        self.assertEqual(caught.exception.code, 2)
        self.assertIn("nessun mpc_timing_*.csv", stderr.getvalue())

    def test_empty_csv_body_is_allowed_without_false_pass(self):
        report = self.report([])
        self.assertEqual(report["files"][0]["cycles"], 0)
        self.assertEqual(report["backends"], [])
        self.assertNotIn("RISPETTATO", timing.render_text(report))


if __name__ == "__main__":
    unittest.main()
