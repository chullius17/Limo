#!/usr/bin/env python3
"""Summarize MPC timing CSVs without ROS or third-party dependencies.

Cadence is measured between actual planner *start* timestamps. The CSV cycle
timestamp is deliberately not used: a long planning call would distort it.
Async calls are consumed worker completions; their plan_ms is background
elapsed time and does not form part of the reporting controller cycle.
The logger terminates every complete record with a newline, so an unfinished
last record is ignored when inspecting a CSV which is still being written.
"""

import argparse
import csv
import io
import json
import math
from pathlib import Path
import statistics
import sys


LEGACY_COLUMNS = (
    "session", "cycle", "ros_time_ns", "steady_time_ns", "backend",
    "plan_called", "plan_failed", "detouring", "success", "budget_ms",
    "deadline_missed", "grid_updated", "grid_ms", "plan_ms", "cycle_ms",
    "cpu_ms", "prepare_ms", "solve_ms", "scoring_ms", "debug_ms",
    "generated", "evaluated", "rejected",
)
CADENCE_COLUMNS = (
    "plan_start_steady_ns", "plan_epoch", "replan_hz", "plan_interval_valid",
    "plan_interval_ms", "rate_limit_violation", "cadence_reset",
)
ASYNC_COLUMNS = ("plan_async", "plan_running", "plan_discarded")
INTEGER_COLUMNS = {
    "session", "cycle", "ros_time_ns", "steady_time_ns", "generated",
    "evaluated", "rejected", "plan_start_steady_ns", "plan_epoch",
}
BOOLEAN_COLUMNS = {
    "plan_called", "plan_failed", "detouring", "success", "deadline_missed",
    "grid_updated", "plan_interval_valid", "rate_limit_violation", "cadence_reset",
    "plan_async", "plan_running", "plan_discarded",
}
METRICS = (
    "cycle_ms", "cpu_ms", "prepare_ms", "grid_ms", "plan_ms", "solve_ms",
    "scoring_ms", "debug_ms",
)
CADENCE_REQUIRED = {"plan_start_steady_ns", "plan_epoch", "replan_hz"}


class TimingDataError(ValueError):
    """An input file cannot be interpreted safely."""


def metric(values):
    """Return a nearest-rank p95, alongside ordinary descriptive statistics."""
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": len(ordered),
        "mean": statistics.mean(ordered),
        "median": statistics.median(ordered),
        "p95": ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)],
        "max": ordered[-1],
    }


def read_csv(path):
    """Read completed records; preserve exact integer nanosecond timestamps."""
    path = Path(path).expanduser().resolve()
    try:
        contents = path.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError) as error:
        raise TimingDataError("{}: {}".format(path, error)) from error
    lines = contents.splitlines(keepends=True)
    skipped = 0
    if lines and not lines[-1].endswith(("\n", "\r")):
        # Never interpret a partially flushed final numeric field as a full row.
        if len(lines) > 1:
            lines.pop()
            skipped = 1
    if not lines:
        raise TimingDataError("{}: file vuoto, manca l'intestazione CSV".format(path))
    reader = csv.reader(io.StringIO("".join(lines)), strict=True)
    try:
        header = next(reader)
        if len(header) != len(set(header)):
            raise TimingDataError("{}: colonne duplicate nell'intestazione".format(path))
        missing = set(LEGACY_COLUMNS) - set(header)
        if missing:
            raise TimingDataError("{}: mancano colonne obbligatorie: {}".format(
                path, ", ".join(sorted(missing))))
        rows = []
        for fields in reader:
            line = reader.line_num
            if len(fields) != len(header):
                raise TimingDataError("{}:{}: attese {} colonne, trovate {}".format(
                    path, line, len(header), len(fields)))
            row = {"_file": str(path), "_line": line}
            for name, raw in zip(header, fields):
                if name not in LEGACY_COLUMNS + CADENCE_COLUMNS + ASYNC_COLUMNS:
                    continue
                try:
                    if name == "backend":
                        value = raw.upper()
                        if value not in ("SMAC", "DIJKSTRA"):
                            raise ValueError("backend atteso: SMAC o DIJKSTRA")
                    elif name in BOOLEAN_COLUMNS:
                        if raw not in ("0", "1"):
                            raise ValueError("booleano atteso: 0 o 1")
                        value = raw == "1"
                    elif name in INTEGER_COLUMNS:
                        value = int(raw)
                        if value < 0 and name != "ros_time_ns":
                            raise ValueError("intero negativo")
                    else:
                        value = float(raw)
                        if not math.isfinite(value) or value < 0:
                            raise ValueError("numero non finito o negativo")
                    if name == "budget_ms" and value <= 0:
                        raise ValueError("budget deve essere positivo")
                except ValueError as error:
                    raise TimingDataError("{}:{}: {}={!r}: {}".format(
                        path, line, name, raw, error)) from error
                row[name] = value
            rows.append(row)
    except csv.Error as error:
        raise TimingDataError("{}:{}: CSV non valido: {}".format(
            path, reader.line_num, error)) from error
    return rows, {
        "path": str(path), "cycles": len(rows),
        "incomplete_rows_skipped": skipped,
        "missing_cadence_columns": sorted(set(CADENCE_COLUMNS) - set(header)),
        "missing_async_columns": sorted(set(ASYNC_COLUMNS) - set(header)),
    }


def cadence(rows):
    """Independently check planner starts within the same file/session/epoch."""
    intervals = []
    checked = []
    violations = []
    reset_short = []
    missing_starts = 0
    logged_violations = 0
    flag_disagreements = 0
    starts = {}
    previous = {}
    epochs = set()
    rates = set()
    metadata_complete = all(CADENCE_REQUIRED <= row.keys() for row in rows)
    for row in rows:
        rate = row.get("replan_hz", 0.0)
        if rate > 0:
            rates.add(rate)
        logged_violations += int(row.get("rate_limit_violation", False))
        if not row["plan_called"]:
            continue
        scope = (row["_file"], row["session"], row["backend"])
        if not CADENCE_REQUIRED <= row.keys() or row["plan_start_steady_ns"] <= 0:
            missing_starts += 1
            # An unmeasured search breaks adjacency: never bridge over it.
            for key in [key for key in starts if key[:3] == scope]:
                del starts[key]
            previous.pop(scope, None)
            continue
        epoch_key = scope + (row["plan_epoch"],)
        epochs.add(epoch_key)
        start = row["plan_start_steady_ns"]
        old = starts.get(epoch_key)
        limited = row["backend"] == "SMAC" and rate > 0
        required_ns = 1_000_000_000.0 / rate if limited else None
        if old is not None:
            delta_ns = start - old["plan_start_steady_ns"]
            if delta_ns < 0:
                raise TimingDataError("{}:{}: plan_start_steady_ns torna indietro "
                                      "nella stessa epoca".format(row["_file"], row["_line"]))
            interval = {
                "file": row["_file"], "session": row["session"],
                "backend": row["backend"], "epoch": row["plan_epoch"],
                "previous_cycle": old["cycle"], "cycle": row["cycle"],
                "interval_ms": delta_ns / 1_000_000.0,
                "required_ms": required_ns / 1_000_000.0 if limited else None,
            }
            intervals.append(interval)
            if limited:
                checked.append(interval)
                # Accommodate at most one nanosecond of timestamp quantization.
                violation = delta_ns + 1 < required_ns
                if violation:
                    violations.append(interval)
                if "rate_limit_violation" in row:
                    flag_disagreements += violation != row["rate_limit_violation"]
        old_global = previous.get(scope)
        if old_global is not None and old_global["plan_epoch"] != row["plan_epoch"]:
            delta_ns = start - old_global["plan_start_steady_ns"]
            if limited and delta_ns + 1 < required_ns:
                reset_short.append({
                    "file": row["_file"], "session": row["session"],
                    "previous_epoch": old_global["plan_epoch"], "epoch": row["plan_epoch"],
                    "previous_cycle": old_global["cycle"], "cycle": row["cycle"],
                    "interval_ms": delta_ns / 1_000_000.0,
                    "required_ms": required_ns / 1_000_000.0,
                })
        starts[epoch_key] = row
        previous[scope] = row
    if violations:
        status = "VIOLATION"
    elif not metadata_complete or missing_starts:
        status = "UNVERIFIABLE"
    elif not any(row["backend"] == "SMAC" and row.get("replan_hz", 0) > 0 for row in rows):
        status = "NOT_LIMITED"
    elif not checked:
        status = "INSUFFICIENT_DATA"
    else:
        status = "VERIFIED"
    values = [entry["interval_ms"] for entry in intervals]
    interval_stats = metric(values)
    interval_stats["min"] = min(values) if values else None
    interval_stats["observed_hz"] = (
        1000.0 * len(values) / sum(values) if values and sum(values) > 0 else None)
    return {
        "status": status, "configured_hz": sorted(rates), "epochs": len(epochs),
        "metadata_complete": metadata_complete,
        "missing_start_timestamps": missing_starts,
        "checked_intervals": len(checked), "intervals": interval_stats,
        "violations": violations, "logged_violations": logged_violations,
        "flag_disagreements": flag_disagreements,
        "reset_short_intervals": reset_short,
        "reset_cycles": sum(row.get("cadence_reset", False) for row in rows),
    }


def summarize(rows):
    times = {}
    for name in METRICS:
        selected = rows
        if name == "plan_ms":
            selected = [row for row in rows if row["plan_called"]]
        elif name == "grid_ms":
            selected = [row for row in rows if row["grid_updated"]]
        times[name] = metric(row[name] for row in selected)
    return {
        "cycles": len(rows),
        "attempts": sum(row["plan_called"] for row in rows),
        "plan_failures": sum(row["plan_called"] and row["plan_failed"] for row in rows),
        "async_cycles": sum(row.get("plan_async", False) for row in rows),
        "in_flight_cycles": sum(row.get("plan_async", False) and
                                row.get("plan_running", False) for row in rows),
        "async_completions": sum(row["plan_called"] and row.get("plan_async", False)
                                 for row in rows),
        "discarded_completions": sum(row["plan_called"] and row.get("plan_discarded", False)
                                     for row in rows),
        "plan_mode_metrics_ms": {
            "synchronous": metric(row["plan_ms"] for row in rows
                                  if row["plan_called"] and not row.get("plan_async", False)),
            "async_worker": metric(row["plan_ms"] for row in rows
                                   if row["plan_called"] and row.get("plan_async", False)),
        },
        "failed_cycles": sum(not row["success"] for row in rows),
        "detour_cycles": sum(row["detouring"] for row in rows),
        "cached_cycles": sum(row["detouring"] and row["success"] and
                             not row["plan_called"] for row in rows),
        "waiting_cycles": sum(row["backend"] == "SMAC" and row["grid_updated"] and
                              not row["plan_called"] and not row["success"] and
                              not row["detouring"] for row in rows),
        "deadline_misses": sum(row["deadline_missed"] for row in rows),
        "metrics_ms": times,
        "cadence": cadence(rows),
    }


def analyze(paths):
    files = []
    all_rows = []
    for path in paths:
        rows, file_info = read_csv(path)
        files.append(file_info)
        all_rows.extend(rows)
    groups = {}
    backends = {}
    for row in all_rows:
        key = (row["_file"], row["session"], row["backend"])
        groups.setdefault(key, []).append(row)
        backends.setdefault(row["backend"], []).append(row)
    return {
        "files": files,
        "backends": [dict(backend=backend, **summarize(rows))
                     for backend, rows in sorted(backends.items())],
        "groups": [dict(file=key[0], session=key[1], backend=key[2], **summarize(rows))
                   for key, rows in sorted(groups.items())],
    }


def number(value):
    return "—" if value is None else "{:.3f}".format(value)


def cadence_text(result):
    labels = {
        "VERIFIED": "RISPETTATO negli intervalli osservati",
        "VIOLATION": "VIOLATO",
        "UNVERIFIABLE": "NON VERIFICABILE: metadati di inizio ricerca mancanti",
        "NOT_LIMITED": "nessun limite di frequenza configurato",
        "INSUFFICIENT_DATA": "NON VERIFICABILE: servono almeno due ricerche nella stessa epoca",
    }
    rates = ", ".join("{:g}".format(rate) for rate in result["configured_hz"]) or "—"
    lines = ["Cadenza: {} (limite Hz: {}; intervalli verificati: {})".format(
        labels[result["status"]], rates, result["checked_intervals"])]
    times = result["intervals"]
    if times["count"]:
        lines.append("  Intervalli tra inizi, ms: min {} | media {} | max {}; Hz osservati {}".format(
            number(times["min"]), number(times["mean"]), number(times["max"]),
            number(times["observed_hz"])))
    lines.append("  Violazioni ricalcolate: {}; flag nel CSV: {}; disaccordi: {}".format(
        len(result["violations"]), result["logged_violations"], result["flag_disagreements"]))
    for entry in result["violations"][:10]:
        lines.append("  VIOLAZIONE sessione {}, epoca {}, ciclo {}→{}: {} ms < {} ms".format(
            entry["session"], entry["epoch"], entry["previous_cycle"], entry["cycle"],
            number(entry["interval_ms"]), number(entry["required_ms"])))
    if result["reset_short_intervals"]:
        lines.append("  Ripartenze dopo reset sotto il periodo: {} (eccezioni tra epoche, "
                     "escluse dal controllo precedente)".format(len(result["reset_short_intervals"])))
        for entry in result["reset_short_intervals"][:10]:
            lines.append("    sessione {}, epoca {}→{}, ciclo {}→{}: {} ms".format(
                entry["session"], entry["previous_epoch"], entry["epoch"],
                entry["previous_cycle"], entry["cycle"], number(entry["interval_ms"])))
    return lines


def render_text(report):
    lines = []
    for file_info in report["files"]:
        lines.append("{}: {} cicli".format(file_info["path"], file_info["cycles"]))
        if file_info["incomplete_rows_skipped"]:
            lines.append("  AVVISO: ignorata 1 riga finale incompleta (file ancora in scrittura).")
        if file_info["missing_cadence_columns"]:
            lines.append("  Metadati cadenza assenti: {}".format(
                ", ".join(file_info["missing_cadence_columns"])))
    for backend in report["backends"]:
        lines.extend(("", "{}: {} cicli, {} ricerche, {} ricerche fallite".format(
            backend["backend"], backend["cycles"], backend["attempts"], backend["plan_failures"])))
        lines.append("  Riuso senza ricerca: {}; attesa SMAC senza percorso: {}; "
                     "cicli falliti: {}; deadline superate: {}".format(
                         backend["cached_cycles"], backend["waiting_cycles"],
                         backend["failed_cycles"], backend["deadline_misses"]))
        if backend["async_cycles"]:
            lines.append("  Async: {} completamenti consumati, {} scartati; "
                         "{} cicli con ricerca in volo su {} cicli async.".format(
                             backend["async_completions"], backend["discarded_completions"],
                             backend["in_flight_cycles"], backend["async_cycles"]))
            lines.append("  plan_ms async misura il worker in background: può superare cycle_ms "
                         "e non fa parte del tempo del ciclo di controllo.")
        lines.append("  Tempi ms                  n       media     mediana         p95         max")
        for name, values in backend["metrics_ms"].items():
            plan_label = "plan (sync/worker async)" if backend["async_cycles"] else "plan (solo ricerche)"
            label = {"plan_ms": plan_label, "grid_ms": "grid (solo update)"}.get(name, name)
            lines.append("  {:<23} {:>5} {:>11} {:>11} {:>11} {:>11}".format(
                label, values["count"], number(values["mean"]), number(values["median"]),
                number(values["p95"]), number(values["max"])))
        if backend["async_cycles"]:
            for mode, values in backend["plan_mode_metrics_ms"].items():
                if not values["count"]:
                    continue
                label = "plan worker async" if mode == "async_worker" else "plan sincrono"
                lines.append("  {:<23} {:>5} {:>11} {:>11} {:>11} {:>11}".format(
                    label, values["count"], number(values["mean"]), number(values["median"]),
                    number(values["p95"]), number(values["max"])))
        lines.extend("  " + line for line in cadence_text(backend["cadence"]))
    if len(report["groups"]) > len(report["backends"]):
        lines.extend(("", "Dettaglio per file e sessione:"))
        for group in report["groups"]:
            lines.append("{} | sessione {} | {} | {} ricerche".format(
                group["file"], group["session"], group["backend"], group["attempts"]))
            lines.extend("  " + line for line in cadence_text(group["cadence"]))
    lines.extend(("", "p95: rango più vicino. 'plan' include tentativi sincroni e completamenti "
                  "async consumati, anche scartati; invii e attese non contano come tentativi. "
                  "'grid' solo aggiornamenti. Cadenza su orologio monotono, "
                  "separata per file/sessione/backend/epoca; tolleranza 1 ns."))
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="*", help="uno o più CSV MPC timing")
    parser.add_argument("--latest", action="store_true", help="includi il CSV più recente")
    parser.add_argument("--directory", default="~/.ros/limo_timing",
                        help="cartella per --latest (default: %(default)s)")
    parser.add_argument("--json", action="store_true", help="stampa il report JSON")
    args = parser.parse_args(argv)
    paths = [Path(path).expanduser().resolve() for path in args.csv]
    if args.latest:
        directory = Path(args.directory).expanduser()
        try:
            candidates = list(directory.glob("mpc_timing_*.csv"))
            if not candidates:
                raise TimingDataError("nessun mpc_timing_*.csv in {}".format(directory))
            paths.append(max(candidates, key=lambda path: (path.stat().st_mtime_ns, path.name)).resolve())
        except OSError as error:
            parser.error(str(error))
        except TimingDataError as error:
            parser.error(str(error))
    if not paths:
        parser.error("specificare almeno un CSV oppure --latest")
    paths = list(dict.fromkeys(paths))
    try:
        report = analyze(paths)
    except TimingDataError as error:
        parser.error(str(error))
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
          if args.json else render_text(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
