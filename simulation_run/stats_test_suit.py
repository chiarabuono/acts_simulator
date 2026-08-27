"""
Runs paired statistical comparisons (Friedman/Wilcoxon for continuous metrics, Cochran's Q/McNemar for binary) 
across simulation configurations from results.xlsx + rubbing/instability logs, writing a full report to statistical_analysis_report.txt.

Adapt: ALPHA for significance threshold, 
"""

from datetime import datetime
import os
from pathlib import Path
import re
import sys
import numpy as np
import pandas as pd
from scipy import stats

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SIMULATOR_PKG = os.path.dirname(_SCRIPT_DIR)
_SRC_DIR = os.path.dirname(_SIMULATOR_PKG)

for p in [_SIMULATOR_PKG, _SRC_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

from acts_simulator.utils_configuration_selection import select_and_load_folder

# ============================== CONFIG ======================================
ALPHA = 0.05

BASE_DIR = Path(__file__).resolve().parent

folder_name, folder_path = select_and_load_folder("simulation_run")

if not folder_path:
    print("No folder selected. Exiting...")
    sys.exit(0)

DATA_DIR = Path(folder_path)
RESULTS_XLSX = DATA_DIR / "results.xlsx"
RUBBING_XLSX = DATA_DIR / "ground_cable_rubbing_log.xlsx"
PARALLEL_ERRORS_XLSX = DATA_DIR / "parallel_matched_errors.xlsx"
MUJOCO_LOG = DATA_DIR / "MUJOCO_LOG.TXT"
OUTPUT_REPORT_TXT = DATA_DIR / "statistical_analysis_report.txt"

ACTS_DIR = BASE_DIR / "acts"
ACTS_RESULTS_XLSX = ACTS_DIR / "results.xlsx"
ACTS_RUBBING_XLSX = ACTS_DIR / "ground_cable_rubbing_log.xlsx"
ACTS_PARALLEL_ERRORS_XLSX = ACTS_DIR / "parallel_matched_errors.xlsx"

CONFIG_COL = ("config")
RUBBING_CONFIG_COL = "model_xml"
POSE_COLS = ["pos_x", "pos_y", "pos_z"]

PRIMARY_CONTINUOUS = ["composite_score"]
PRIMARY_BINARY = ["pose_reached", "orientation_reached", "cable_rubbed"]

DIAGNOSTIC_CONTINUOUS = [
    "conditioning_index",
    "manipulability",
    "radius_available_force",
    "capacity_margin",
    "worst_case_capacity_margin",
    "position_error",
    "orientation_error",
]
DIAGNOSTIC_BINARY = ["simulation_unstable"]

class TeeStream:
    def __init__(self, file_path):
        self.file = open(file_path, "w", encoding="utf-8")
        self.stdout = sys.stdout

    def write(self, data):
        self.stdout.write(data)
        self.file.write(data)

    def flush(self):
        self.stdout.flush()
        self.file.flush()

    def close(self):
        self.file.close()


def parse_mujoco_log(log_path):
    """Parses MUJOCO_LOG.TXT and extracts timestamped instability errors."""
    if not os.path.exists(log_path):
        return pd.DataFrame()

    mujoco_events = []
    current_timestamp = None
    current_time_str = None

    with open(log_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            ts_match = re.match(
                r"^[A-Z][a-z]{2}\s+[A-Z][a-z]{2}\s+\d+\s+(\d{2}:\d{2}:\d{2})\s+\d{4}",
                line,
            )
            if ts_match:
                current_time_str = ts_match.group(1)
                current_timestamp = datetime.strptime(
                    current_time_str, "%H:%M:%S"
                )
                continue

            if line.startswith("WARNING:") and current_timestamp:
                sim_time_match = re.search(r"Time\s*=\s*(\d+(?:\.\d+)?)", line)
                sim_time = (
                    float(sim_time_match.group(1)) if sim_time_match else None
                )

                mujoco_events.append({
                    "wall_clock_time": current_time_str,
                    "wall_clock_dt": current_timestamp,
                    "warning_message": line,
                    "mujoco_sim_time": sim_time,
                })
    return pd.DataFrame(mujoco_events)


def load_parallel_matched_errors(matched_file: Path) -> pd.DataFrame:
    """Safely loads parallel_matched_errors (.xlsx or .csv) if present.
    Logs an informative message if absent instead of raising an exception.
    """
    csv_path = matched_file.with_suffix(".csv")

    target_file = None
    if matched_file.exists():
        target_file = matched_file
    elif csv_path.exists():
        target_file = csv_path

    if target_file is None:
        print(f"[INFO] '{matched_file.name}' not found in {matched_file.parent}. Skipping precalculated match.")
        return pd.DataFrame()

    try:
        if target_file.suffix.lower() == ".csv":
            df = pd.read_csv(target_file)
        else:
            df = pd.read_excel(target_file)
        print(f"[INFO] Successfully loaded '{target_file.name}' from {target_file.parent}")
        return df
    except Exception as e:
        print(f"[WARNING] Could not read '{target_file.name}': {e}. Continuing without it.")
        return pd.DataFrame()


def add_instability_features(
    df: pd.DataFrame,
    rub_df: pd.DataFrame,
    log_path: Path,
    config_col: str,
    matched_errors_df: pd.DataFrame = None,
) -> pd.DataFrame:
    """Disambiguates parallel MuJoCo warnings and tags unstable poses in df.
    
    Uses precalculated parallel_matched_errors if provided, otherwise parses MUJOCO_LOG.
    """
    df = df.copy()
    if "simulation_unstable" not in df.columns:
        df["simulation_unstable"] = False

    unstable_config_poses = set()

    # Priority 1: Use precalculated parallel_matched_errors table
    if matched_errors_df is not None and not matched_errors_df.empty:
        print(" -> Processing instability features using precalculated parallel errors table...")
        for _, row in matched_errors_df.iterrows():
            cfg_name = row.get("matched_model_xml", row.get("matched_config", None))
            pose_idx = row.get("matched_pose_index", None)

            # Skip unmatched rows or invalid entries
            if pd.notna(cfg_name) and str(cfg_name).strip() != "N/A" and pd.notna(pose_idx) and str(pose_idx).strip() != "N/A":
                try:
                    unstable_config_poses.add((str(cfg_name), int(pose_idx)))
                except (ValueError, TypeError):
                    continue
    else:
        # Priority 2: Fallback to on-the-fly parsing of MUJOCO_LOG and rubbing log
        print(" -> Precalculated parallel errors table not available. Falling back to MUJOCO_LOG parsing...")
        df_errors = parse_mujoco_log(log_path)
        if not df_errors.empty and rub_df is not None and not rub_df.empty:
            rub_df = rub_df.copy()
            rub_df["wall_clock_dt"] = pd.to_datetime(
                rub_df["time"].astype(str), format="%H:%M:%S"
            )

            for _, err in df_errors.iterrows():
                err_wall_time = err["wall_clock_dt"]
                err_sim_time = err["mujoco_sim_time"]

                time_window = rub_df[
                    (rub_df["wall_clock_dt"] - err_wall_time).abs()
                    <= pd.Timedelta(seconds=30)
                ].copy()

                if time_window.empty:
                    time_window = rub_df.copy()

                if (
                    err_sim_time is not None
                    and "simulation_time_s" in time_window.columns
                ):
                    time_window["sim_time_diff"] = (
                        time_window["simulation_time_s"] - err_sim_time
                    ).abs()
                    best_match_idx = time_window["sim_time_diff"].idxmin()
                    best_match = time_window.loc[best_match_idx]

                    cfg_name = best_match.get("model_xml", best_match.get(config_col))
                    pose_idx = best_match.get("pose_index")

                    if pd.notna(cfg_name) and pd.notna(pose_idx):
                        unstable_config_poses.add((str(cfg_name), int(pose_idx)))

    # Flag corresponding rows in main df
    for cfg, pose_idx in unstable_config_poses:
        mask = (df[config_col].astype(str) == str(cfg)) & (df["pose_index"] == pose_idx)
        df.loc[mask, "simulation_unstable"] = True

    return df

def check_pairing(df: pd.DataFrame, config_col: str, pose_cols: list) -> bool:
    configs = df[config_col].unique()
    ref = df[df[config_col] == configs[0]][pose_cols].reset_index(drop=True)
    ok = True
    for c in configs[1:]:
        cur = df[df[config_col] == c][pose_cols].reset_index(drop=True)
        same = len(cur) == len(ref) and (cur.values == ref.values).all()
        if not same:
            ok = False
            print(
                f"  WARNING: '{c}' does not match pose order/values of '{configs[0]}'."
            )
    return ok


def holm_correction(pvals: dict) -> dict:
    items = sorted(pvals.items(), key=lambda kv: kv[1])
    m = len(items)
    adjusted = {}
    running_max = 0.0
    for i, (label, p) in enumerate(items):
        adj = min((m - i) * p, 1.0)
        running_max = max(running_max, adj)
        adjusted[label] = running_max
    return adjusted


def bh_fdr_correction(pvals: dict) -> dict:
    items = sorted(pvals.items(), key=lambda kv: kv[1])
    m = len(items)
    raw_adj = [min(p * m / (i + 1), 1.0) for i, (_, p) in enumerate(items)]
    adj = [0.0] * m
    running_min = 1.0
    for i in range(m - 1, -1, -1):
        running_min = min(running_min, raw_adj[i])
        adj[i] = running_min
    return {items[i][0]: adj[i] for i in range(m)}


def add_pose_index(df: pd.DataFrame, config_col: str) -> pd.DataFrame:
    df = df.copy()
    df["pose_index"] = df.groupby(config_col).cumcount() + 1
    return df


def add_cable_features(df: pd.DataFrame, rub: pd.DataFrame, config_col: str, rubbing_config_col: str) -> pd.DataFrame:
    df = add_pose_index(df, config_col)
    if rub is not None and not rub.empty:
        if "pose_index" not in rub.columns:
            rub = add_pose_index(rub, rubbing_config_col)

        agg = (rub.groupby([rubbing_config_col, "pose_index"])["violation_margin_m"].min().reset_index().rename(columns={rubbing_config_col: config_col, "violation_margin_m": "cable_severity"}))
        df = df.merge(agg, on=[config_col, "pose_index"], how="left")
    else:
        df["cable_severity"] = np.nan

    df["cable_rubbed"] = df["cable_severity"].notna()
    return df


def clean_boolean_series(series: pd.Series) -> pd.Series:
    if series.dtype == bool:
        return series
    return (series.astype(str) .str.strip() .str.upper() .map({"TRUE": True, "1": True, "1.0": True}) .fillna(False))


def continuous_pivot(df, config_col, metric):
    configs = list(df[config_col].unique())
    return pd.DataFrame({c: df[df[config_col] == c][metric].reset_index(drop=True) for c in configs})


def binary_pivot(df, config_col, metric):
    configs = list(df[config_col].unique())
    return pd.DataFrame({
        c: clean_boolean_series(df[df[config_col] == c][metric]).reset_index(drop=True)
        for c in configs
    })


def friedman_omnibus(pv):
    pv_clean = pv.dropna()
    if len(pv_clean) < 2:
        return np.nan, np.nan
    return stats.friedmanchisquare(*[pv_clean[c].values for c in pv_clean.columns])


def cochran_omnibus(pv: pd.DataFrame):
    from statsmodels.stats.contingency_tables import cochrans_q
    res = cochrans_q(pv.values)
    return res.statistic, res.pvalue


def wilcoxon_pairwise_vs_reference(pv: pd.DataFrame, reference: str):
    configs = list(pv.columns)
    raw_p, diagnostics = {}, {}
    for c in configs:
        if c == reference:
            continue
        a, b = pv[reference].dropna(), pv[c].dropna()
        diff = (pv[reference] - pv[c]).dropna()
        if len(diff) > 0 and np.allclose(diff, 0):
            w_stat, w_p, t_p, shapiro_p = np.nan, 1.0, 1.0, np.nan
        else:
            try:
                w_stat, w_p = stats.wilcoxon(a, b)
            except ValueError:
                w_stat, w_p = np.nan, 1.0
            t_stat, t_p = stats.ttest_rel(a, b)
            shapiro_p = (stats.shapiro(diff).pvalue if len(diff) >= 3 and diff.std() > 0 else np.nan)
        raw_p[c] = w_p
        diagnostics[c] = dict(mean_diff=diff.mean(), w_stat=w_stat, w_p=w_p, t_p=t_p, shapiro_p=shapiro_p, n=len(diff))
    return holm_correction(raw_p), diagnostics


def mcnemar_pairwise_vs_reference(pv: pd.DataFrame, reference: str):
    from statsmodels.stats.contingency_tables import mcnemar

    configs = list(pv.columns)
    raw_p = {}
    for c in configs:
        if c == reference:
            continue
        tab = pd.crosstab(pv[reference], pv[c])
        tab = tab.reindex(
            index=[False, True], columns=[False, True], fill_value=0
        )
        res = mcnemar(tab, exact=True)
        raw_p[c] = res.pvalue
    return holm_correction(raw_p), raw_p


def print_pairwise_continuous(reference, corrected, diagnostics):
    for c, adj_p in corrected.items():
        d = diagnostics[c]
        sig = "SIGNIFICANT" if adj_p < ALPHA else "not significant"
        print(f"  {reference} vs {c}  (n={d['n']}): mean diff = {d['mean_diff']:+.4f} | Wilcoxon p={d['w_p']:.4f} (Holm p={adj_p:.4f}) -> {sig}")


def print_pairwise_binary(reference, corrected, raw_p):
    for c, adj_p in corrected.items():
        sig = "SIGNIFICANT" if adj_p < ALPHA else "not significant"
        print(f"  {reference} vs {c}: McNemar p={raw_p[c]:.4f} (Holm-adjusted p={adj_p:.4f}) -> {sig}")


def run_metric_group(group_name, df, config_col, continuous_metrics, binary_metrics, reference):
    print("\n" + "#" * 80)
    print(f"# {group_name.upper()} METRICS")
    print("#" * 80)

    pivots, omnibus_results = {}, {}

    for metric in continuous_metrics:
        pv = continuous_pivot(df, config_col, metric)
        pivots[metric] = ("continuous", pv)
        stat, p = friedman_omnibus(pv)
        omnibus_results[metric] = ("continuous", stat, p)

    for metric in binary_metrics:
        pv = binary_pivot(df, config_col, metric)
        pivots[metric] = ("binary", pv)
        stat, p = cochran_omnibus(pv)
        omnibus_results[metric] = ("binary", stat, p)

    raw_p = {m: v[2] for m, v in omnibus_results.items()}
    fdr_p = bh_fdr_correction(raw_p)

    print(
        f"\nOmnibus tests, BH-FDR corrected across {len(raw_p)} {group_name} metrics:"
    )
    for m in sorted(raw_p, key=lambda m: fdr_p[m]):
        kind, stat, p = omnibus_results[m]
        sig = "SIGNIFICANT" if fdr_p[m] < ALPHA else "not significant"
        print(
            f"  {m:30s} {kind:11s} stat={stat:8.3f} raw_p={p:9.5f} FDR_p={fdr_p[m]:9.5f} -> {sig}"
        )

    for metric in list(continuous_metrics) + list(binary_metrics):
        kind, pv = pivots[metric]
        means_or_rates = pv.mean().sort_values(ascending=False)
        print(f"\n--- {metric} ({kind}) ---")
        for c in means_or_rates.index:
            print(f"  {str(c):65s} value={means_or_rates[c]:8.4f}")

        if fdr_p[metric] >= ALPHA:
            continue

        ref_here = (
            reference
            if reference in pv.columns
            else (means_or_rates.index[0] if reference is None else None)
        )
        if ref_here:
            print(f"  Pairwise tests vs reference = '{ref_here}':")
            if kind == "continuous":
                corr, diag = wilcoxon_pairwise_vs_reference(pv, ref_here)
                print_pairwise_continuous(ref_here, corr, diag)
            else:
                corr, raw = mcnemar_pairwise_vs_reference(pv, ref_here)
                print_pairwise_binary(ref_here, corr, raw)


def load_data():
    if not RESULTS_XLSX.exists():
        print(f"[ERROR] Could not find 'results.xlsx' in: {DATA_DIR}")
        sys.exit(1)

    df = pd.read_excel(RESULTS_XLSX)
    rub = pd.read_excel(RUBBING_XLSX) if RUBBING_XLSX.exists() else pd.DataFrame()
    matched_errors = load_parallel_matched_errors(PARALLEL_ERRORS_XLSX)

    ref_name = None
    if ACTS_RESULTS_XLSX.exists():
        acts_df = pd.read_excel(ACTS_RESULTS_XLSX)
        ref_name = acts_df[CONFIG_COL].unique()[0]
        df = pd.concat([df, acts_df], ignore_index=True)

    if ACTS_RUBBING_XLSX.exists():
        acts_rub = pd.read_excel(ACTS_RUBBING_XLSX)
        rub = pd.concat([rub, acts_rub], ignore_index=True)

    acts_matched_errors = load_parallel_matched_errors(ACTS_PARALLEL_ERRORS_XLSX)
    if not acts_matched_errors.empty:
        matched_errors = pd.concat([matched_errors, acts_matched_errors], ignore_index=True)

    return df, rub, matched_errors, ref_name


def main():
    logger = TeeStream(OUTPUT_REPORT_TXT)
    sys.stdout = logger

    try:
        print("=" * 80)
        print("Loading data and parsing MuJoCo instability logs")
        print("=" * 80)
        df, rub, matched_errors, reference_name = load_data()

        df = add_cable_features(df, rub, CONFIG_COL, RUBBING_CONFIG_COL)
        df = add_instability_features(df, rub, MUJOCO_LOG, CONFIG_COL, matched_errors)

        print("\n" + "=" * 80)
        print("Checking configuration pose pairing")
        print("=" * 80)
        check_pairing(df, CONFIG_COL, POSE_COLS)

        run_metric_group("primary", df, CONFIG_COL, PRIMARY_CONTINUOUS, PRIMARY_BINARY, reference_name)
        run_metric_group("diagnostic", df, CONFIG_COL, DIAGNOSTIC_CONTINUOUS, DIAGNOSTIC_BINARY, reference_name)

        print("\n" + "=" * 80)
        print(f"Report saved to: {OUTPUT_REPORT_TXT}")
        print("=" * 80)

    finally:
        sys.stdout = logger.stdout
        logger.close()


if __name__ == "__main__":
    main()