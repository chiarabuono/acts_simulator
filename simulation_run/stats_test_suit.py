"""
Statistical test suite for comparing robot grasp/placement configurations.
"""

from pathlib import Path
import os
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

BASE_DIR = Path(__file__).resolve().parent

folder_name, folder_path = select_and_load_folder("simulation_run")

if not folder_path:
    print("No folder selected. Exiting...")
    sys.exit(0)


DATA_DIR = Path(folder_path)
RESULTS_XLSX = DATA_DIR / "results.xlsx"
RUBBING_XLSX = DATA_DIR / "ground_cable_rubbing_log.xlsx"
OUTPUT_REPORT_TXT = DATA_DIR / "statistical_analysis_report.txt"

ACTS_DIR = BASE_DIR / "acts"
ACTS_RESULTS_XLSX = ACTS_DIR / "results.xlsx"
ACTS_RUBBING_XLSX = ACTS_DIR / "ground_cable_rubbing_log.xlsx"

CONFIG_COL = "config"                    # column naming the configuration in results.xlsx
RUBBING_CONFIG_COL = "model_xml"         # column naming the configuration in the rubbing log
POSE_COLS = ["pos_x", "pos_y", "pos_z"]  # columns identifying a pose (must match across configs)

PRIMARY_CONTINUOUS = ["composite_score"]
PRIMARY_BINARY = ["pose_reached", "orientation_reached", "cable_rubbed"]

# Updated metric name: 'radius_available_force' (was 'radius_available_wrench')
DIAGNOSTIC_CONTINUOUS = [
    "conditioning_index",
    "manipulability",
    "radius_available_force",
    "capacity_margin",
    "worst_case_capacity_margin",
    "position_error",
    "orientation_error",
]
DIAGNOSTIC_BINARY = []

ALPHA = 0.05

# =============================================================================


# ------------------------------- Utilities ----------------------------------

class TeeStream:
    """Helper class to write output simultaneously to console and a text file."""
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


def check_pairing(df: pd.DataFrame, config_col: str, pose_cols: list) -> bool:
    """Confirm each configuration was tested on the same ordered sequence of poses."""
    configs = df[config_col].unique()
    ref = df[df[config_col] == configs[0]][pose_cols].reset_index(drop=True)
    ok = True
    for c in configs[1:]:
        cur = df[df[config_col] == c][pose_cols].reset_index(drop=True)
        same = len(cur) == len(ref) and (cur.values == ref.values).all()
        if not same:
            ok = False
            print(f"  WARNING: '{c}' does not match the pose order/values of '{configs[0]}'. "
                  f"Paired tests will NOT be valid for this configuration.")
    return ok


def holm_correction(pvals: dict) -> dict:
    """Holm-Bonferroni step-down correction. Input/output: {label: p-value}."""
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
    """Benjamini-Hochberg FDR correction. Input/output: {label: p-value}."""
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
    """Add a 1-indexed pose_index within each configuration group."""
    df = df.copy()
    df["pose_index"] = df.groupby(config_col).cumcount() + 1
    return df


def add_cable_features(df: pd.DataFrame, rub: pd.DataFrame, config_col: str,
                        rubbing_config_col: str) -> pd.DataFrame:
    """Add pose-level cable_rubbed and cable_severity to df."""
    df = add_pose_index(df, config_col)

    if rub is not None and not rub.empty:
        agg = (rub.groupby([rubbing_config_col, "pose_index"])["violation_margin_m"]
                  .min()
                  .reset_index()
                  .rename(columns={rubbing_config_col: config_col,
                                    "violation_margin_m": "cable_severity"}))
        df = df.merge(agg, on=[config_col, "pose_index"], how="left")
    else:
        df["cable_severity"] = np.nan

    df["cable_rubbed"] = df["cable_severity"].notna()
    return df


def clean_boolean_series(series: pd.Series) -> pd.Series:
    """Safely convert boolean string values ('TRUE', 'FALSE', NaN) into real booleans."""
    if series.dtype == bool:
        return series
    return series.astype(str).str.strip().str.upper().map({'TRUE': True, '1': True, '1.0': True}).fillna(False)


# --------------------------- Omnibus test runners ----------------------------

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
    # Option A: Strict paired complete-case analysis
    # Drop any row (pose) that isn't present across ALL configurations
    pv_clean = pv.dropna()
    
    # Ensure all groups still have identical lengths >= 2
    configs = pv_clean.columns
    arrays = [pv_clean[c].values for c in configs]
    
    if len(pv_clean) < 2:
        print("Warning: Insufficient paired samples for Friedman test.")
        return np.nan, np.nan

    return stats.friedmanchisquare(*arrays)

def cochran_omnibus(pv: pd.DataFrame):
    from statsmodels.stats.contingency_tables import cochrans_q
    res = cochrans_q(pv.values)
    return res.statistic, res.pvalue


# --------------------------- Pairwise test runners ---------------------------

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
            shapiro_p = stats.shapiro(diff).pvalue if len(diff) >= 3 and diff.std() > 0 else np.nan
        raw_p[c] = w_p
        diagnostics[c] = dict(mean_diff=diff.mean(), w_stat=w_stat, w_p=w_p,
                               t_p=t_p, shapiro_p=shapiro_p, n=len(diff))
    corrected = holm_correction(raw_p)
    return corrected, diagnostics


def mcnemar_pairwise_vs_reference(pv: pd.DataFrame, reference: str):
    from statsmodels.stats.contingency_tables import mcnemar
    configs = list(pv.columns)
    raw_p = {}
    for c in configs:
        if c == reference:
            continue
        tab = pd.crosstab(pv[reference], pv[c])
        tab = tab.reindex(index=[False, True], columns=[False, True], fill_value=0)
        res = mcnemar(tab, exact=True)
        raw_p[c] = res.pvalue
    corrected = holm_correction(raw_p)
    return corrected, raw_p


# ------------------------------- Reporting -----------------------------------

def print_pairwise_continuous(reference, corrected, diagnostics):
    for c, adj_p in corrected.items():
        d = diagnostics[c]
        sig = "SIGNIFICANT" if adj_p < ALPHA else "not significant"
        note = "(diffs look non-normal, trust Wilcoxon)" if not np.isnan(d["shapiro_p"]) and d["shapiro_p"] < 0.05 else ""
        print(f"  {reference} vs {c}  (n={d['n']}):")
        print(f"     mean diff = {d['mean_diff']:+.4f}   Wilcoxon p={d['w_p']:.4f} "
              f"(Holm-adjusted p={adj_p:.4f}) -> {sig}")
        print(f"     [cross-check: paired t-test p={d['t_p']:.4f}, "
              f"Shapiro normality-of-diff p={d['shapiro_p']:.4f} {note}]")


def print_pairwise_binary(reference, corrected, raw_p):
    for c, adj_p in corrected.items():
        sig = "SIGNIFICANT" if adj_p < ALPHA else "not significant"
        print(f"  {reference} vs {c}: McNemar p={raw_p[c]:.4f} (Holm-adjusted p={adj_p:.4f}) -> {sig}")


def run_metric_group(group_name, df, config_col, continuous_metrics, binary_metrics, reference):
    print("\n" + "#" * 80)
    print(f"# {group_name.upper()} METRICS")
    print("#" * 80)

    pivots = {}
    omnibus_results = {}

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

    if not omnibus_results:
        print("(no metrics in this group)")
        return

    raw_p = {m: v[2] for m, v in omnibus_results.items()}
    fdr_p = bh_fdr_correction(raw_p)

    print(f"\nOmnibus tests (Friedman for continuous, Cochran's Q for binary), "
          f"BH-FDR corrected across the {len(raw_p)} {group_name} metrics:")
    print(f"  {'metric':30s} {'type':11s} {'stat':>8s} {'raw p':>9s} {'FDR p':>9s}  result")
    for m in sorted(raw_p, key=lambda m: fdr_p[m]):
        kind, stat, p = omnibus_results[m]
        sig = "SIGNIFICANT" if fdr_p[m] < ALPHA else "not significant"
        print(f"  {m:30s} {kind:11s} {stat:8.3f} {p:9.5f} {fdr_p[m]:9.5f}  {sig}")

    for metric in list(continuous_metrics) + list(binary_metrics):
        kind, pv = pivots[metric]
        means_or_rates = pv.mean().sort_values(ascending=False)
        print(f"\n--- {metric} ({kind}) ---")
        label = "mean" if kind == "continuous" else "rate"
        for c in means_or_rates.index:
            extra = f"  std={pv[c].std():.4f}" if kind == "continuous" else \
                    f"  ({int(pv[c].sum())}/{len(pv[c])})"
            print(f"  {str(c):65s} {label}={means_or_rates[c]:8.4f}{extra}")

        if fdr_p[metric] >= ALPHA:
            print(f"  Omnibus not significant after FDR correction (p={fdr_p[metric]:.4f}) "
                  f"-> pairwise tests skipped for this metric.")
            continue

        if reference is None:
            reference_here = means_or_rates.index[0]
            print(f"  No reference given; using best-performing config as stand-in reference: {reference_here}")
        elif reference not in pv.columns:
            print(f"  WARNING: reference '{reference}' not found for this metric. Skipping pairwise tests.")
            continue
        else:
            reference_here = reference

        print(f"  Pairwise tests vs reference = '{reference_here}':")
        if kind == "continuous":
            corrected, diagnostics = wilcoxon_pairwise_vs_reference(pv, reference_here)
            print_pairwise_continuous(reference_here, corrected, diagnostics)
        else:
            corrected, raw = mcnemar_pairwise_vs_reference(pv, reference_here)
            print_pairwise_binary(reference_here, corrected, raw)


def cable_severity_exploratory(df, config_col, reference):
    print("\n" + "#" * 80)
    print("# CABLE-RUBBING SEVERITY (exploratory -- not FDR-controlled)")
    print("#" * 80)

    pv = continuous_pivot(df, config_col, "cable_severity")
    configs = list(pv.columns)
    print(f"\n{'config':65s} {'n_rubbed':>9s} {'mean_severity_m':>17s} {'worst_severity_m':>18s}")
    for c in configs:
        col = pv[c].dropna()
        print(f"{str(c):65s} {len(col):9d} {col.mean() if len(col) else float('nan'):17.5f} "
              f"{col.min() if len(col) else float('nan'):18.5f}")

    if reference is None:
        reference = pv.mean().idxmax() if not pv.isna().all().all() else None
        if reference is None:
            print("\nNo cable rubbing detected across configurations.")
            return
        print(f"\nNo reference given; using config with mildest mean severity as stand-in: {reference}")
    elif reference not in configs:
        print(f"\nWARNING: reference '{reference}' not found. Skipping.")
        return

    print(f"\nPairwise comparisons vs reference = '{reference}' (joint-rubbed subset only):")
    for c in configs:
        if c == reference:
            continue
        joint = pv[[reference, c]].dropna()
        if len(joint) < 5:
            print(f"  {reference} vs {c}: only {len(joint)} jointly-rubbed poses -- too few to test.")
            continue
        diff = joint[reference] - joint[c]
        if np.allclose(diff, 0):
            print(f"  {reference} vs {c}  (n={len(joint)} jointly-rubbed poses): identical values.")
            continue
        w_stat, w_p = stats.wilcoxon(joint[reference], joint[c])
        print(f"  {reference} vs {c}  (n={len(joint)} jointly-rubbed poses): "
              f"mean diff={diff.mean():+.4f}, Wilcoxon p={w_p:.4f} (uncorrected)")


# --------------------------------- Loading -----------------------------------

def load_with_optional_reference():
    """Load candidate results and optional rubbing log safely."""
    if not RESULTS_XLSX.exists():
        print(f"[ERROR] Could not find 'results.xlsx' in: {DATA_DIR}")
        sys.exit(1)

    df = pd.read_excel(RESULTS_XLSX)
    rub = pd.read_excel(RUBBING_XLSX) if RUBBING_XLSX.exists() else pd.DataFrame()

    reference_name = None

    if ACTS_RESULTS_XLSX.exists():
        acts_df = pd.read_excel(ACTS_RESULTS_XLSX)
        reference_candidates = acts_df[CONFIG_COL].unique()
        reference_name = reference_candidates[0]
        df = pd.concat([df, acts_df], ignore_index=True)
        print(f"Loaded reference configuration '{reference_name}' from {ACTS_RESULTS_XLSX}")
    else:
        print(f"No reference results found in {ACTS_RESULTS_XLSX}. Comparing {df[CONFIG_COL].nunique()} candidate configuration(s).")

    if ACTS_RUBBING_XLSX.exists():
        acts_rub = pd.read_excel(ACTS_RUBBING_XLSX)
        rub = pd.concat([rub, acts_rub], ignore_index=True)

    return df, rub, reference_name


def main():
    # Stream terminal prints to both terminal and text file
    logger = TeeStream(OUTPUT_REPORT_TXT)
    sys.stdout = logger

    try:
        print("=" * 80)
        print("Loading data")
        print("=" * 80)
        df, rub, reference_name = load_with_optional_reference()



        print("\n" + "=" * 80)
        print("STEP 1: Checking configuration pose pairing")
        print("=" * 80)
        ok = check_pairing(df, CONFIG_COL, POSE_COLS)
        print("Pairing OK." if ok else "Pairing issue detected.")

        df = add_cable_features(df, rub, CONFIG_COL, RUBBING_CONFIG_COL)

        run_metric_group("primary", df, CONFIG_COL, PRIMARY_CONTINUOUS, PRIMARY_BINARY, reference_name)
        run_metric_group("diagnostic", df, CONFIG_COL, DIAGNOSTIC_CONTINUOUS, DIAGNOSTIC_BINARY, reference_name)
        cable_severity_exploratory(df, CONFIG_COL, reference_name)

        print("\n" + "=" * 80)
        print("Done.")
        print(f"Report saved to: {OUTPUT_REPORT_TXT}")
        print("=" * 80)

    finally:
        sys.stdout = logger.stdout
        logger.close()


if __name__ == "__main__":
    main()