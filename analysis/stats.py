"""Statistics used by every analysis script (Module 20; protocol in docs/preregistration.md §7).

    bootstrap_ci(x)            percentile 95% CI of the median, resampling units with replacement
    wilcoxon_p(d)              two-sided Wilcoxon signed-rank p, zero differences dropped
    holm(p)                    Holm step-down adjusted p-values
    paired_effect(a, b)        median of b - a with CI and p, for one (map, method) cell
    loglog_slope(df)           slope of log(latency) on log(n_frontiers), cluster bootstrap CI over runs
    ratio_of_medians_ci(a, b)  CI of median(a) / median(b) under a joint (paired) bootstrap

Fixed: 10,000 resamples, seed 249, alpha 0.05. Change nothing here after prereg-v1.
"""
from __future__ import annotations

import math

import numpy as np
from scipy import stats as sps

N_BOOT = 10_000
SEED = 249
ALPHA = 0.05


def bootstrap_ci(x, stat=np.median, n_boot=N_BOOT, seed=SEED, alpha=ALPHA):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return (math.nan, math.nan)
    if len(x) == 1:
        return (float(x[0]), float(x[0]))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), size=(n_boot, len(x)))
    boots = np.apply_along_axis(stat, 1, x[idx]) if stat is not np.median else np.median(x[idx], axis=1)
    return (float(np.percentile(boots, 100 * alpha / 2)), float(np.percentile(boots, 100 * (1 - alpha / 2))))


def wilcoxon_p(d) -> float:
    d = np.asarray(d, dtype=float)
    d = d[np.isfinite(d)]
    if np.count_nonzero(d) < 2:
        return math.nan
    try:
        return float(sps.wilcoxon(d, zero_method="wilcox", alternative="two-sided").pvalue)
    except ValueError:
        return math.nan


def holm(pvals):
    """Holm-Bonferroni adjusted p-values (NaNs are left out and stay NaN)."""
    p = np.asarray(pvals, dtype=float)
    out = np.full(len(p), math.nan)
    ok = np.where(np.isfinite(p))[0]
    m = len(ok)
    if m == 0:
        return out
    order = ok[np.argsort(p[ok])]
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (m - rank) * p[i]))
        out[i] = running
    return out


def paired_effect(base, other, censored=None) -> dict:
    """Effect of `other` vs `base` over paired units: d = other - base."""
    base, other = np.asarray(base, float), np.asarray(other, float)
    ok = np.isfinite(base) & np.isfinite(other)
    d = other[ok] - base[ok]
    rel = d / base[ok]
    lo, hi = bootstrap_ci(d)
    rlo, rhi = bootstrap_ci(rel)
    return {"n": int(ok.sum()), "n_censored": int(np.asarray(censored)[ok].sum()) if censored is not None else 0,
            "median_diff": float(np.median(d)) if len(d) else math.nan, "ci_low": lo, "ci_high": hi,
            "median_rel": float(np.median(rel)) if len(rel) else math.nan, "rel_ci_low": rlo, "rel_ci_high": rhi,
            "p": wilcoxon_p(d)}


def _ols_slope(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3 or np.ptp(x) == 0:
        return math.nan, math.nan
    b, a = np.polyfit(x, y, 1)
    return float(b), float(a)


def loglog_slope(df, x="n_frontiers", y="latency_s", cluster="run_id", n_boot=2000, seed=SEED) -> dict:
    """OLS slope of log(y) on log(x) with a cluster (run) bootstrap CI. Rows with x <= 0 or
    y <= 0 are dropped (log undefined)."""
    d = df[(df[x] > 0) & (df[y] > 0)]
    if len(d) < 3:
        return {"n_cycles": int(len(d)), "n_runs": 0, "slope": math.nan, "ci_low": math.nan, "ci_high": math.nan}
    lx, ly = np.log(d[x].to_numpy(float)), np.log(d[y].to_numpy(float))
    slope, intercept = _ols_slope(lx, ly)
    groups = d[cluster].to_numpy()
    uniq = np.unique(groups)
    idx_of = {g: np.where(groups == g)[0] for g in uniq}
    rng = np.random.default_rng(seed)
    boots = []
    for _ in range(n_boot):
        pick = rng.choice(uniq, size=len(uniq), replace=True)
        ii = np.concatenate([idx_of[g] for g in pick])
        b, _ = _ols_slope(lx[ii], ly[ii])
        if np.isfinite(b):
            boots.append(b)
    lo, hi = (np.percentile(boots, [2.5, 97.5]) if boots else (math.nan, math.nan))
    return {"n_cycles": int(len(d)), "n_runs": int(len(uniq)), "slope": slope, "intercept": intercept,
            "ci_low": float(lo), "ci_high": float(hi)}


def ratio_of_medians_ci(num, den, n_boot=N_BOOT, seed=SEED, alpha=ALPHA):
    """CI of median(num) / median(den) with the pairs resampled together. Resamples where
    median(den) <= 0 are kept as +inf (no break-even)."""
    num, den = np.asarray(num, float), np.asarray(den, float)
    ok = np.isfinite(num) & np.isfinite(den)
    num, den = num[ok], den[ok]
    if len(num) == 0:
        return math.nan, (math.nan, math.nan)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(num), size=(n_boot, len(num)))
    mn, md = np.median(num[idx], axis=1), np.median(den[idx], axis=1)
    r = np.where(md > 0, mn / np.where(md > 0, md, 1.0), np.inf)
    point = float(np.median(num) / np.median(den)) if np.median(den) > 0 else math.inf
    r = np.sort(r)                     # order statistics: +inf (no break-even) sorts last, no NaN
    lo = r[int(np.floor(alpha / 2 * (n_boot - 1)))]
    hi = r[int(np.ceil((1 - alpha / 2) * (n_boot - 1)))]
    return point, (float(lo), float(hi))
