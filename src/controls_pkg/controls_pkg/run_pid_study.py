"""P / PD / PI / PID steering study on the bicycle model: experiments and plots.

Plain script (no ROS). From src/controls_pkg:

    python3 -s -m controls_pkg.run_pid_study

writes figures to docs/pid_study/figures, numbers to docs/pid_study/results and
fills the tables in docs/pid_study/README.md (between BEGIN/END markers).
`-s` skips ~/.local so the system numpy and matplotlib agree.
"""
import argparse
import csv
import glob
import itertools
import math
import os
import re
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

from .car_course_path import make_track_path
from .pid_sim import (Gains, SimConfig, circle_path, cone_track_path, simulate,
                      step_metrics, straight_path, tracking_metrics)

TYPES = ('P', 'PD', 'PI', 'PID')
MODES = ('cross_track', 'lookahead')
MODE_NAME = {'cross_track': 'cross-track error', 'lookahead': 'look-ahead heading error'}
TRIM = math.radians(2.0)            # steering trim used as the standing disturbance
EFFORT_LIMIT = 180.0                # design spec: servo slew rate [deg/s]
MAX_OVERSHOOT = 10.0                # design spec: step overshoot [% of the offset]
MAX_SETTLE = 4.0                    # design spec: step settling time [s]
EVAL_SEEDS = range(20)

LINE, CIRCLE = straight_path(), circle_path()
CONE, RACE = cone_track_path(), make_track_path(1)
TRACKS = {'cone track': CONE, 'race track': RACE}

GRID = {
    'cross_track': {'kp': (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0),
                    'ki': (0.1, 0.25, 0.5, 1.0, 2.0, 4.0),
                    'kd': (0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0),
                    'tau': (0.0, 0.1, 0.3)},
    'lookahead': {'kp': (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0),
                  'ki': (0.1, 0.25, 0.5, 1.0, 2.0),
                  'kd': (0.05, 0.1, 0.2, 0.4, 0.8),
                  'tau': (0.1, 0.3, 0.5)},
}

# chart colours: one fixed hue per controller type, one blue ramp for ordered sweeps
COLOR = {'P': '#2a78d6', 'PD': '#eb6834', 'PI': '#1baf7a', 'PID': '#eda100'}
RAMP = ('#9ec5f4', '#5598e7', '#2a78d6', '#184f95', '#0d366b')
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRIDC, AXIS, SURFACE = '#e1e0d9', '#c3c2b7', '#fcfcfb'
REF = '#898781'
NOTE = {'facecolor': SURFACE, 'edgecolor': 'none', 'pad': 2.0}     # backing for notes
BOXED = {'frameon': True, 'facecolor': SURFACE, 'edgecolor': 'none', 'framealpha': 1.0}


# ---------------------------------------------------------------- tuning ----

def tuning_runs(mode):
    """Return the three noisy runs every candidate is scored on."""
    base = SimConfig(error_mode=mode, noise=True, steer_bias=TRIM)
    return ((LINE, replace(base, offset=0.5, t_max=20.0, seed=100), False),
            (CIRCLE, replace(base, seed=101), True),
            (CONE, replace(base, seed=102), True))


def score(job):
    """Score one candidate: clean step response, then the noisy runs.

    Returns the mean RMS error and steering effort over the noisy runs (inf
    when the car leaves the track) and the overshoot and settling time of the
    clean step.
    """
    mode, g = job
    step = step_metrics(simulate(LINE, Gains(*g), SimConfig(error_mode=mode, offset=0.5,
                                                            t_max=20.0)))
    rms, effort = [], []
    for path, cfg, must_finish in tuning_runs(mode):
        log = simulate(path, Gains(*g), cfg)
        if log['off_track'] or (must_finish and not log['finished']):
            return {'g': g, 'rms': math.inf, 'effort': math.inf, **step}
        m = tracking_metrics(log)
        rms.append(m['rms_cte'])
        effort.append(m['effort'])
    return {'g': g, 'rms': float(np.mean(rms)), 'effort': float(np.mean(effort)), **step}


def candidates(mode, kind):
    """(kp, ki, kd, tau) grid for one controller type."""
    g = GRID[mode]
    ki = g['ki'] if 'I' in kind else (0.0,)
    kd = g['kd'] if 'D' in kind else (0.0,)
    tau = g['tau'] if 'D' in kind else (0.0,)
    return list(itertools.product(g['kp'], ki, kd, tau))


def tune(pool):
    """Grid-search every controller type in both error modes.

    Rule: among the candidates that complete every noisy run and meet the
    three design specs (step overshoot, step settling time, steering effort),
    take the one with the lowest mean RMS cross-track error.
    """
    rows, chosen = [], {}
    for mode in MODES:
        for kind in TYPES:
            cands = candidates(mode, kind)
            res = pool.map(score, [(mode, g) for g in cands], chunksize=8)
            done = [r for r in res if math.isfinite(r['rms'])]
            ok = [r for r in done if r['effort'] <= EFFORT_LIMIT
                  and r['overshoot'] <= MAX_OVERSHOOT and r['settle'] <= MAX_SETTLE]
            best = min(ok, key=lambda r: r['rms']) if ok else None
            chosen[mode, kind] = best['g'] if best else None
            row = {'error signal': MODE_NAME[mode], 'type': kind, 'tried': len(cands),
                   'complete': len(done), 'meet specs': len(ok)}
            for i, name in enumerate(('kp', 'ki', 'kd', 'tau')):
                row[name] = best['g'][i] if best else ''
            for name in ('rms', 'effort', 'overshoot', 'settle'):
                row[name] = best[name] if best else math.nan
            rows.append(row)
    # a type with no working setting is shown as the ablation of the next richer one
    for mode in MODES:
        for kind, donor in (('PI', 'PID'), ('P', 'PD')):
            if chosen[mode, kind] is None:
                kp, ki, _, _ = chosen[mode, donor]
                chosen[mode, kind] = (kp, ki if 'I' in kind else 0.0, 0.0, 0.0)
    return rows, chosen


# -------------------------------------------------------------- plotting ----

def setup_matplotlib():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'font.size': 10, 'axes.titlesize': 11, 'axes.titleweight': 'bold',
        'axes.titlelocation': 'left', 'axes.labelcolor': INK2, 'text.color': INK,
        'axes.edgecolor': AXIS, 'axes.linewidth': 1.0, 'axes.spines.top': False,
        'axes.spines.right': False, 'xtick.color': MUTED, 'ytick.color': MUTED,
        'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2, 'axes.grid': True,
        'grid.color': GRIDC, 'grid.linewidth': 1.0, 'axes.axisbelow': True,
        'lines.linewidth': 2.0, 'lines.solid_capstyle': 'round', 'legend.frameon': False,
        'legend.fontsize': 9, 'figure.dpi': 100, 'savefig.dpi': 150,
    })
    return plt


def save(fig, out, name):
    fig.tight_layout()
    path = os.path.join(out, 'figures', name)
    fig.savefig(path)
    print('saved', os.path.relpath(path))


def ramp(n):
    return [RAMP[round(i * (len(RAMP) - 1) / max(1, n - 1))] for i in range(n)]


def log_axis(ax, values):
    """Log x axis with plain-number tick labels."""
    from matplotlib.ticker import FuncFormatter, NullFormatter
    ax.set_xscale('log')
    ax.set_xlim(min(values) / 1.3, max(values) * 1.3)
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
    ax.xaxis.set_minor_formatter(NullFormatter())


def recovery_time(log, start, band=0.005):
    """Seconds after `start` until the error stays inside +-band [m]; nan if never."""
    t, cte = log['t'], np.abs(log['cte'])
    outside = np.where((t > start) & (cte > band))[0]
    if len(outside) == 0:
        return 0.0
    if t[outside[-1]] > t[-1] - 3.0:
        return math.nan
    return float(t[outside[-1] + 1] - start)


def label(kind, g):
    parts = [f'kp={g[0]:g}']
    if g[1]:
        parts.append(f'ki={g[1]:g}')
    if g[2]:
        parts.append(f'kd={g[2]:g}')
    return f"{kind} ({', '.join(parts)})"


def fmt(x, digits=3):
    if isinstance(x, str):
        return x
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return 'n/a'
    if isinstance(x, (int, np.integer)):
        return str(x)
    return f'{x:.{digits}f}'


def md_table(rows, columns, digits=None):
    """Markdown table from a list of dicts; `columns` is {key: heading}."""
    digits = digits or {}
    out = ['| ' + ' | '.join(columns.values()) + ' |',
           '|' + '|'.join('---' for _ in columns) + '|']
    for r in rows:
        out.append('| ' + ' | '.join(fmt(r[k], digits.get(k, 3)) for k in columns) + ' |')
    return '\n'.join(out)


def write_csv(out, name, rows):
    path = os.path.join(out, 'results', name)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------- 1. controller comparison ----

def exp_tracks(plt, out):
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    for ax, (name, path) in zip(axes, TRACKS.items()):
        ax.plot(path.x, path.y, color=COLOR['P'])
        ax.plot(path.x[0], path.y[0], 'o', color=INK, ms=7, mec=SURFACE, mew=2)
        ax.annotate('start', (path.x[0], path.y[0]), xytext=(6, 8),
                    textcoords='offset points', color=INK2)
        ax.set(title=f'{name}: {path.s[-1]:.1f} m, tightest radius {path.min_radius:.2f} m',
               xlabel='x [m]', ylabel='y [m]', aspect='equal')
    save(fig, out, 'fig01_tracks.png')


def exp_step(plt, out, chosen):
    """0.5 m offset from a straight line, ideal sensors, servo lag and delay on."""
    rows = []
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.4), sharex=True)
    for col, mode in enumerate(MODES):
        for kind in TYPES:
            g = chosen[mode, kind]
            log = simulate(LINE, Gains(*g), SimConfig(error_mode=mode, offset=0.5, t_max=20.0))
            m = step_metrics(log)
            rows.append({'error signal': MODE_NAME[mode], 'type': kind, 'gains': label(kind, g),
                         **m, 'max': float(np.abs(log['cte']).max())})
            axes[0, col].plot(log['t'], log['cte'], color=COLOR[kind], label=label(kind, g))
            axes[1, col].plot(log['t'], np.degrees(log['command']), color=COLOR[kind])
        axes[0, col].axhline(0.0, color=REF, lw=1)
        axes[0, col].set(title=f'Steering on {MODE_NAME[mode]}', ylabel='cross-track error [m]')
        low, high = axes[0, col].get_ylim()
        axes[0, col].set_ylim(low, high + 0.4 * (high - low))
        axes[0, col].legend(loc='upper right')
        axes[1, col].set(xlabel='time [s]', ylabel='steering command [deg]')
    save(fig, out, 'fig02_step_response.png')
    write_csv(out, 'step_response.csv', rows)
    return rows


def exp_disturbance(plt, out, chosen):
    """Straight line; a 3 degree steering trim appears at t = 5 s."""
    rows = []
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    for ax, mode in zip(axes, MODES):
        for kind in TYPES:
            g = chosen[mode, kind]
            log = simulate(LINE, Gains(*g), SimConfig(
                error_mode=mode, steer_bias=math.radians(3.0), bias_start=5.0, t_max=30.0))
            t, cte = log['t'], log['cte']
            rows.append({'error signal': MODE_NAME[mode], 'type': kind,
                         'peak': float(np.abs(cte).max()) * 1000.0,
                         'final': float(np.mean(cte[t > t[-1] - 3.0])) * 1000.0,
                         'ripple': float(np.ptp(cte[t > t[-1] - 3.0])) * 1000.0})
            ax.plot(t, cte * 1000.0, color=COLOR[kind], label=kind)
        ax.axvline(5.0, color=REF, lw=1)
        ax.set(title=f'Steering on {MODE_NAME[mode]}', xlabel='time [s]', ylim=(-30.0, 50.0))
        ax.legend(ncol=4, loc='upper right', **BOXED)
        ax.annotate('trim appears', (5.0, -30.0), xytext=(4, 4), textcoords='offset points',
                    color=INK2, fontsize=9, bbox=NOTE)
    axes[0].set(ylabel='cross-track error [mm]')
    axes[0].annotate('P and PI swing off this scale\n(up to about 1 m, see table)', (0.97, 0.06),
                     xycoords='axes fraction', ha='right', color=INK2, fontsize=9, bbox=NOTE)
    save(fig, out, 'fig03_disturbance.png')
    write_csv(out, 'disturbance.csv', rows)
    return rows


def exp_circle(plt, out, chosen):
    """Constant-radius turn (R = 1.5 m), to see whether a steady error remains."""
    rows = []
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    for ax, mode in zip(axes, MODES):
        for kind in TYPES:
            g = chosen[mode, kind]
            log = simulate(CIRCLE, Gains(*g), SimConfig(error_mode=mode))
            t, cte = log['t'], log['cte']
            tail = t > t[-1] - 8.0
            rows.append({'error signal': MODE_NAME[mode], 'type': kind,
                         'final': float(np.mean(cte[tail])) * 1000.0,
                         'ripple': float(np.ptp(cte[tail])) * 1000.0,
                         'finished': 'yes' if log['finished'] else 'no'})
            ax.plot(t, cte * 1000.0, color=COLOR[kind], label=kind)
        ax.axhline(0.0, color=REF, lw=1)
        ax.set(title=f'Steering on {MODE_NAME[mode]}', xlabel='time [s]', ylim=(-90.0, 130.0))
        ax.legend(ncol=4, loc='upper right', **BOXED)
    axes[0].set(ylabel='cross-track error [mm]')
    axes[0].annotate('P and PI swing off this scale\n(up to about 1.2 m, see table)', (0.97, 0.06),
                     xycoords='axes fraction', ha='right', color=INK2, fontsize=9, bbox=NOTE)
    save(fig, out, 'fig04_constant_turn.png')
    write_csv(out, 'constant_turn.csv', rows)
    return rows


def lap_job(job):
    """One noisy lap with the standing trim; `tag` comes back with the metrics."""
    tag, track, mode, g, seed = job
    gains = g if isinstance(g, Gains) else Gains(*g)
    cfg = SimConfig(error_mode=mode, noise=True, steer_bias=TRIM, seed=seed)
    return tag, tracking_metrics(simulate(TRACKS[track], gains, cfg))


def lap_stats(pool, cases, track='cone track', mode='cross_track', seeds=range(10)):
    """Run {name: gains} over the seeds; return {name: summary row}."""
    res = pool.map(lap_job, [(n, track, mode, g, s) for n, g in cases.items() for s in seeds],
                   chunksize=4)
    return {n: summarise([m for tag, m in res if tag == n]) for n in cases}


def summarise(runs):
    """Mean and standard deviation over the runs that finished."""
    done = [m for m in runs if m['finished']]
    row = {'finished': f'{len(done)}/{len(runs)}'}
    for k in ('rms_cte', 'max_cte', 'effort', 'saturated'):
        v = [m[k] for m in done]
        row[k] = float(np.mean(v)) if v else math.nan
        row[k + '_sd'] = float(np.std(v)) if v else math.nan
    return row


def exp_laps(plt, out, chosen, pool):
    """Noisy laps with a 2 degree trim, 20 seeds, on both tracks."""
    jobs = [((track, mode, kind, seed), track, mode, chosen[mode, kind], seed)
            for track in TRACKS for mode in MODES for kind in TYPES for seed in EVAL_SEEDS]
    res = pool.map(lap_job, jobs, chunksize=4)
    raw, rows, groups = [], [], {}
    for (track, mode, kind, seed), m in res:
        raw.append({'track': track, 'error signal': MODE_NAME[mode], 'type': kind,
                    'seed': seed, **m})
        groups.setdefault((track, mode, kind), []).append(m)
    for (track, mode, kind), runs in groups.items():
        rows.append({'track': track, 'error signal': MODE_NAME[mode], 'type': kind,
                     **summarise(runs)})
    write_csv(out, 'laps_runs.csv', raw)
    write_csv(out, 'laps_summary.csv', rows)

    fig, axes = plt.subplots(2, 2, figsize=(10, 6.4), sharey=True)
    for r, track in enumerate(TRACKS):
        for c, mode in enumerate(MODES):
            ax = axes[r, c]
            for i, kind in enumerate(TYPES):
                runs = groups[track, mode, kind]
                v = [m['rms_cte'] * 1000.0 for m in runs if m['finished']]
                if v:
                    jitter = np.linspace(-0.16, 0.16, len(v))
                    ax.plot(i + jitter, v, 'o', color=COLOR[kind], ms=5, mec=SURFACE, mew=0.8)
                    ax.plot([i - 0.28, i + 0.28], [np.mean(v)] * 2, color=INK, lw=2)
                    ax.annotate(f'{np.mean(v):.0f}', (i + 0.3, np.mean(v)), xytext=(3, -3),
                                textcoords='offset points', color=INK2, fontsize=9)
                if len(v) < len(runs):
                    ax.annotate(f'{len(runs) - len(v)}/{len(runs)}\nleft the track', (i, 0.5),
                                xycoords=('data', 'axes fraction'), ha='center', va='center',
                                color=INK2, fontsize=9)
            ax.set_xticks(range(len(TYPES)))
            ax.set_xticklabels(TYPES)
            ax.set_xlim(-0.6, len(TYPES) - 0.3)
            ax.set_ylim(bottom=0.0)
            ax.grid(axis='x', visible=False)
            ax.set(title=f'{track.capitalize()}, steering on {MODE_NAME[mode]}',
                   ylabel='RMS cross-track error [mm]' if c == 0 else '')
    save(fig, out, 'fig05_laps.png')

    # one representative lap per working controller, to see where the error is made
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.0))
    axes[0].plot(CONE.x, CONE.y, color=REF, lw=1, label='reference')
    for mode, kind in (('cross_track', 'PD'), ('cross_track', 'PID'), ('lookahead', 'P')):
        colour = COLOR[kind]
        log = simulate(CONE, Gains(*chosen[mode, kind]),
                       SimConfig(error_mode=mode, noise=True, steer_bias=TRIM, seed=0))
        name = f'{kind}, {MODE_NAME[mode]}'
        axes[0].plot(log['x'], log['y'], color=colour, lw=1.5, label=name)
        axes[1].plot(log['t'] * 0.6, log['cte'] * 1000.0, color=colour, lw=1.5, label=name)
    axes[0].set(title='Cone track, one noisy lap', xlabel='x [m]', ylabel='y [m]', aspect='equal')
    axes[1].axhline(0.0, color=REF, lw=1)
    axes[1].set(title='Error along the lap', xlabel='distance driven [m]',
                ylabel='cross-track error [mm]')
    axes[1].legend()
    save(fig, out, 'fig06_lap_detail.png')
    return rows


# ------------------------------------------------- 2. effect of each gain ----

def exp_gain_sweeps(plt, out, base, pool):
    """Change one gain of the tuned cross-track PID at a time."""
    names = ('kp', 'ki', 'kd')
    fine = np.geomspace(0.125, 8.0, 13)
    step_cfg = SimConfig(offset=0.5, t_max=20.0)
    trim_cfg = SimConfig(steer_bias=math.radians(3.0), bias_start=5.0, t_max=30.0)
    rows = []
    for n, name in enumerate(names):
        def scaled(f):
            g = list(base)
            g[n] = base[n] * f
            return tuple(g)

        fig, axes = plt.subplots(1, 3, figsize=(12, 3.6),
                                 gridspec_kw={'width_ratios': [1.5, 1, 1]})
        # kp and kd shape the step response; ki only has a job when there is a trim
        family = (0.0, 0.25, 1.0, 4.0, 16.0) if name == 'ki' else (0.25, 0.5, 1.0, 2.0, 4.0)
        for f, colour in zip(family, RAMP):
            log = simulate(LINE, Gains(*scaled(f)), trim_cfg if name == 'ki' else step_cfg)
            y = log['cte'] * 1000.0 if name == 'ki' else log['cte']
            axes[0].plot(log['t'], y, color=colour, lw=3.0 if f == 1.0 else 1.6,
                         label=f'{name} = {scaled(f)[n]:g}' + (' (tuned)' if f == 1.0 else ''))
        axes[0].axhline(0.0, color=REF, lw=1)
        if name == 'ki':
            axes[0].set(title='A 3 deg steering trim appears at 5 s', xlabel='time [s]',
                        ylabel='cross-track error [mm]', ylim=(-12.0, 34.0))
        else:
            axes[0].set(title=f'0.5 m step as {name} changes', xlabel='time [s]',
                        ylabel='cross-track error [m]', ylim=(-0.5, 1.0))
        axes[0].legend(loc='upper right', ncol=2)

        laps = lap_stats(pool, {i: scaled(f) for i, f in enumerate(fine)}, seeds=range(5))
        for i, f in enumerate(fine):
            g = scaled(f)
            m = step_metrics(simulate(LINE, Gains(*g), step_cfg))
            rows.append({'gain': name, 'value': g[n], **m,
                         'recovery': recovery_time(simulate(LINE, Gains(*g), trim_cfg), 5.0),
                         'lap finished': laps[i]['finished'], 'lap rms': laps[i]['rms_cte'],
                         'lap effort': laps[i]['effort']})
        mine = [r for r in rows if r['gain'] == name]
        x = [r['value'] for r in mine]
        second = (('recovery', 'Time to remove the trim error') if name == 'ki'
                  else ('settle', 'Settling time of the step (5 %)'))
        panels = (('overshoot', 'Overshoot of the step', '[% of offset]'), (*second, '[s]'))
        for ax, (key, title, unit) in zip(axes[1:], panels):
            y = [r[key] for r in mine]
            ax.plot(x, y, 'o-', color=COLOR['P'], ms=5)
            ax.set(title=title, xlabel=name, ylabel=unit)
            log_axis(ax, x)
            ax.set_ylim(bottom=0.0)
            ax.axvline(base[n], color=REF, lw=1)
            ax.annotate('tuned', (base[n], 1.0), xycoords=('data', 'axes fraction'),
                        xytext=(3, -10), textcoords='offset points', color=INK2, fontsize=9)
            if any(math.isnan(v) for v in y):
                ax.annotate('no point: never', (0.97, 0.05), xycoords='axes fraction', ha='right',
                            color=INK2, fontsize=9)
        save(fig, out, f'fig0{7 + n}_{name}_sweep.png')
    write_csv(out, 'gain_sweeps.csv', rows)

    # noisy-lap view of the same sweeps: accuracy and steering effort
    fig, axes = plt.subplots(2, 3, figsize=(12, 5.6), sharex='col', sharey='row')
    for n, name in enumerate(names):
        mine = [r for r in rows if r['gain'] == name]
        x = [r['value'] for r in mine]
        axes[0, n].plot(x, [r['lap rms'] * 1000.0 for r in mine], 'o-', color=COLOR['P'], ms=5)
        axes[0, n].set(title=f'Noisy cone lap as {name} changes',
                       ylabel='RMS error [mm]' if n == 0 else '')
        axes[1, n].plot(x, [r['lap effort'] for r in mine], 'o-', color=COLOR['PD'], ms=5)
        axes[1, n].axhline(EFFORT_LIMIT, color=REF, lw=1)
        axes[1, n].set(xlabel=name, ylabel='steering effort [deg/s]' if n == 0 else '')
        for ax in axes[:, n]:
            log_axis(ax, x)
            ax.set_ylim(bottom=0.0)
            ax.axvline(base[n], color=REF, lw=1)
    axes[1, 0].annotate('servo slew limit', (0.02, EFFORT_LIMIT),
                        xycoords=('axes fraction', 'data'), xytext=(0, 3),
                        textcoords='offset points', color=INK2, fontsize=9)
    save(fig, out, 'fig10_gain_sweeps_lap.png')
    return rows


def map_job(job):
    kp, kd, tau = job
    log = simulate(LINE, Gains(kp, 0.0, kd, tau), SimConfig(offset=0.5, t_max=20.0))
    return step_metrics(log)['settle']


def exp_kp_kd_map(plt, out, base, pool):
    """Settling time of the step response over the kp-kd plane (ki = 0)."""
    kps, kds = np.geomspace(0.25, 32.0, 22), np.geomspace(0.125, 16.0, 22)
    res = pool.map(map_job, [(kp, kd, base[3]) for kd in kds for kp in kps], chunksize=8)
    z = np.array(res).reshape(len(kds), len(kps))
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list('blues', ['#cde2fb', '#2a78d6', '#0d366b'])
    cmap.set_bad('#e1e0d9')
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    ax.grid(False)
    mesh = ax.pcolormesh(kps, kds, np.ma.masked_invalid(z), cmap=cmap, shading='nearest',
                         vmin=0.0, vmax=12.0, edgecolors=SURFACE, linewidth=0.5)
    ax.plot(base[0], base[2], 'o', color=COLOR['PD'], ms=9, mec=SURFACE, mew=2)
    ax.annotate('tuned', (base[0], base[2]), xytext=(8, 6), textcoords='offset points', color=INK)
    ax.set(title='Settling time of the 0.5 m step, ki = 0 (grey: never settles)', xscale='log',
           yscale='log', xlabel='kp', ylabel='kd')
    fig.colorbar(mesh, ax=ax, label='settling time [s]', extend='max')
    save(fig, out, 'fig11_kp_kd_map.png')
    np.savetxt(os.path.join(out, 'results', 'kp_kd_settling.csv'),
               np.vstack([np.r_[np.nan, kps], np.c_[kds, z]]), delimiter=',', fmt='%.4g')
    return float(np.mean(np.isnan(z))) * 100.0


# -------------------------------------------- 3. implementation techniques ----

def exp_d_term(plt, out, base, pool):
    """Where the D term gets its derivative from, on noisy cone laps (tuned PD gains)."""
    kp, _, kd, tau0 = base
    taus = (0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0)
    cases = {f'difference, tau = {t:g} s': Gains(kp, 0.0, kd, t, measured_rate=False)
             for t in taus}
    cases.update({f'measured rate, tau = {t:g} s': Gains(kp, 0.0, kd, t) for t in taus})
    stats = lap_stats(pool, cases)
    rows = [{'D term': n, **r} for n, r in stats.items()]
    write_csv(out, 'd_term.csv', rows)

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    x = range(len(taus))
    for src, colour in (('difference', COLOR['PD']), ('measured rate', COLOR['P'])):
        mine = [stats[f'{src}, tau = {t:g} s'] for t in taus]
        axes[0].plot(x, [r['rms_cte'] * 1000.0 for r in mine], 'o-', color=colour, ms=5, label=src)
        axes[1].plot(x, [r['effort'] for r in mine], 'o-', color=colour, ms=5, label=src)
    axes[0].set(title='Accuracy', ylabel='RMS cross-track error [mm]')
    axes[1].axhline(EFFORT_LIMIT, color=REF, lw=1)
    axes[1].set(title='Steering effort', ylabel='RMS command rate [deg/s]')
    for ax in axes[:2]:
        ax.set_xticks(list(x))
        ax.set_xticklabels([f'{t:g}' for t in taus])
        ax.set(xlabel='D filter time constant tau [s]')
        ax.set_ylim(bottom=0.0)
        ax.legend(title='derivative from')
    raw = Gains(kp, 0.0, kd, 0.0, measured_rate=False)
    for g, colour, name in ((raw, COLOR['PD'], 'difference'),
                            (Gains(kp, 0.0, kd, tau0), COLOR['P'], 'measured rate')):
        log = simulate(CONE, g, SimConfig(noise=True, steer_bias=TRIM, seed=0))
        w = (log['t'] > 10.0) & (log['t'] < 14.0)
        axes[2].plot(log['t'][w], np.degrees(log['command'][w]), color=colour, lw=1.5, label=name)
    axes[2].set(title='Steering command, 4 s of a lap', xlabel='time [s]', ylabel='[deg]',
                ylim=(-34.0, 52.0))
    axes[2].legend(loc='upper right', ncol=2)
    save(fig, out, 'fig12_d_term.png')
    return rows


def exp_anti_windup(plt, out, base):
    """Large offset keeps the steering saturated; compare the integral with and without."""
    kp, ki, kd, tau = base
    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.5))
    for aw, colour, name in ((True, COLOR['P'], 'anti-windup on'),
                             (False, COLOR['PD'], 'anti-windup off')):
        log = simulate(LINE, Gains(kp, ki, kd, tau, anti_windup=aw),
                       SimConfig(offset=1.0, t_max=30.0, off_track=4.0))
        m = step_metrics(log)
        rows.append({'anti-windup': 'on' if aw else 'off', 'ki': ki,
                     'overshoot': m['overshoot'], 'settle': m['settle'],
                     'largest integral term': float(np.degrees(np.abs(log['i']).max())),
                     'saturated': tracking_metrics(log)['saturated']})
        axes[0].plot(log['t'], log['cte'], color=colour, label=name)
        axes[1].plot(log['t'], np.degrees(log['i']), color=colour)
        axes[2].plot(log['t'], np.degrees(log['command']), color=colour)
    axes[0].axhline(0.0, color=REF, lw=1)
    axes[0].set(title='Recovery from a 1.0 m offset', xlabel='time [s]',
                ylabel='cross-track error [m]')
    axes[0].legend()
    axes[1].set(title='Integral term', xlabel='time [s]', ylabel='[deg of steering]')
    axes[2].set(title='Steering command', xlabel='time [s]', ylabel='[deg]')
    save(fig, out, 'fig13_anti_windup.png')
    write_csv(out, 'anti_windup.csv', rows)
    return rows


def exp_delay(plt, out, base):
    """Step response of the tuned PD as the command delay grows."""
    kp, _, kd, tau = base
    rows = []
    steps = (0, 1, 2, 4, 6)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    for d, colour in zip(steps, RAMP):
        log = simulate(LINE, Gains(kp, 0.0, kd, tau),
                       SimConfig(offset=0.5, t_max=20.0, delay_steps=d))
        axes[0].plot(log['t'], log['cte'], color=colour, label=f'{d * 50} ms')
    for d in range(0, 11):
        for servo in (0.08, 0.2):
            m = step_metrics(simulate(LINE, Gains(kp, 0.0, kd, tau), SimConfig(
                offset=0.5, t_max=20.0, delay_steps=d, servo_tau=servo)))
            rows.append({'delay_ms': d * 50, 'servo_tau': servo, **m})
    axes[0].axhline(0.0, color=REF, lw=1)
    axes[0].set(title='Step response as the command delay grows', xlabel='time [s]',
                ylabel='cross-track error [m]', ylim=(-0.2, 0.9))
    axes[0].legend(title='delay', loc='upper right', ncol=3)
    for servo, colour in ((0.08, COLOR['P']), (0.2, COLOR['PD'])):
        mine = [r for r in rows if r['servo_tau'] == servo]
        axes[1].plot([r['delay_ms'] for r in mine], [r['overshoot'] for r in mine], 'o-',
                     color=colour, ms=5, label=f'servo lag {servo * 1000:.0f} ms')
    axes[1].set(title='Overshoot', xlabel='command delay [ms]', ylabel='[% of offset]')
    axes[1].legend()
    save(fig, out, 'fig14_delay.png')
    write_csv(out, 'delay.csv', rows)
    return rows


# ------------------------------------------ 4. look-ahead and waypoint aim ----

def exp_lookahead(plt, out, chosen):
    """P on a look-ahead point is PD on cross-track error in disguise."""
    k, speed, ld = chosen['lookahead', 'P'][0], 0.6, 0.5
    ideal = SimConfig(offset=0.3, t_max=15.0, servo_tau=0.0, delay_steps=0)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    log = simulate(LINE, Gains(k), replace(ideal, error_mode='lookahead'))
    axes[0].plot(log['t'], log['cte'], color=COLOR['P'], lw=4, alpha=0.35,
                 label=f'P on look-ahead heading, kp = {k:g}')
    eq = (k / ld, 0.0, k / speed, 0.0)
    log2 = simulate(LINE, Gains(*eq), ideal)
    axes[0].plot(log2['t'], log2['cte'], color=COLOR['PD'], lw=1.5,
                 label=f'PD on cross-track, kp = {eq[0]:g}, kd = {eq[2]:.2f}')
    n = min(len(log['cte']), len(log2['cte']))
    gap = float(np.max(np.abs(log['cte'][:n] - log2['cte'][:n])))
    axes[0].axhline(0.0, color=REF, lw=1)
    axes[0].set(title='Nearly the same response (no lag, no delay)', xlabel='time [s]',
                ylabel='cross-track error [m]')
    axes[0].legend()
    rows = []
    lds = (0.2, 0.3, 0.5, 0.8, 1.2)
    for d, colour in zip(lds, ramp(len(lds))):
        log = simulate(LINE, Gains(k), SimConfig(error_mode='lookahead', lookahead=d,
                                                 offset=0.3, t_max=15.0))
        rows.append({'lookahead': d, **step_metrics(log)})
        axes[1].plot(log['t'], log['cte'], color=colour, label=f'{d:g} m')
    axes[1].axhline(0.0, color=REF, lw=1)
    axes[1].set(title=f'Look-ahead distance acts like kd (P, kp = {k:g})', xlabel='time [s]')
    axes[1].legend(title='look-ahead')
    save(fig, out, 'fig15_lookahead.png')
    write_csv(out, 'lookahead_distance.csv', rows)
    return rows, eq, gap


def waypoint_job(job):
    name, g, seed = job
    cfg = SimConfig(error_mode='waypoint', control_dt=0.1, delay_steps=0, noise=True, seed=seed,
                    t_max=300.0)
    log = simulate(CONE, Gains(*g, measured_rate=False), cfg)
    return name, {**tracking_metrics(log), 'lap_time': float(log['t'][-1]),
                  'heading_error': float(np.mean(np.abs(log['error'])))}


def exp_waypoint(plt, out, pool):
    """Reproduce p_controller / pd_controller / pi_controller with their Gazebo gains."""
    cases = {'P (kp=2)': (2.0, 0.0, 0.0, 0.0),
             'PD (kd=0.5, raw D)': (2.0, 0.0, 0.5, 0.0),
             'PD (kd=0.5, tau=0.3)': (2.0, 0.0, 0.5, 0.3),
             'PI (ki=0.2)': (2.0, 0.2, 0.0, 0.0)}
    res = pool.map(waypoint_job, [(n, g, s) for n, g in cases.items() for s in range(10)])
    rows = []
    for n in cases:
        runs = [m for name, m in res if name == n]
        done = [m for m in runs if m['finished']]
        lap_time = float(np.mean([m['lap_time'] for m in done])) if done else math.nan
        rows.append({'controller': n, **summarise(runs), 'lap_time': lap_time,
                     'heading_error': float(np.mean([m['heading_error'] for m in runs]))})
    write_csv(out, 'waypoint_mode.csv', rows)

    # why the raw D term fails here: the error itself is noisy, and differencing magnifies it
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    cfg = SimConfig(error_mode='waypoint', control_dt=0.1, delay_steps=0, seed=0)
    for noise, colour, name in ((True, COLOR['PD'], 'with sensor noise'),
                                (False, COLOR['P'], 'no noise')):
        log = simulate(CONE, Gains(2.0), replace(cfg, noise=noise))
        w = (log['t'] > 6.0) & (log['t'] < 12.0)
        err = np.degrees(log['error'])
        axes[0].plot(log['t'][w], err[w], color=colour, lw=1.5, label=name)
        d_term = 0.5 * np.diff(err, prepend=err[0]) / cfg.control_dt
        axes[1].plot(log['t'][w], d_term[w], color=colour, lw=1.5, label=name)
    axes[0].set(title='Heading error to the next waypoint', xlabel='time [s]', ylabel='[deg]')
    axes[0].legend()
    for limit in (-30.0, 30.0):
        axes[1].axhline(limit, color=REF, lw=1)
    axes[1].annotate('steering limit', (0.02, 30.0), xycoords=('axes fraction', 'data'),
                     xytext=(0, 3), textcoords='offset points', color=INK2, fontsize=9)
    axes[1].set(title='What kd = 0.5 turns it into (raw difference)', xlabel='time [s]',
                ylabel='D term [deg of steering]')
    save(fig, out, 'fig16_waypoint_error.png')
    return rows


# -------------------------------------------------- 5. Gazebo validation ----

def exp_gazebo(plt, out):
    """Summarise the CSVs written by run_pid_gazebo.py (skipped when there are none)."""
    files = sorted(glob.glob(os.path.join(out, 'results', 'gazebo', '*.csv')))
    if not files:
        return None
    groups = {}
    for f in files:
        name, seed = re.match(r'(.+)_seed(\d+)\.csv', os.path.basename(f)).groups()
        name = name.replace('_', ' ')
        if not os.path.exists(f.replace('.csv', '.status')):
            continue                # run still in progress
        d = np.genfromtxt(f, delimiter=',', names=True)
        with open(f.replace('.csv', '.status')) as s:
            status = s.read().strip()
        rate = np.diff(d['command']) / np.maximum(np.diff(d['t']), 1e-6)
        groups.setdefault(name, []).append({
            'seed': int(seed), 'status': status, 'finished': status == 'finished',
            'rms_cte': float(np.sqrt(np.mean(d['cte'] ** 2))),
            'max_cte': float(np.max(np.abs(d['cte']))),
            'effort': float(np.degrees(np.sqrt(np.mean(rate ** 2)))),
            'saturated': float(np.mean(np.abs(d['command']) >= math.radians(30.0) - 1e-6) * 100.0),
            't': d['t'], 'cte': d['cte'], 'x': d['x'], 'y': d['y']})
    order = [n for n in (*TYPES, 'P look-ahead') if n in groups]
    rows = []
    for n in order:
        ends = sorted({m['status'].replace('_', ' ') for m in groups[n]})
        rows.append({'controller': n, 'outcome': ', '.join(ends), **summarise(groups[n])})
    write_csv(out, 'gazebo_summary.csv', rows)

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.0))
    axes[0].plot(CONE.x, CONE.y, color=REF, lw=1, label='reference')
    for name in order:
        if not any(m['finished'] for m in groups[name]):
            continue                # left the track within seconds: nothing to draw
        colour = COLOR.get(name, '#e87ba4')
        r = min((m for m in groups[name] if m['finished']), key=lambda m: m['seed'])
        axes[0].plot(r['x'], r['y'], color=colour, lw=1.5, label=name)
        axes[1].plot(r['t'], r['cte'] * 1000.0, color=colour, lw=1.5, label=name)
    axes[0].set(title='Gazebo, cone track, first seed', xlabel='x [m]', ylabel='y [m]',
                aspect='equal')
    axes[1].axhline(0.0, color=REF, lw=1)
    axes[1].set(title='Error over the lap', xlabel='simulation time [s]',
                ylabel='cross-track error [mm]')
    axes[1].legend()
    save(fig, out, 'fig17_gazebo.png')
    return rows


# ------------------------------------------------------------------ main ----

LAP_COLS = {'finished': 'laps finished', 'rms_cte': 'RMS error [mm]', 'rms_cte_sd': 'sd',
            'max_cte': 'max error [mm]', 'effort': 'effort [deg/s]', 'saturated': 'saturated [%]'}


def in_mm(rows):
    keys = ('rms_cte', 'rms_cte_sd', 'max_cte')
    return [{**r, **{k: r[k] * 1000.0 for k in keys}} for r in rows]


def fill_tables(readme, tables):
    """Replace the text between <!-- BEGIN name --> and <!-- END name --> markers."""
    if not os.path.exists(readme):
        return
    with open(readme) as f:
        text = f.read()
    for name, table in tables.items():
        text = re.sub(rf'(<!-- BEGIN {name} -->\n).*?(<!-- END {name} -->)',
                      lambda m: m.group(1) + table + '\n' + m.group(2), text, flags=re.S)
    with open(readme, 'w') as f:
        f.write(text)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(here, '..', '..', '..', 'docs', 'pid_study'))
    ap.add_argument('--workers', type=int, default=os.cpu_count())
    args = ap.parse_args()
    out = os.path.abspath(args.out)
    for sub in ('figures', 'results'):
        os.makedirs(os.path.join(out, sub), exist_ok=True)
    plt = setup_matplotlib()

    with Pool(args.workers) as pool:
        tuning, chosen = tune(pool)
        write_csv(out, 'tuning.csv', tuning)
        for (mode, kind), g in chosen.items():
            print(f'{MODE_NAME[mode]:26s} {kind:4s} kp={g[0]:g} ki={g[1]:g} kd={g[2]:g} '
                  f'tau={g[3]:g}')
        base = chosen['cross_track', 'PID']
        pd = chosen['cross_track', 'PD']

        exp_tracks(plt, out)
        step = exp_step(plt, out, chosen)
        dist = exp_disturbance(plt, out, chosen)
        circ = exp_circle(plt, out, chosen)
        laps = exp_laps(plt, out, chosen, pool)
        sweeps = exp_gain_sweeps(plt, out, base, pool)
        unstable = exp_kp_kd_map(plt, out, pd, pool)
        dterm = exp_d_term(plt, out, pd, pool)
        windup = exp_anti_windup(plt, out, base)
        delay = exp_delay(plt, out, pd)
        look, eq, gap = exp_lookahead(plt, out, chosen)
        wayp = exp_waypoint(plt, out, pool)
    gazebo = exp_gazebo(plt, out)

    def d(default, **special):
        return {**{k: default for k in ('rise', 'overshoot', 'settle', 'sse', 'max', 'peak',
                                        'final', 'ripple', 'rms_cte', 'rms_cte_sd', 'max_cte',
                                        'effort', 'saturated', 'rms', 'kp', 'ki', 'kd', 'tau',
                                        'value', 'lap rms', 'lap effort', 'lookahead')},
                **special}

    tables = {
        'tuning': md_table(tuning, {
            'error signal': 'error signal', 'type': 'type', 'tried': 'settings tried',
            'complete': 'complete all runs', 'meet specs': 'also meet the specs',
            'kp': 'kp', 'ki': 'ki', 'kd': 'kd', 'tau': 'tau [s]', 'rms': 'RMS error [m]',
            'effort': 'effort [deg/s]', 'overshoot': 'overshoot [%]',
            'settle': 'settling [s]'}, d(2, rms=3, effort=0, overshoot=1)),
        'step': md_table(step, {
            'error signal': 'error signal', 'gains': 'controller', 'rise': 'rise time [s]',
            'overshoot': 'overshoot [%]', 'settle': 'settling time [s]',
            'max': 'largest error [m]'}, d(2, overshoot=1)),
        'disturbance': md_table(dist, {
            'error signal': 'error signal', 'type': 'type', 'peak': 'peak error [mm]',
            'final': 'final error [mm]', 'ripple': 'final ripple [mm]'}, d(1)),
        'circle': md_table(circ, {
            'error signal': 'error signal', 'type': 'type', 'finished': 'completed',
            'final': 'steady error [mm]', 'ripple': 'ripple [mm]'}, d(1)),
        'laps': md_table(in_mm(laps), {'track': 'track', 'error signal': 'error signal',
                                       'type': 'type', **LAP_COLS}, d(0, saturated=1)),
        'gain_sweeps': md_table(
            [{**r, 'lap rms': r['lap rms'] * 1000.0} for name in ('kp', 'ki', 'kd')
             for r in [x for x in sweeps if x['gain'] == name][::2]],     # every second value
            {'gain': 'gain', 'value': 'value', 'rise': 'rise time [s]',
             'overshoot': 'overshoot [%]',
             'settle': 'settling time [s]', 'recovery': 'time to remove trim error [s]',
             'lap finished': 'laps finished', 'lap rms': 'lap RMS error [mm]',
             'lap effort': 'lap effort [deg/s]'},
            {**d(2, overshoot=1, value=3), 'lap rms': 0, 'lap effort': 0, 'recovery': 1}),
        'd_term': md_table(in_mm(dterm), {'D term': 'D term', **LAP_COLS}, d(0, saturated=1)),
        'anti_windup': md_table(windup, {
            'anti-windup': 'anti-windup', 'ki': 'ki', 'overshoot': 'overshoot [%]',
            'settle': 'settling time [s]', 'largest integral term': 'largest integral term [deg]',
            'saturated': 'saturated [%]'}, {**d(1, ki=2), 'largest integral term': 0}),
        'delay': md_table([r for r in delay if r['delay_ms'] in (0, 50, 100, 150, 200, 300)], {
            'delay_ms': 'delay [ms]', 'servo_tau': 'servo lag [s]', 'overshoot': 'overshoot [%]',
            'settle': 'settling time [s]'}, d(1, servo_tau=2)),
        'lookahead': md_table(look, {
            'lookahead': 'look-ahead [m]', 'rise': 'rise time [s]', 'overshoot': 'overshoot [%]',
            'settle': 'settling time [s]'}, d(2, overshoot=1, lookahead=1)),
        'waypoint': md_table(in_mm(wayp), {
            'controller': 'controller', 'finished': 'laps finished in 300 s',
            'lap_time': 'lap time [s]', 'heading_error': 'mean heading error [rad]',
            'rms_cte': 'RMS error [mm]', 'saturated': 'saturated [%]'},
            {**d(0, saturated=0), 'lap_time': 0, 'heading_error': 2}),
    }
    if gazebo:
        tables['gazebo'] = md_table(in_mm(gazebo), {
            'controller': 'controller', 'outcome': 'outcome', **LAP_COLS}, d(0, saturated=1))
    with open(os.path.join(out, 'results', 'tables.md'), 'w') as f:
        for name, table in tables.items():
            f.write(f'## {name}\n\n{table}\n\n')
        f.write(f'## notes\n\nkp-kd map: {unstable:.0f} % of the cells never settle.\n'
                f'look-ahead equivalence: cross-track PD gains kp = {eq[0]:.3f}, '
                f'kd = {eq[2]:.3f}; largest difference between the two responses '
                f'{gap * 1000.0:.1f} mm.\n')
    fill_tables(os.path.join(out, 'README.md'), tables)
    print('tables written to', os.path.relpath(os.path.join(out, 'results', 'tables.md')))


if __name__ == '__main__':
    main()
