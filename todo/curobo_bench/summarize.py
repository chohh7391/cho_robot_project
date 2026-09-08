#!/usr/bin/env python3
"""Summarize and compare bench_planning.py CSVs.

    python3 todo/curobo_bench/summarize.py todo/curobo_bench/*.csv

With more than one pipeline present, the same rows are also printed as an
OMPL-vs-other comparison, which is what Step 3 of `todo/CUROBO_MOVEIT_TODO.md`
judges. Rows are grouped by (scene, goal_type); unreachable goals -- ones no
pipeline ever solves -- are reported separately so they do not drag the medians
of the solvable set.
"""

import argparse
import csv
import math
import statistics
import sys


def load(paths):
    rows = []
    for path in paths:
        with open(path, newline='') as handle:
            for row in csv.DictReader(handle):
                row['success'] = row['success'] == 'True'
                for key in ('planning_time_s', 'wall_time_s', 'traj_duration_s'):
                    row[key] = float(row.get(key, 'nan'))
                row['error_code'] = int(row['error_code'])
                rows.append(row)
    return rows


def quantile(values, fraction):
    if not values:
        return float('nan')
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def describe(rows):
    ok = [r for r in rows if r['success']]
    times = [r['planning_time_s'] for r in ok if not math.isnan(r['planning_time_s'])]
    durations = [r['traj_duration_s'] for r in ok if not math.isnan(r['traj_duration_s'])]
    failures = [r for r in rows if not r['success']]
    return {
        'n': len(rows),
        'ok': len(ok),
        'median_ms': statistics.median(times) * 1000 if times else float('nan'),
        'p95_ms': quantile(times, 0.95) * 1000 if times else float('nan'),
        'max_ms': max(times) * 1000 if times else float('nan'),
        'traj_s': statistics.median(durations) if durations else float('nan'),
        'fail_wall_s': (statistics.median([r['wall_time_s'] for r in failures])
                        if failures else float('nan')),
    }


def unsolvable_goals(rows):
    """Goal ids that never succeeded under any pipeline -- bad goals, not slow planners."""
    by_goal = {}
    for row in rows:
        by_goal.setdefault(row['goal_id'], []).append(row['success'])

    return {goal for goal, results in by_goal.items() if not any(results)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+')
    parser.add_argument('--keep-unsolvable', action='store_true',
                        help='include goals that no pipeline ever solves')
    args = parser.parse_args()

    rows = load(args.csv)
    if not rows:
        print('no rows')
        return 1

    dropped = set() if args.keep_unsolvable else unsolvable_goals(rows)
    kept = [r for r in rows if r['goal_id'] not in dropped]

    pipelines = sorted({r['pipeline'] for r in rows})
    scalings = sorted({r.get('scaling', '') for r in kept})
    groups = sorted({(r['scene'], r['goal_type'], r.get('scaling', ''))
                     for r in kept})

    show_scaling = len(scalings) > 1
    header = (f"{'scene':<16}{'goals':<7}{'pipeline':<18}"
              + (f"{'scale':>7}" if show_scaling else '')
              + f"{'ok':>8}{'median':>10}{'p95':>10}{'max':>10}"
                f"{'traj':>9}{'plan share':>12}")
    print(header)
    print('-' * len(header))
    for scene, goal_type, scaling in groups:
        for pipeline in pipelines:
            subset = [r for r in kept if r['scene'] == scene
                      and r['goal_type'] == goal_type and r['pipeline'] == pipeline
                      and r.get('scaling', '') == scaling]
            if not subset:
                continue
            s = describe(subset)
            share = (s['median_ms'] / 1000) / ((s['median_ms'] / 1000) + s['traj_s']) * 100 \
                if not math.isnan(s['traj_s']) else float('nan')
            print(f"{scene:<16}{goal_type:<7}{pipeline:<18}"
                  + (f"{scaling:>7}" if show_scaling else '')
                  + f"{s['ok']:>4}/{s['n']:<3}{s['median_ms']:>9.1f}ms"
                    f"{s['p95_ms']:>9.1f}ms{s['max_ms']:>9.1f}ms"
                    f"{s['traj_s']:>8.2f}s{share:>11.2f}%")

    if dropped:
        excluded = [r for r in rows if r['goal_id'] in dropped]
        print(f"\n제외한 목표 (어떤 플래너도 못 푼 것): {sorted(dropped)}")
        for scene in sorted({r['scene'] for r in excluded}):
            for pipeline in pipelines:
                subset = [r for r in excluded
                          if r['pipeline'] == pipeline and r['scene'] == scene]
                if not subset:
                    continue
                wall = statistics.median([r['wall_time_s'] for r in subset])
                print(f"  {scene:<16}{pipeline:<20} 실패 1건당 소요 중앙값 "
                      f"{wall:.2f} s  (n={len(subset)})")
        print('  -> 실패를 얼마나 빨리 포기하는가도 비교 항목이다.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
