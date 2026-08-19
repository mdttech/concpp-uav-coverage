import re
import ast
import os
import sys


# ============================================================ per-run parsing
# Identical logic to verify_multibase_run.py -- not reinvented, just reused,
# so results here are guaranteed consistent with every individual PASS/FAIL
# already confirmed earlier.

def in_range(cell, network, comm_range):
    cx, cy = cell
    return any((cx - nx) ** 2 + (cy - ny) ** 2 <= comm_range ** 2 for nx, ny in network)


def analyze(log_path, network, comm_range):
    """log_path may be a single combined log file (e.g. from `tee`), OR a
    directory -- in which case every *.log file inside it is read and
    merged in filename order before parsing. This second form is what
    ROS 2's own ~/.ros/log/<session>/ directories look like: one file per
    node (world_sim-1.log, cp_node-2.log, ...) rather than one interleaved
    stream, since ROS splits per-node output by default regardless of
    whether a `tee` was also used on the combined launch stdout."""
    robots_seen = set()
    violations = []
    total_cells_checked = 0
    max_t_start = 0
    round_count = 0
    completed = False
    completion_ts = None
    first_ts = None
    map_dims = None
    free_cells = None

    ts_pattern = re.compile(r'\[(\d+)\.\d+\]')
    map_pattern = re.compile(r'Loaded map .*?: (\d+)x(\d+), (\d+) free cells')

    if os.path.isdir(log_path):
        files = sorted(
            os.path.join(log_path, f) for f in os.listdir(log_path)
            if f.endswith('.log')
        )
    else:
        files = [log_path]

    def all_lines():
        for fp in files:
            with open(fp, errors='replace') as f:
                yield from f

    for line in all_lines():
        m = ts_pattern.search(line)
        ts = int(m.group(1)) if m else None
        if ts is not None and first_ts is None:
            first_ts = ts

        mm = map_pattern.search(line)
        if mm:
            map_dims = (int(mm.group(1)), int(mm.group(2)))
            free_cells = int(mm.group(3))

        if 'round full paths' in line:
            m2 = re.search(r'\(t_start=(\d+)\): (\{.*\})', line)
            if not m2:
                continue
            t_start = int(m2.group(1))
            sigma = ast.literal_eval(m2.group(2))
            if sigma:
                round_count += 1
            for rid, cells in sigma.items():
                robots_seen.add(rid)
                for i, cell in enumerate(cells):
                    cell = tuple(cell)
                    total_cells_checked += 1
                    max_t_start = max(max_t_start, t_start + i)
                    if not in_range(cell, network, comm_range):
                        violations.append((rid, cell, t_start + i))

        if 'COVERAGE COMPLETE' in line:
            completed = True
            if ts is not None:
                completion_ts = ts

    return {
        'robots_seen': sorted(robots_seen),
        'num_robots': len(robots_seen),
        'round_count': round_count,
        'cells_checked': total_cells_checked,
        'violations': len(violations),
        'violation_detail': violations,
        'completed': completed,
        'final_clk': max_t_start,
        'wall_seconds': (completion_ts - first_ts) if (completion_ts and first_ts) else None,
        'map_dims': map_dims,
        'free_cells': free_cells,
    }


# ==================================================================== manifest
# Every test configuration given throughout this conversation. Paths match
# the exact filenames used in the `tee` commands provided each time -- if
# you saved a log somewhere else, edit the 'log' field for that entry.

MANIFEST = [
    # --- original 6-config matrix ---
    {'log': '~/concpp_logs/t1.log', 'label': 'T1', 'map': 'small_test',
     'bases': [(5, 5)], 'robots_per_base': [4], 'comm_range': 3.0, 'pattern': '1-base'},
    {'log': '~/concpp_logs/t2.log', 'label': 'T2', 'map': 'test_16x16',
     'bases': [(2, 2), (13, 12)], 'robots_per_base': [3, 3], 'comm_range': 4.0, 'pattern': 'disconnected'},
    {'log': '~/concpp_logs/t3.log', 'label': 'T3', 'map': 'test_16x16',
     'bases': [(5, 5), (10, 9)], 'robots_per_base': [3, 3], 'comm_range': 4.0, 'pattern': 'overlapping'},
    {'log': '~/concpp_logs/t4.log', 'label': 'T4', 'map': 'test_32x32',
     'bases': [(2, 2), (15, 29), (29, 2)], 'robots_per_base': [2, 3, 4], 'comm_range': 4.0, 'pattern': 'skewed-alloc'},
    {'log': '~/concpp_logs/t5.log', 'label': 'T5', 'map': 'test_32x32',
     'bases': [(2, 2), (28, 28)], 'robots_per_base': [3, 3], 'comm_range': 2.0, 'pattern': 'small-range'},
    {'log': '~/concpp_logs/t6.log', 'label': 'T6', 'map': 'test_64x64',
     'bases': [(2, 2), (2, 61), (61, 2), (61, 61)], 'robots_per_base': [2, 2, 2, 2], 'comm_range': 6.0, 'pattern': '4-base'},

    # --- overlapping-range set ---
    {'log': '~/concpp_logs/u1.log', 'label': 'U1', 'map': 'test_16x16',
     'bases': [(5, 5), (10, 9)], 'robots_per_base': [1, 1], 'comm_range': 4.0, 'pattern': 'overlapping'},
    {'log': '~/concpp_logs/u2.log', 'label': 'U2', 'map': 'test_32x32',
     'bases': [(8, 8), (16, 8), (12, 16)], 'robots_per_base': [1, 1, 1], 'comm_range': 6.0, 'pattern': 'overlapping'},
    {'log': '~/concpp_logs/u3.log', 'label': 'U3', 'map': 'test_64x64',
     'bases': [(20, 20), (28, 20), (19, 28), (28, 28)], 'robots_per_base': [1, 1, 1, 1], 'comm_range': 8.0, 'pattern': 'overlapping'},

    # --- 12-config 3-base pattern matrix ---
    {'log': '~/concpp_logs/s8_sep.log', 'label': 'S8-sep', 'map': 'test_8x8',
     'bases': [(0, 1), (2, 5), (6, 7)], 'robots_per_base': [1, 1, 1], 'comm_range': 1.5, 'pattern': 'separate'},
    {'log': '~/concpp_logs/s8_int.log', 'label': 'S8-int', 'map': 'test_8x8',
     'bases': [(6, 7), (7, 5), (5, 5)], 'robots_per_base': [1, 1, 1], 'comm_range': 1.5, 'pattern': 'intersect'},
    {'log': '~/concpp_logs/s8_mix.log', 'label': 'S8-mix', 'map': 'test_8x8',
     'bases': [(2, 2), (4, 4), (7, 0)], 'robots_per_base': [1, 1, 1], 'comm_range': 1.5, 'pattern': 'mixed'},
    {'log': '~/concpp_logs/s16_sep.log', 'label': 'S16-sep', 'map': 'test_16x16',
     'bases': [(8, 2), (4, 10), (7, 15)], 'robots_per_base': [1, 1, 1], 'comm_range': 2.5, 'pattern': 'separate'},
    {'log': '~/concpp_logs/s16_int.log', 'label': 'S16-int', 'map': 'test_16x16',
     'bases': [(8, 11), (12, 10), (8, 10)], 'robots_per_base': [1, 1, 1], 'comm_range': 2.5, 'pattern': 'intersect'},
    {'log': '~/concpp_logs/s16_mix.log', 'label': 'S16-mix', 'map': 'test_16x16',
     'bases': [(4, 8), (3, 12), (15, 7)], 'robots_per_base': [1, 1, 1], 'comm_range': 2.5, 'pattern': 'mixed'},
    {'log': '~/concpp_logs/s32_sep.log', 'label': 'S32-sep', 'map': 'test_32x32',
     'bases': [(12, 5), (26, 22), (25, 30)], 'robots_per_base': [1, 1, 1], 'comm_range': 4.0, 'pattern': 'separate'},
    {'log': '~/concpp_logs/s32_int.log', 'label': 'S32-int', 'map': 'test_32x32',
     'bases': [(16, 10), (15, 4), (11, 10)], 'robots_per_base': [1, 1, 1], 'comm_range': 4.0, 'pattern': 'intersect'},
    {'log': '~/concpp_logs/s32_mix_v2.log', 'label': 'S32-mix', 'map': 'test_32x32',
     'bases': [(27, 2), (26, 4), (13, 10)], 'robots_per_base': [1, 1, 1], 'comm_range': 4.0, 'pattern': 'mixed'},
    {'log': '~/concpp_logs/s64_sep.log', 'label': 'S64-sep', 'map': 'test_64x64',
     'bases': [(58, 10), (39, 47), (48, 62)], 'robots_per_base': [1, 1, 1], 'comm_range': 6.0, 'pattern': 'separate'},
    {'log': '~/concpp_logs/s64_int.log', 'label': 'S64-int', 'map': 'test_64x64',
     'bases': [(44, 6), (42, 10), (40, 13)], 'robots_per_base': [1, 1, 1], 'comm_range': 6.0, 'pattern': 'intersect'},
    {'log': '~/concpp_logs/s64_mix.log', 'label': 'S64-mix', 'map': 'test_64x64',
     'bases': [(42, 21), (50, 18), (44, 59)], 'robots_per_base': [1, 1, 1], 'comm_range': 6.0, 'pattern': 'mixed'},

    # --- real benchmark maps ---
    {'log': '~/concpp_logs/m128_sep.log', 'label': 'M128-sep', 'map': 'maze-128-128-2',
     'bases': [(61, 66), (36, 119), (16, 86)], 'robots_per_base': [1, 1, 1], 'comm_range': 10.0, 'pattern': 'separate'},
    {'log': '~/concpp_logs/m128_int.log', 'label': 'M128-int', 'map': 'maze-128-128-2',
     'bases': [(34, 106), (37, 105), (44, 95)], 'robots_per_base': [1, 1, 1], 'comm_range': 10.0, 'pattern': 'intersect'},
    {'log': '~/concpp_logs/m128_mix.log', 'label': 'M128-mix', 'map': 'maze-128-128-2',
     'bases': [(43, 22), (37, 12), (26, 94)], 'robots_per_base': [1, 1, 1], 'comm_range': 10.0, 'pattern': 'mixed'},
    {'log': '~/concpp_logs/m256_sep.log', 'label': 'M256-sep', 'map': 'Milan_1_256',
     'bases': [(20, 127), (114, 143), (216, 96)], 'robots_per_base': [1, 1, 1], 'comm_range': 15.0, 'pattern': 'separate'},
    {'log': '~/concpp_logs/m256_int.log', 'label': 'M256-int', 'map': 'Milan_1_256',
     'bases': [(197, 149), (186, 157), (188, 138)], 'robots_per_base': [1, 1, 1], 'comm_range': 15.0, 'pattern': 'intersect'},
    {'log': '~/concpp_logs/m256_mix.log', 'label': 'M256-mix', 'map': 'Milan_1_256',
     'bases': [(182, 203), (193, 225), (188, 26)], 'robots_per_base': [1, 1, 1], 'comm_range': 15.0, 'pattern': 'mixed'},
]


def main():
    found, missing = [], []
    for cfg in MANIFEST:
        log_path = os.path.expanduser(cfg['log'])
        if os.path.exists(log_path):
            r = analyze(log_path, cfg['bases'], cfg['comm_range'])
            found.append({**cfg, **r})
        else:
            missing.append(cfg)

    print('=' * 78)
    print(f'AGGREGATE LOG ANALYSIS -- {len(found)} of {len(MANIFEST)} expected runs found')
    print('=' * 78)

    if missing:
        print(f'\n{len(missing)} log(s) not found (never run, or saved elsewhere):')
        for cfg in missing:
            print(f"  {cfg['label']:20s} expected at {cfg['log']}")

    if not found:
        print('\nNo logs found to analyze -- nothing further to report.')
        return

    # -------------------------------------------------- per-run table
    print(f'\n{"-"*88}\nPER-RUN RESULTS\n{"-"*88}')
    hdr = f'{"Label":22s} {"Map":16s} {"Pattern":12s} {"Robots":7s} {"Rounds":7s} {"FinalCLK":9s} {"Viol.":6s} {"Done":5s}'
    print(hdr)
    print('-' * len(hdr))
    for r in found:
        print(f"{r['label']:22s} {r['map']:16s} {r['pattern']:12s} "
              f"{r['num_robots']:<7d} {r['round_count']:<7d} {r['final_clk']:<9d} "
              f"{r['violations']:<6d} {'Y' if r['completed'] else 'N':5s}")

    # -------------------------------------------------- correctness summary
    total_cells = sum(r['cells_checked'] for r in found)
    total_violations = sum(r['violations'] for r in found)
    all_completed = all(r['completed'] for r in found)
    print(f'\n{"-"*78}\nCORRECTNESS SUMMARY\n{"-"*78}')
    print(f'Total path cells checked across all runs: {total_cells}')
    print(f'Total range violations found:              {total_violations}')
    print(f'All runs reached COVERAGE COMPLETE:        {all_completed}')
    incomplete = [r['label'] for r in found if not r['completed']]
    if incomplete:
        print(f'  did NOT complete: {incomplete}')

    # -------------------------------------------------- pattern comparison
    print(f'\n{"-"*78}\nPATTERN COMPARISON (avg final CLK, avg rounds -- excludes 1-base/skewed/small-range)\n{"-"*78}')
    by_pattern = {}
    for r in found:
        if r['pattern'] in ('separate', 'intersect', 'mixed', 'overlapping'):
            by_pattern.setdefault(r['pattern'], []).append(r)
    for pattern, rows in sorted(by_pattern.items()):
        avg_clk = sum(x['final_clk'] for x in rows) / len(rows)
        avg_rounds = sum(x['round_count'] for x in rows) / len(rows)
        print(f'  {pattern:12s} n={len(rows):2d}  avg final_clk={avg_clk:6.1f}  avg rounds={avg_rounds:5.1f}')

    # -------------------------------------------------- scale trend
    print(f'\n{"-"*78}\nSCALE TREND (final CLK vs. free-cell count, where map data was logged)\n{"-"*78}')
    scale_rows = [r for r in found if r['free_cells']]
    scale_rows.sort(key=lambda r: r['free_cells'])
    for r in scale_rows:
        print(f"  {r['label']:20s} free_cells={r['free_cells']:6d}  final_clk={r['final_clk']:4d}  "
              f"robots={r['num_robots']}")

    # -------------------------------------------------- optional chart
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(13, 5))

        if scale_rows:
            axes[0].scatter([r['free_cells'] for r in scale_rows],
                             [r['final_clk'] for r in scale_rows], color='#1B2A4A')
            for r in scale_rows:
                axes[0].annotate(r['label'], (r['free_cells'], r['final_clk']),
                                  fontsize=7, xytext=(3, 3), textcoords='offset points')
            axes[0].set_xlabel('Free cells in map')
            axes[0].set_ylabel('Final CLK (mission time)')
            axes[0].set_title('Mission time vs. map scale')

        if by_pattern:
            patterns = sorted(by_pattern.keys())
            avgs = [sum(x['final_clk'] for x in by_pattern[p]) / len(by_pattern[p]) for p in patterns]
            axes[1].bar(patterns, avgs, color='#1B2A4A')
            axes[1].set_ylabel('Avg final CLK')
            axes[1].set_title('Mission time by base-range pattern')

        plt.tight_layout()
        plt.savefig('/tmp/log_insights.png', dpi=130)
        print(f'\nChart saved to /tmp/log_insights.png')
    except ImportError:
        print('\n(matplotlib not available -- skipping chart, text summary above still complete)')


if __name__ == '__main__':
    main()
