import re
import ast
import sys
import argparse


def in_range(cell, network, comm_range):
    cx, cy = cell
    return any((cx - nx) ** 2 + (cy - ny) ** 2 <= comm_range ** 2 for nx, ny in network)


def analyze(log_path, network, comm_range):
    robots_seen = set()
    violations = []
    total_cells_checked = 0
    max_t_start = 0
    round_count = 0
    completed = False
    completion_line_ts = None
    first_line_ts = None

    ts_pattern = re.compile(r'\[(\d+)\.\d+\]')

    with open(log_path) as f:
        for line in f:
            m = ts_pattern.search(line)
            ts = int(m.group(1)) if m else None
            if ts is not None and first_line_ts is None:
                first_line_ts = ts

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
                    completion_line_ts = ts

    return {
        'robots_seen': sorted(robots_seen),
        'round_count': round_count,
        'total_cells_checked': total_cells_checked,
        'violations': violations,
        'completed': completed,
        'final_clk_estimate': max_t_start,
        'wall_seconds_estimate': (completion_line_ts - first_line_ts)
                                   if (completion_line_ts and first_line_ts) else None,
    }


def main():
    p = argparse.ArgumentParser(
        description='Verify a multi-base ConCPP run: checks every logged path cell '
                    'stays in range of the given network, and reports completion/timing.')
    p.add_argument('log_path')
    p.add_argument('--bases', required=True,
                    help='e.g. "2,2;28,28" -- semicolon-separated x,y pairs')
    p.add_argument('--comm-range', type=float, required=True)
    args = p.parse_args()

    network = []
    for pair in args.bases.split(';'):
        x, y = pair.split(',')
        network.append((int(x), int(y)))

    r = analyze(args.log_path, network, args.comm_range)

    print(f'Bases: {network}, comm_range={args.comm_range}')
    print(f'Robots seen in log: {r["robots_seen"]} ({len(r["robots_seen"])} total)')
    print(f'Non-empty rounds: {r["round_count"]}')
    print(f'Path cells checked: {r["total_cells_checked"]}')
    print(f'Reached COVERAGE COMPLETE: {r["completed"]}')
    print(f'Final CLK reached (mission-time proxy): {r["final_clk_estimate"]}')
    if r['wall_seconds_estimate'] is not None:
        print(f'Approx wall time to completion: {r["wall_seconds_estimate"]}s')

    if r['violations']:
        print(f'\n{len(r["violations"])} CROSS-ISLAND VIOLATION(S) FOUND:')
        for rid, cell, t in r['violations'][:20]:
            print(f'  robot {rid} at {cell} (t~{t}) -- out of range of every base')
        if len(r['violations']) > 20:
            print(f'  ... and {len(r["violations"]) - 20} more')
        print('\nVERDICT: FAIL')
    else:
        print('\nVERDICT: PASS -- every logged path cell stayed in range of the network')


if __name__ == '__main__':
    main()
