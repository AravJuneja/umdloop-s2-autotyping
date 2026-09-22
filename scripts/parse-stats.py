"""Turn captured typist launch logs into per-episode timings.

Reads the logs ``e2e-stats.sh`` captured (one per launch code) and reports,
per completed episode: how long typing took, the cost per character, and the
cold-start latency from the projector's first publish to the first press.

Usage: python3 scripts/parse-stats.py [log-dir]
"""

import re
import statistics as st
import sys
from pathlib import Path

TIMESTAMP = re.compile(r'\[(\d{10})\.(\d+)\]')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
PRESS = re.compile(r'\]: [A-Z0-9]: q=')


def events(path):
    """Yield (time, kind, payload) for the lines that mark progress."""
    for raw in path.read_text(errors='replace').splitlines():
        line = ANSI.sub('', raw)
        stamp = TIMESTAMP.search(line)
        if not stamp:
            continue
        when = float(f'{stamp.group(1)}.{stamp.group(2)[:6]}')
        if 'published key positions' in line:
            yield when, 'publish', None
        elif PRESS.search(line):
            yield when, 'press', None
        elif 'typed ' in line and '/sim/done' in line:
            yield when, 'done', line.split('typed ')[1].split(';')[0]


def episodes(path):
    """One row per completed episode in a single code's log."""
    first_publish = previous_done = started = None
    presses = 0
    for when, kind, payload in events(path):
        if kind == 'publish' and first_publish is None:
            first_publish = when
        elif kind == 'press':
            presses += 1
            started = started or when
        elif kind == 'done' and started is not None:
            yield {
                'code': payload,
                'chars': presses,
                'typing': when - started,
                'per_char': (when - started) / max(presses, 1),
                'start': (started - first_publish)
                if previous_done is None and first_publish
                else None,
                'exact': payload == path.stem,
            }
            previous_done, started, presses = when, None, 0


def summarise(values, label):
    values = [v for v in values if v is not None]
    if not values:
        return
    spread = st.stdev(values) if len(values) > 1 else 0.0
    print(
        f'  {label:28s} n={len(values):2d}  mean={st.mean(values):6.1f}s  '
        f'sd={spread:4.1f}  min={min(values):6.1f}  max={max(values):6.1f}'
    )


def main():
    directory = Path(sys.argv[1] if len(sys.argv) > 1 else 'log/e2e-stats')
    rows = [row for path in sorted(directory.glob('*.log')) for row in episodes(path)]
    if not rows:
        print(f'no completed episodes found in {directory}')
        return 1

    print(f'{"code":8s} {"chars":>5s} {"typing":>7s} {"s/char":>7s} {"start":>7s}  exact')
    for row in sorted(rows, key=lambda r: (r['chars'], r['code'])):
        start = f'{row["start"]:7.1f}' if row['start'] else ' ' * 7
        print(
            f'{row["code"]:8s} {row["chars"]:5d} {row["typing"]:7.1f} '
            f'{row["per_char"]:7.2f} {start}  {"yes" if row["exact"] else "NO"}'
        )

    exact = sum(r['exact'] for r in rows)
    print(f'\nepisodes={len(rows)}  exact={exact}/{len(rows)}  codes={len({r["code"] for r in rows})}')
    summarise([r['typing'] + (r['start'] or 0) for r in rows], 'end to end')
    summarise([r['start'] for r in rows], 'cold start -> first press')
    summarise([r['typing'] for r in rows], 'typing time')
    summarise([r['per_char'] for r in rows], 'seconds per character')
    return 0 if exact == len(rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
