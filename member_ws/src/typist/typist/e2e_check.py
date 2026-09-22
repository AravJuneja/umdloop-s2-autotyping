"""Run one typing episode and report the latched result.

Runs one episode against the already-running stack: waits for /sim/result,
prints target/typed/exact_match, exits 0 on exact match, 1 otherwise. Defaults
to a 90-second fail-fast timeout so a stalled episode stops the run instead of
hanging for ten minutes.
"""

import argparse
import csv
import os
import time

TIMEOUT_SEC = float(os.environ.get('E2E_TIMEOUT', '90'))
CSV_FIELDS = (
    'code',
    'seed',
    'exact_match',
    'attempted',
    'accepted',
    'rejections',
    'sim_elapsed_s',
    'wall_elapsed_s',
    'failure',
)


def _record_latest(received, message):
    received['r'] = message


def _result_row(code, result, wall_elapsed, failure):
    rejections = ';'.join(
        f'{reason}:{count}'
        for reason, count in zip(result.rejection_reasons, result.rejection_counts)
    )
    return {
        'code': code,
        'seed': str(result.seed),
        'exact_match': str(bool(result.exact_match)),
        'attempted': str(result.presses_attempted),
        'accepted': str(result.presses_accepted),
        'rejections': rejections,
        'sim_elapsed_s': f'{result.elapsed:.1f}',
        'wall_elapsed_s': f'{wall_elapsed:.1f}',
        'failure': failure,
    }


def _result_seed(node, message_type, qos, rclpy, timeout=15.0):
    """Read the currently latched /sim/result seed, or None if unlatched."""
    got = {}
    sub = node.create_subscription(
        message_type, '/sim/result', lambda message: _record_latest(got, message), qos
    )
    t0 = time.monotonic()
    while 'r' not in got and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    try:
        node.destroy_subscription(sub)
    except Exception:
        pass
    return got['r'].seed if 'r' in got else None


def main():
    import rclpy
    from autotype_msgs.msg import EpisodeResult
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--expect-new',
        action='store_true',
        help='subscribe first, then let the caller reset; the first result '
        'seen after subscribing starts this episode, so ignore any result '
        'already latched at startup.',
    )
    parser.add_argument(
        '--code',
        default='',
        help='launch code being typed; recorded in the CSV row only.',
    )
    parser.add_argument(
        '--csv',
        default='',
        help='append one result row to this CSV path (creates the file).',
    )
    args = parser.parse_args()
    rclpy.init()
    node = Node('e2e_check')
    t0 = time.monotonic()
    if args.expect_new:
        # Drain the latch replay (if any) before the reset happens.
        drained = _result_seed(node, EpisodeResult, latched, rclpy, timeout=min(15.0, TIMEOUT_SEC))
        if drained is not None:
            print(f'discarded pre-reset seed={drained}', flush=True)
    else:
        drained = _result_seed(node, EpisodeResult, latched, rclpy, timeout=min(15.0, TIMEOUT_SEC))
    if time.monotonic() - t0 >= TIMEOUT_SEC:
        print('E2E FAIL: no /sim/result within timeout')
        raise SystemExit(1)
    got = {}
    node.create_subscription(
        EpisodeResult,
        '/sim/result',
        lambda message: _record_latest(got, message),
        latched,
    )
    if args.expect_new:
        # Wait for any seed except the drained one (or any seed at all when
        # nothing was latched: the reset clears the latch, so the next
        # result is this episode's).
        while time.monotonic() - t0 < TIMEOUT_SEC:
            rclpy.spin_once(node, timeout_sec=0.5)
            if 'r' in got and got['r'].seed != drained:
                break
        else:
            print('E2E FAIL: no /sim/result within timeout')
            raise SystemExit(1)
    else:
        if drained is None:
            print('stale_seed=None; waiting for fresh result', flush=True)
        else:
            # Late subscriber: the latch replays instantly; discard it.
            t1 = time.monotonic()
            while 'r' not in got and time.monotonic() - t1 < 10.0:
                rclpy.spin_once(node, timeout_sec=0.5)
            if 'r' in got:
                print(f'discarded stale seed={got["r"].seed}', flush=True)
            got.pop('r', None)
        while time.monotonic() - t0 < TIMEOUT_SEC:
            rclpy.spin_once(node, timeout_sec=0.5)
            if 'r' in got and got['r'].seed != drained:
                break
        else:
            print('E2E FAIL: no /sim/result within timeout')
            raise SystemExit(1)
    if 'r' not in got or got['r'].seed == drained:
        print('E2E FAIL: no /sim/result within timeout')
        raise SystemExit(1)
    r = got['r']
    wall_elapsed = time.monotonic() - t0
    print(
        f'seed={r.seed} target={r.target} typed={r.typed} '
        f'exact={r.exact_match} attempted={r.presses_attempted} '
        f'accepted={r.presses_accepted} elapsed={r.elapsed:.1f}s'
    )
    if list(r.rejection_reasons):
        print(f'rejections: {list(zip(r.rejection_reasons, r.rejection_counts))}')
    if args.csv:
        failure = ''
        if not r.exact_match:
            failure = 'non-exact result'
        exists = os.path.exists(args.csv)
        os.makedirs(os.path.dirname(args.csv) or '.', exist_ok=True)
        with open(args.csv, 'a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            if not exists:
                writer.writeheader()
            writer.writerow(_result_row(args.code or r.target, r, wall_elapsed, failure))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if r.exact_match else 1)


if __name__ == '__main__':
    main()
