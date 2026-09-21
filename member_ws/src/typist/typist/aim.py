"""Aim the stylus at a world target: FK, aim inverse, base search.

Pure math from INTERFACES.md sections 2-3. No ROS, no simulator imports.
"""

import math

BASE_H = 0.30
L1 = 0.60
L2 = 0.40

Q_MIN = [-2.0944, -0.5236, -2.4435, -0.7854, -0.6109]
Q_MAX = [2.0944, 1.7453, 0.0, 0.7854, 0.6109]

STYLUS_MIN = 0.05
STYLUS_MAX = 0.35
MAX_INCIDENCE_DEG = 55.0


def forward_head(q):
    """Head origin and head-frame axes in world for joints [yaw, sh, el]."""
    c0, s0 = math.cos(q[0]), math.sin(q[0])
    t1, phi = q[1], q[1] + q[2]
    elbow = (
        L1 * math.cos(t1) * c0,
        L1 * math.cos(t1) * s0,
        BASE_H + L1 * math.sin(t1),
    )
    head = (
        elbow[0] + L2 * math.cos(phi) * c0,
        elbow[1] + L2 * math.cos(phi) * s0,
        elbow[2] + L2 * math.sin(phi),
    )
    xh = (math.cos(phi) * c0, math.cos(phi) * s0, math.sin(phi))
    yh = (-s0, c0, 0.0)
    zh = (-math.sin(phi) * c0, -math.sin(phi) * s0, math.cos(phi))
    return head, (xh, yh, zh)


def aim_pan_tilt(head, axes, target):
    """Pan/tilt/range aiming the head-frame stylus at target (exact inverse)."""
    xh, yh, zh = axes
    d = [target[i] - head[i] for i in range(3)]
    rng = math.sqrt(sum(v * v for v in d))
    d = [v / rng for v in d]
    dx = sum(d[i] * xh[i] for i in range(3))
    dy = sum(d[i] * yh[i] for i in range(3))
    dz = sum(d[i] * zh[i] for i in range(3))
    pan = math.atan2(dy, dx)
    tilt = math.asin(max(-1.0, min(1.0, dz)))
    return pan, tilt, rng


def solve_for_key(target, panel_normal, step=0.05):
    """Full 5-joint solution with incidence/range/limit checks, or None.

    Searches base/shoulder/elbow coarsely; pan/tilt come from the exact aim
    inverse. Prefers near-upright panels (low incidence), then near-home base.
    """
    best = None
    yaw = 0.0
    while yaw <= Q_MAX[0] + 1e-9:
        for sign in (1.0, -1.0):
            y = sign * yaw
            sh = Q_MIN[1]
            while sh <= Q_MAX[1] + 1e-9:
                el = Q_MIN[2]
                while el <= Q_MAX[2] + 1e-9:
                    head, axes = forward_head([y, sh, el])
                    pan, tilt, rng = aim_pan_tilt(head, axes, target)
                    if (
                        Q_MIN[3] <= pan <= Q_MAX[3]
                        and Q_MIN[4] <= tilt <= Q_MAX[4]
                        and STYLUS_MIN <= rng <= STYLUS_MAX
                    ):
                        xh, yh, zh = axes
                        ct = math.cos(tilt)
                        aim = (
                            xh[0] * ct * math.cos(pan)
                            + yh[0] * ct * math.sin(pan)
                            + zh[0] * math.sin(tilt),
                            xh[1] * ct * math.cos(pan)
                            + yh[1] * ct * math.sin(pan)
                            + zh[1] * math.sin(tilt),
                            xh[2] * ct * math.cos(pan)
                            + yh[2] * ct * math.sin(pan)
                            + zh[2] * math.sin(tilt),
                        )
                        inc = math.degrees(
                            math.acos(
                                max(
                                    -1.0,
                                    min(
                                        1.0,
                                        -(
                                            aim[0] * panel_normal[0]
                                            + aim[1] * panel_normal[1]
                                            + aim[2] * panel_normal[2]
                                        ),
                                    ),
                                )
                            )
                        )
                        if inc <= MAX_INCIDENCE_DEG:
                            err = abs(y) * 2.0 + abs(sh - 1.1) + abs(el + 1.5)
                            key = (inc, err, rng)
                            if best is None or key < best[0]:
                                best = (key, [y, sh, el, pan, tilt], rng, inc)
                    el += step
                sh += step
        yaw += step
    if best is None:
        return None
    (_, _, _), q, rng, inc = best
    return q, rng, inc
