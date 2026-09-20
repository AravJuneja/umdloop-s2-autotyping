# panel_detect

Finds the panel's four ArUco markers in the camera image and says where
their corners are. Detection only: no pose, no range, no board fit. Those
belong to whoever consumes `~/detections`.

## Running it

```bash
ros2 launch panel_detect panel_detect.launch.py   # this node and robot_state
```

The detector reads frames from `robot_state`, not from the camera, so the
launch file starts both. Running it alone (`ros2 run panel_detect
panel_detect`) works too, and sits silent until a `robot_state` appears.

It logs one line per five seconds:

```
frames=68 (13.6 Hz) skipped=0 complete=68 markers=272 mean=8.8 ms
```

## What it publishes

- `~/detections` — `MarkerDetections`: the camera's own stamp, the frame id,
  one `MarkerDetection` per marker found ascending by id, and `missing_ids`
  naming the panel markers this frame did not show. A consumer does not have
  to know that the panel carries ids 0 to 3 to notice one is gone.
- `~/overlay` — the frame with the markers outlined, each corner numbered in
  the order it is reported, the id above each marker, and a banner listing
  any missing ids. Only drawn when something is subscribed.
- `~/save_overlay` — `std_srvs/Trigger`. Writes the latest overlay as a PNG
  and returns the path. The directory is the `overlay_dir` parameter,
  `/tmp/panel_detect` by default, because a debugging call should not drop
  files into a working tree.

Corners are reported in the order the marker is **printed** — its own
top-left, top-right, bottom-right, bottom-left — not in the order they
happen to land in the image. OpenCV rotates the quad to match the decoded
orientation, so a marker seen upside down still reports its printed top-left
first and the correspondence with the board-frame table in the challenge
spec holds without a second guess.

## How precise the corners are

Two numbers, because they answer different questions.

**Against synthetic ground truth**, markers rendered through a known
homography at known sub-pixel offsets and areas-averaged down the way a
renderer's edges would be, at the 19–24 px the home pose gives us:

| refinement | centre RMS | centre p95 | centre bias | side bias |
|---|---|---|---|---|
| none | 0.197 px | 0.306 px | 0.010 px | −3.7% |
| sub-pixel | 0.126 px | 0.213 px | 0.012 px | −2.2% |
| **contour** (shipped) | **0.078 px** | **0.163 px** | 0.010 px | −4.8% |
| AprilTag | 0.645 px | 0.749 px | 0.641 px | −0.1% |

Every refinement pulls the marker's outline inward by a systematic fraction
of a pixel — that is the "side bias" column, and roughly half a pixel of it
is a convention difference about where a corner is rather than an error.
It cancels at the centre, which is why the centre is the column that
decides. It also matters less than it looks to a four-marker board fit: the
board's scale comes from the ~450 px between markers, not from the 21 px
across one. Contour refinement wins the centre by a factor of 1.6 over
sub-pixel; AprilTag is the only one that gets the outline right and the only
one that moves the centre.

`test_detector.py` re-runs that measurement and holds the p95 under
`MAX_CENTRE_ERROR_PX`. Sub-pixel refinement ignores
`cornerRefinementWinSize` entirely at this marker size — 2, 3 and 5 give
bit-identical results — so there is no window to tune.

**On the live simulator**, 150 frames over 10 s at `q_home`:

| quantity | std | peak-to-peak |
|---|---|---|
| marker centre, absolute | 0.58–0.65 px | 2.4–2.8 px |
| distance between two markers | 0.045–0.109 px | 0.22–0.49 px |

The two rows disagree by a factor of six because **the arm never stops
moving**. Nothing commands it, the watchdog holds the target velocity at
zero, and the joints still wander about 0.001 rad over ten seconds at up to
0.0065 rad/s. At a 900 px focal length that is a couple of pixels of image
motion, which is exactly the absolute spread above.

The distance between two markers is nearly immune to that, since a small
camera rotation moves every marker together. What survives is the detector,
and it repeats to about a tenth of a pixel. Read the first row as a bound on
the whole rig and the second as a bound on this package.

## Keeping up

The subscription is best-effort at depth one. `robot_state` publishes
`~/updates/image` reliable and transient-local, which a best-effort volatile
subscriber is compatible with, and the shallow queue is what makes the
middleware drop a frame we are not ready for instead of stacking it behind
us.

In practice we see 13.2–13.6 Hz of the camera's 15, at 8.8 ms of work per
frame. The loss is not us falling behind — 8.8 ms is room for 110 Hz — it is
2.7 MB frames being dropped in transit by a depth-one queue, which is the
behaviour we asked for. Detection stays current under it rather than
drifting further behind the camera as an episode runs.

## Tests

```bash
python3 -m pytest src/panel_detect/test
```

No simulator, no graph for most of it. `test/fixtures/` holds three real
camera frames, one from each of three separate episodes with the arm within
0.001 rad of `q_home`, so the panel sits somewhere different in each. The
partial-view case is a crop of a fixture made at test time rather than a
fourth file. Overlays written during tests go to pytest's `tmp_path`.

Two tests do stand up a real graph, on a domain id derived from the pid, to
check that a frame arriving on `robot_state`'s topic comes back out as
detections and that the save service writes the file it claims.

## Changing the contract

Every threshold lives in `panel_detect/config.py` — the dictionary, the
refinement, the precision bound, the topics, the QoS, the overlay's
colours. The numbers that came from a measurement sit next to a note saying
which measurement.
