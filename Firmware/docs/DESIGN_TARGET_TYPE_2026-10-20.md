# Target type selection: HUMAN, HEAD, CAT, DOG (design, 2026-10-20)

Owner request 2026-10-20: the operator chooses what the turret follows, from the web UI, in the
existing style; model switching keeps the outage short and the turret safe while it happens; the
existing candidate selection keeps working.

## Owner decisions (2026-10-20)

| Question | Decision |
|---|---|
| How many types at once | One: HUMAN, HEAD, CAT or DOG. |
| How a model change happens | Hot swap inside the running visiond: camera, sockets and the rest of the stack stay up. |
| The turret during a swap | Drop the target, brake and hold energised, no automatic mode changes; resume the mode it was in once the new model's frames arrive. |
| Where in the UI | A row of type chips at the head of the TARGETS drawer, above the candidate list. |
| HUMAN vs HEAD | HUMAN aims at the torso, HEAD at the head (pose keypoints). Default HEAD: today's behaviour. |
| Restart | The type persists, like the watch point (`run/state/target_type.json`). |
| Other classes | Hidden: only the chosen type is detected, listed, overlaid and selectable. |
| Change while tracking | Allowed: the target is dropped, the turret holds through the swap, then resumes. |

## What each type is

| Type | Model (profile) | Classes kept | Aim point |
|---|---|---|---|
| HUMAN | `hailo_yolov8s_pose` | person | torso (shoulders, hips; box fallback) |
| HEAD | `hailo_yolov8s_pose` | person | head (nose, eyes, ears; shoulders, then box fallback) |
| CAT | `hailo_yolov8n` COCO-80 | cat | box centre |
| DOG | `hailo_yolov8n` COCO-80 | dog | box centre |

HUMAN and HEAD share a model, so changing between them swaps nothing on the Hailo: only the aim
point and the tracks. CAT and DOG share the other model.

## Measured on the station (Hailo-8, stack stopped, 2026-10-20)

| | Time |
|---|---|
| Close the pose network, open the objects network, first frame | ~0.1 s (open 44-85 ms, close 35-39 ms, first frame 10-16 ms) |
| Both networks configured, swap by activation | 17 ms |
| Inference per frame | pose 8.5 ms, objects 6.8 ms |

The Hailo is not the outage. The outage is the tracker: new tracks need a few frames to confirm,
~0.3-0.5 s at 30 fps. So the swap closes one adapter and opens the other (the code that exists),
and no dual-network machinery is needed.

## Flow

1. **Operator** taps a chip: `set_target_type HEAD|HUMAN|CAT|DOG` on `/api/command`.
2. **controld** validates it, persists it, and if it differs from the type applied now enters
   *switching*:
   - remembers the mode to resume (AUTO_ROAM, SURVEILLANCE, AUTO_TRACK becomes the mode it falls
     back to after a loss, MANUAL stays MANUAL);
   - drops any selection and brakes to a hold, motors energised (the MANUAL hold path);
   - suppresses every automatic hand-off and refuses manual motion;
   - publishes `target_type_requested` in its telemetry, which visiond already reads (the
     expiring controller context).
3. **visiond** sees requested != applied and swaps: closes the adapter, builds the new profile's
   adapter (a new model only for HUMAN/HEAD <-> CAT/DOG), sets the kept classes and aim point,
   clears tracks and the selection, and from then on stamps every perception frame with the
   type it applied (native frame header, flag bits 2-4).
4. **controld** leaves *switching* when frames stamped with the requested type have been fresh
   for 0.3 s, and returns to the remembered mode.
5. **Failure** (no matching frames after 10 s): not a hazard, so not a fault (ruling 2026-10-02).
   The turret stays held, the page says why, and the operator can pick a type again.

Boot is Shutdown (no motion), so start-up needs no special path: visiond starts on the launcher's
profile and swaps to the persisted type as soon as it reads controld's context.

## Where the code changes

| Area | Change |
|---|---|
| `perception/configs/perception_v1.json` | `target_types`: profile, classes, aim for each type. |
| `perception/visiond.py`, `pipeline.py`, `model/adapter.py` | Swap on a context change; kept classes and aim set per type; tracks and selection cleared; applied type on every frame. |
| `perception/protocol/native_wire.py`, `control/src/tracks/perception_wire.hpp` | Applied type in header flag bits 2-4 (both sides ship together). |
| `perception/selection/control_context.py` | Read `target_type_requested`. |
| `control/src/control/control_loop.*`, `web/command_validation.hpp` | `set_target_type`; the *switching* hold and resume; persistence; telemetry. |
| `control/src/tracks/target_selection_manager.hpp` | Allowed classes from the type, not a constant. |
| `web/webd/hud.py` | Type chips in the TARGETS drawer, the switching state, the type in the header line. |

## Verification

- Unit tests on both sides: the type table, the swap on a context change, the header bits, the
  switching hold and resume, the 10 s HOLD, persistence, the HUD chips and command.
- Station: each change HEAD -> HUMAN -> CAT -> DOG -> HEAD, timed from the tap to the first
  selectable candidate, with the turret's position logged through the swap (it must not move);
  the resumed mode; click selection in each type. Cat and dog need a cat or a dog in view (a
  picture on a screen will do for a first check).
