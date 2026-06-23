# 08 — Lessons Mined from the Sibling Project (`~/Desktop/Thesis`)

There is a second, earlier attempt at this same thesis in `~/Desktop/Thesis`. Architecturally it
went a different way — it leaned on **MoveIt Task Constructor (MTC) + a PCL point-cloud
perception pipeline** — and it **never reached a working pick**, which is *why* the current
project (`Thesisorg`) was started deliberately simpler (classical HSV perception, the direct
`move_group` action interface, no MTC). That is the headline lesson: *simple, debuggable, and
working beats sophisticated and stuck.*

**But** the old project kept an excellent **engineering problem-and-resolution log**
(now consolidated in [docs/engineering/TROUBLESHOOTING.md](file:///home/omar/Desktop/Thesisorg/docs/engineering/TROUBLESHOOTING.md), with the original archived under `docs/_archive/2026-06-19_reorg/`), and several of its findings are
genuinely valuable — some confirm what we independently discovered, two are things we had **not**
analysed, and one is a **cleaner fix than ours**. This chapter preserves that value so the folder
can be deleted without losing it.

---

## 8.1 The comparison table

| Their problem (their log) | Our experience | Verdict for us |
|---|---|---|
| **P1: Physics collapse — NaN inertia.** Manufacturer URDF gave `joint2` inertias violating the rigid-body triangle inequality ($I_{xx}+I_{yy}\ge I_{zz}$); the ODE solver returns NaN and the arm explodes. | We never hit this — **our arm inertias are all `1e-3` (equal, positive)**, which always satisfies the inequality. | ✅ **New concept to know**; we are already safe, but document it (§8.2). |
| **P2: Gripper linkage sag/disconnect.** Gazebo Classic ignores URDF `<mimic>`, so the parallel fingers free-spin and drift apart. Fix: declare the mimic in `<ros2_control>` so the hardware plugin drives the passive joints. | We have a **known, deferred gripper finger-asymmetry bug** (see project memory). Same root cause. | 🔧 **Actionable** — this is likely our gripper fix (§8.3). |
| **P3: Controller-manager race.** Spawners ran before the robot finished spawning → "controller manager not available," limp arm. Fix: sequence launch (gazebo → spawn → broadcaster → controllers) with event handlers. | We hit limp-arm / controller issues too (the EOL `gazebo_ros2_control` story). | 🔧 **Good practice** to verify in our launch (§8.4). |
| **P4: Arm–chassis frame disconnect.** Arm mounted on a legacy `g_base` adapter that doesn't exist physically → offset frames. Fix: mount `joint1` directly on the top plate; SRDF chain base = `joint1`. | We mount the arm at `base_link (-0.03,0,0.02)`, chain base `joint1`. Same approach, already correct. | ✅ Confirms our choice. |
| **P5: Uncommanded wheel drift.** Wheels rotate at zero command due to solver noise when **joint friction/damping are unspecified**. Fix: `<dynamics damping="0.1" friction="0.05"/>` on the wheel joints. | We fought exactly this (base creeping under arm reaction). We patched it with `base_pin` (kinematic hold). **Our wheel joints have no `<dynamics>`.** | 🔧 **Cleaner fix than ours** — adopt it (§8.5). |
| **P6: MoveIt self-collision.** Collision checker flagged the arm base against the chassis → failed/winding plans. Fix: `disable_collisions` for the arm-base/chassis pair. | This is the *same class* as our ⭐ collision-matrix bug (chapter 03), which we diagnosed far more deeply (the gripper/wrist vs wheels/laser/camera pairs). | ✅ Confirms our finding; ours is the more complete version. |

---

## 8.2 New concept: moments of inertia must be physically valid

A rigid body's inertia tensor diagonal must satisfy the **triangle inequalities**:
$$I_{xx}+I_{yy}\ge I_{zz},\quad I_{yy}+I_{zz}\ge I_{xx},\quad I_{zz}+I_{xx}\ge I_{yy}.$$
If a URDF link violates this (e.g. a manufacturer file with `0.0001+0.0001 < 0.0006`), Gazebo's
ODE solver evaluates an impossible tensor, returns NaN, and the model **explodes on spawn**.
*How to check:* for every link, confirm the three diagonal terms obey the inequalities; equal
positive values (our case, `1e-3`) are always valid. *This is a distinct failure mode from the
contact-explosion in chapter 03 — that one is a runtime contact force; this one is a bad model
that NaNs immediately on load.*

## 8.3 Actionable: the gripper mimic fix (likely our deferred gripper bug)

Gazebo Classic does **not** enforce URDF `<mimic>` joints — it treats the passive fingers as
free-spinning, so a parallel/adaptive gripper sags and the fingers lose sync. The robust fix is
to declare the mimic **inside `<ros2_control>`** so the hardware plugin computes and drives the
passive joints every control cycle:
```xml
<joint name="gripper_left3_to_gripper_left1">
  <param name="mimic">gripper_controller</param>
  <param name="multiplier">-1.0</param>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```
Our project has a *deferred* finger-asymmetry bug; this is the most likely correct fix. (Verify
the mimic exists in **both** the URDF joint tree and the `<ros2_control>` block — their
best-practice note 3.)

## 8.4 Good practice: sequence the launch with event handlers

Don't start controller spawners on a timer and hope; chain them on *events*:
1. start Gazebo + publish `robot_description`; 2. spawn the robot; 3. on spawn success, load
`joint_state_broadcaster`; 4. on broadcaster active, load the arm + gripper controllers. This
removes the race condition entirely (more reliable than fixed `sleep`/`TimerAction` delays).

## 8.5 Cleaner fix than ours: damp the wheels instead of pinning the base

We stopped the base drifting under the arm's reaction force with `base_pin.py` (a kinematic hold
via `/set_entity_state`). The sibling project's fix is more physical and simpler: give the wheel
joints real **friction and damping** so they don't creep in the first place —
```xml
<dynamics damping="0.1" friction="0.05" />
```
on each wheel joint in `limo_ackerman_base.xacro` (ours currently has none). This is worth
adopting: it fixes the *cause* (numerically slippery wheels) rather than masking the *symptom*,
and unlike `base_pin` it still lets the base be driven normally — so it works for the **full
navigate-then-grasp pipeline**, not just the isolated test. `base_pin` remains a useful belt-and-
braces hold during manipulation.

## 8.6 Structural idea worth borrowing: a module-based learning roadmap

Their original roadmap (now consolidated into [docs/engineering/STATUS.md](file:///home/omar/Desktop/Thesisorg/docs/engineering/STATUS.md) and archived under `docs/_archive/2026-06-19_reorg/roadmap.md`) was a 7-module beginner course (ROS 2 fundamentals → launch/sim →
URDF → architecture → Nav2/SLAM → MoveIt → writing nodes) with a CLI quick-reference and a
**progress tracker**. Our teaching folder covers the same material with more depth and the
manipulation war stories; the *progress-tracker* and *per-module exercises* idea is a nice
addition we could fold into this folder's README as a "study path with checkboxes."

---

## 8.7 Recommendation before deleting `~/Desktop/Thesis`

Safe to delete **after** these are captured (they are, here). Concretely worth *acting on* from
it:
1. **Adopt the wheel `<dynamics>`** (§8.5) — fixes base drift at the source. *Small, high value.*
2. **Apply the gripper mimic in `<ros2_control>`** (§8.3) — likely resolves our deferred gripper bug.
3. **Keep the inertia-validity check** (§8.2) in mind whenever a link is added.
4. Their original `roadmap.md` and `ENGINEERING_DATA_DOSSIER.md` have been consolidated into `docs/engineering/` and archived under `docs/_archive/2026-06-19_reorg/` for reference.
