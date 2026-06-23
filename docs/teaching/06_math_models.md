# 06 — The Mathematical Models, in one place

This collects the maths the project actually relies on, each explained for someone seeing it
for the first time. None of it is harder than secondary-school trigonometry plus the *idea* of
a rotation matrix.

---

## 6.1 Camera de-projection (pixel + depth → 3-D point)

A camera maps a 3-D point to a pixel by *projection*. De-projection reverses it using the
**intrinsics** `fx, fy` (focal lengths, in pixels) and `cx, cy` (the optical centre pixel).
For a pixel `(u, v)` at measured depth `Z`:

```
X = (u − cx) · Z / fx        (metres to the right)
Y = (v − cy) · Z / fy        (metres down/up)
Z = Z                        (metres forward)
```

**Why it works:** `(u − cx)` is the pixel offset from the centre; dividing by `fx` converts
pixels to a *tangent angle*; multiplying by depth `Z` converts that angle to *metres*. A pixel
is a ray out of the camera; depth says how far along the ray the surface is. (Used in
`box_pose_estimator.py`.)

---

## 6.2 Orientation: rotation matrices and quaternions

An **orientation** in 3-D needs 3 numbers of information, but representing it is subtle.

- A **rotation matrix** `R` (3×3) is the most concrete: its three columns are where the x, y, z
  axes of the rotated frame point, expressed in the parent frame. If a body's frame has axes
  `(x̂, ŷ, ẑ)` in the parent, then `R = [x̂ ŷ ẑ]`. This is exactly how we reasoned about the
  gripper: *"point the fingers (+y axis) straight down" → ŷ = (0,0,−1)*, then solved for the
  other columns and converted to a quaternion.
- A **quaternion** `q = (x, y, z, w)` is a 4-number encoding of the same rotation, used
  everywhere in ROS because it has no "gimbal lock" and interpolates cleanly. You rarely read
  one by eye; you build it from a rotation matrix or an axis–angle and trust it.

**The grasp orientations we used (all "gripper pointing down," different finger rolls):**

| Quaternion `(x,y,z,w)` | What it does | Reachable to 0.20 m? |
|---|---|---|
| `(1, 0, 0, 0)` | gripper *z* down, fingers one way | ✗ (over-twists when extended) |
| `(0.707, 0.707, 0, 0)` | gripper *z* down, fingers rolled 90° | ✓ |
| `(−0.707, 0, 0, 0.707)` | gripper **+y (the fingers) straight down** | ✓ — the correct grasp |

The headline lesson restated mathematically: *the same target position with two different
roll components of `q` can be the difference between an IK solution existing or not.*

**Rotation-matrix → quaternion** (the conversion `grasp_attacher.py` uses, trace method):
```
t = R[0,0] + R[1,1] + R[2,2]
if t > 0:  S = 2√(t+1);  w = S/4;  x = (R[2,1]−R[1,2])/S;  y = (R[0,2]−R[2,0])/S;  z = (R[1,0]−R[0,1])/S
else: pick the largest diagonal term and use the matching branch (see the code).
```

---

## 6.3 Reachability of a small arm (why ~0.28 m ≠ "reach 0.28 m")

The myCobot 280's "280" is ~0.28 m: the *maximum* reach with the arm stretched **straight
out**. That is **not** the reach for a **top-down** grasp, because to point the gripper *down*
the wrist must fold, which consumes reach. From the real link lengths (read out of
`mycobot_280_m5.urdf`):

- shoulder sits ~0.132 m above the arm base;
- upper-arm link ≈ 0.110 m, forearm link ≈ 0.116 m → the elbow-to-wrist span is only ~**0.226 m**.

To grasp a box ~0.24 m forward *while keeping the wrist above it pointing down*, the arm would
need to span more than that 0.226 m — so a **pure top-down** grasp at that distance has **no IK
solution**. The practical consequences (the project's "reachability is the binding constraint"
finding):

- Top-down reach is much shorter than the 0.28 m straight-out figure.
- It can be *shorter than the robot's own front overhang*, so a box the base can drive up to
  may sit beyond the arm's top-down envelope — forcing either an angled/side grasp, a
  camera/arm layout change, or careful base-stop placement.

**How we measured it (don't trust the datasheet, ask the solver):** sweep target poses through
`/compute_ik` and record which return `error_code = 1`. That builds an empirical reach map far
more trustworthy than hand arithmetic.

---

## 6.4 Forward kinematics (how a target becomes a gripper position)

A serial arm is a chain of frames; each joint `i` contributes a transform `Tᵢ(θᵢ)` (a rotation
by the joint angle composed with the fixed link offset). The gripper's pose in the base frame
is the product:

```
T_base→tcp(θ) = T₁(θ₁) · T₂(θ₂) · … · T₆(θ₆)
```

**Forward kinematics** evaluates this for given angles (one answer, easy). **Inverse
kinematics** inverts it — find `θ` so that `T_base→tcp(θ)` equals a desired pose (hard; KDL does
it numerically). MoveIt uses IK for the goal and then OMPL to find a collision-free path of
`θ` values to get there.

---

## 6.5 The weld's coordinate composition (rigid transforms)

A pose can be written as a 4×4 **homogeneous transform** `T = [[R, p],[0,1]]` (rotation `R`,
position `p`). Composing transforms is matrix multiplication; "the inverse pose" is the matrix
inverse. The weld needs the box to follow the gripper in the **world** frame, given that the
state plugin can't reference a link. So:

```
B            = world pose of the (pinned) base            ← from /get_entity_state('mbot','world'), constant
T_base→tcp   = base_footprint → gripper_tcp               ← from TF (changes as the arm moves)
gripper_world = B · T_base→tcp                            ← gripper pose in the world

At attach:   offset = (gripper_world)⁻¹ · box_world       ← box, expressed in the gripper frame
Every tick:  box_world = gripper_world · offset           ← put the box back at that offset → it follows
```

In the shipped node we use a lighter **position-follow** version (track the gripper's world
*position* and keep the box's orientation), which is enough for a straight lift-and-place. The
full pose-follow is the equation above. (See `grasp_attacher.py`.)

The single idea behind all of it: **`A → C` = `A → B` composed with `B → C`.** Chain the
transforms you *can* measure to get the one you *want*. That is also exactly what TF does for
you automatically inside one tree (chapter 02, step 4) — here we did it by hand across the
TF tree and the Gazebo world because they are two separate coordinate systems bridged only by
the pinned base.

---

### Closing thought
Almost every "hard" moment in this project reduced to one of two things: **a coordinate frame
we had wrong**, or **a constraint (collision, reach, orientation) we hadn't *measured*.** The
maths here is modest; the discipline of *checking* it against the live system is what actually
moved the project forward.
