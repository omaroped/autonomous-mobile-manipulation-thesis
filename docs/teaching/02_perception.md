# 02 — Perception: turning a camera image into "the box is *here*"

**Goal of this layer:** produce one small message — *"the box is at (x, y, z) in the arm's
frame"* — published on the topic `/box_pose`. Everything downstream (the grasp) consumes that
single fact. The node that does it is
`src/ros2_ws/src/limo_ros2/limo_car/scripts/box_pose_estimator.py`.

We deliberately use **classical computer vision** (colour + depth), *not* deep learning. For a
known, brightly-coloured object this is robust, transparent, needs no training data, and is
easy to debug — and an earlier attempt that used a heavy PCL/segmentation pipeline never
reached a working pick. (See the project's "simple beats complex" principle.)

---

## 2.1 What the camera gives us

The simulated **depth camera** publishes three topics (Gazebo's camera plugin):

| Topic | Type | Meaning |
|-------|------|---------|
| `/rgb/image_raw` | `sensor_msgs/Image` | the colour picture (a grid of pixels) |
| `/depth_camera/depth/image_raw` | `sensor_msgs/Image` (32-bit float) | for each pixel, the **distance** to whatever it sees |
| `/depth_camera/depth/camera_info` | `sensor_msgs/CameraInfo` | the camera's **intrinsics**: `fx, fy` (focal lengths) and `cx, cy` (the image centre) |

A normal colour camera throws away depth — it only knows *direction*, not *distance*. A
**depth (RGB-D) camera** also gives, per pixel, how far away that point is. With *both* you can
reconstruct a true 3-D point. That reconstruction is the heart of this chapter.

---

## 2.2 The five steps (with the maths)

### Step 1 — Find the box (HSV colour segmentation)
The box is vivid blue. We convert the image from RGB to **HSV** (Hue, Saturation, Value),
because in HSV "blueness" is mostly one number (Hue), almost independent of lighting — far more
robust than thresholding raw R/G/B. We keep only pixels whose Hue/Sat/Val fall in the blue
range, clean up the speckle with morphological *open*/*close*, find the blue blob, and take its
**centroid pixel** `(u, v)`.

```
HSV blue range used:  lower = [110, 150, 50]   upper = [130, 255, 255]
```

### Step 2 — How far is it? (depth lookup)
Read the depth image at that same pixel `(u, v)` → the distance **Z** (we median a small patch
to reject noise). Now we know *which pixel* the box is and *how far* it is.

### Step 3 — De-projection: pixel + depth → a real 3-D point
A camera *projects* the 3-D world onto a 2-D grid of pixels. De-projection *undoes* that, using
the intrinsics. For a pixel `(u, v)` at depth `Z`, the 3-D point in the **camera's** frame is:

```
X = (u − cx) · Z / fx
Y = (v − cy) · Z / fy
Z = Z
```

Intuition: `(u − cx)` is how far right of centre the pixel is, in *pixels*; dividing by the
focal length `fx` converts "pixels" into an *angle*; multiplying by the distance `Z` converts
that angle into *metres sideways*. Same for vertical. `Z` straight ahead is the depth itself.
That's the whole trick — a pixel is a *ray*, and depth tells you *how far along the ray*.

### Step 4 — Move it into the arm's frame (TF)
That `(X, Y, Z)` is in **camera** coordinates. The arm plans in **`base_link`**. We ask TF for
the transform from the camera's optical frame to `base_link` and apply it. Now the point is
expressed where the arm can use it.

> The camera's *optical* frame follows a specific convention (z forward, x right, y down — REP
> 103). A real bug here: the camera plugin originally stamped its images with a frame name that
> **didn't exist in the TF tree**, so this transform threw an error and perception produced
> nothing. Fix: stamp with the real `depth_link` frame. *If a transform fails, perception is
> silently dead.*

### Step 5 — Publish `/box_pose`
We publish a `geometry_msgs/PoseStamped` on `/box_pose`: the box's position in `base_link`,
with an identity orientation (the *grasp* orientation is chosen later by the brain, not here).
We also publish a marker so you can *see* the estimate in RViz, and we **latch** the last good
detection (re-publish it at 10 Hz) so a momentary loss of sight doesn't starve the brain.

Watch it live:
```bash
ros2 topic echo /box_pose
# box @ base_link: x=0.218 y=0.010 z=0.012 (cam Z=0.118)
```

---

## 2.3 The honest limitation you must understand: the near-clip blind spot

Every camera has a **near-clip distance**: anything closer than that is *not rendered* — the
camera is physically blind to it. Ours is set low (5 cm) but it still matters, because of a
geometry clash that runs through the whole project:

- The myCobot 280 is short. The only place it can reach top-down is **~12–15 cm in front** of
  the base.
- But the camera is mounted **~10 cm forward** of the arm, so an object 12–15 cm from the *arm*
  is only ~2–5 cm from the *camera* — **inside the near-clip blind spot.**

The consequence, stated plainly: **the spot the arm can reach and the spot the camera can see
barely overlap.** This is *the* central reachability tension of the project. It is why, in the
isolated grasp test, we *bypassed* perception and fed the arm a known box position — not
because perception was broken, but because the reachable box sits where the camera can't see
it. Resolving it for the full pipeline forces a real design choice: move the camera back, lift
the object, or grasp from the side (see chapter 05 and the thesis "reachability" finding).

---

## 2.4 How to debug perception (the diagnostic mindset)

If `/box_pose` is wrong or empty, check the chain *in order* — each step depends on the one
before:

```bash
ros2 topic hz /rgb/image_raw                    # 1. is the camera publishing at all?
ros2 topic echo /depth_camera/depth/camera_info --once   # 2. are intrinsics present?
ros2 run rqt_image_view rqt_image_view          # 3. open /perception/..._debug to SEE the mask
ros2 run tf2_ros tf2_echo base_link depth_link  # 4. does the camera→arm transform resolve?
ros2 topic echo /box_pose                        # 5. final output
```
This "follow the data forward until it stops" method is the single most useful debugging habit
in the whole project.

**Next:** `03_moveit_and_grasping.md` — how the arm turns "the box is here" into motion, and
how it actually grips.
