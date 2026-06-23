# Live Demo Plan (optional but powerful)

A short live demo makes "I've already built it" undeniable. Keep it simple and safe — show the
part that reliably works (**autonomous navigation**), and *narrate* the rest.

## What to show (in priority order)
1. **Autonomous navigation (the reliable win).** Launch the sim with Gazebo + RViz, send the
   robot from spawn to the workstation, and let him watch the Ackermann base plan a smooth,
   feasible path and drive there. This is solid and visually convincing.
2. **Perception + planning (show, don't risk).** Show the camera view / detected block and the
   MoveIt planning scene in RViz. Explain the grasp pipeline.
3. **The grasp attempt + the honest caveat.** If you run the full pick, narrate the depth-bias
   finding rather than promising a clean lift. Turning a glitch into "here's the sim-to-real
   issue I measured" is a *strength*.

## Pre-meeting checklist (do this BEFORE he's watching)
- [ ] Bring the sim up **staged** (Gazebo first, then RViz a few seconds later) — starting both
      at once makes RViz crash on this GPU. (Known issue; staged start avoids it.)
- [ ] Confirm the robot spawns and Nav2 is active before he arrives.
- [ ] Have the **exposé** (`docs/proposal/expose_final.tex` or `docs/proposal/expose_final.pdf`) open as backup if the demo hiccups.
- [ ] Have one screenshot/screen-recording of a successful navigation run as a fallback, in case
      live Gazebo misbehaves.

## What to say while it runs
- During navigation: *"This is the car-like base planning a feasible path — note it never turns
  in place, which is the constraint that makes the final approach interesting."*
- During perception: *"Colour + depth gives the object pose; here's where I measured the depth
  over-reporting at close range."*
- If anything stalls: *"This is exactly one of the failure modes I'm characterising —"* and move on.

## Golden rule
**Never let a live failure look like a surprise.** If you've already named it as a known finding,
a hiccup *reinforces* your credibility instead of hurting it.
