# Meeting Agenda & Script (you lead with this)

Keep it ~25–30 min. You drive; let him decide the scope question (Section 4) so he feels ownership.

---

## 0. Opening (1 min)
> "Thanks for the time. I've been building the mobile-manipulation system in simulation and
> it's come a long way — I wanted to show you where it is, walk through the real challenges
> I've run into, and agree with you on the final scope so I can lock it in and execute."

Sets the tone: you've *done work*, you *lead*, and you *want his input on scope* (not permission).

---

## 1. The project in one line (1 min)
> "An Agilex LIMO Cobot that autonomously navigates a small warehouse, detects a block,
> picks it with the myCobot arm, drives it to a workbench, and places it — developed in
> Gazebo first, then transferred to the real robot. The thesis question is really about the
> **simulation-to-real gap** on a low-cost, resource-constrained platform."

---

## 2. What already works (3–4 min) — show confidence
Walk through, briefly (have the demo from `04_demo_plan.md` ready):
- Full ROS 2 Humble + Gazebo sim; URDF + sensors audited against real specs.
- **Nav2 autonomously drives the car-like (Ackermann) base** to the workstation.
- MoveIt 2 collision-aware arm planning; HSV + depth perception → object pose.
- A coordinated state machine running the whole mission.

Message: *"The integration skeleton is done. The interesting part now is the hard problems."*

---

## 3. The real challenges I found (5 min) — your research credibility
Present these as **findings** (details in the exposé's "Preliminary Work" section):
1. **Non-holonomic final approach** — a car-like base can't centre on the object; needs a
   visual-docking stage Nav2 alone can't provide.
2. **Sim-to-real perception gap** — depth over-reports object distance at close range
   (~0.27 m vs true 0.18 m) → planning fails. Quantified.
3. **`/cmd_vel` arbitration** between navigation and docking controllers.
4. **Traction vs. stability trade-off** in the wheel-contact physics.
5. **Reachability coupling** — short 280 mm arm + solid table dictates where the base must stop.

> "These are exactly the kinds of issues that don't show up in papers but decide whether the
> system works. Characterising and solving them is, I think, the real contribution."

---

## 4. Scope decision (8–10 min) — **let him steer here** (use `02` + `03`)
Present the tiered scope and ask his preference. Lead with:
> "I want to be realistic about what's achievable in the thesis window. Here's how I'd tier it —
> I'd propose the single-block pick-place cycle as the core, and treat stacking/assembly as a
> stretch goal. Does that match your expectations, or would you push the bar somewhere else?"

Then use the questions in `03_questions_and_anticipated_QA.md` to get his call on:
- core vs. stretch boundary (esp. **stacking**),
- how much real-hardware vs. simulation weight,
- evaluation metrics he cares about.

---

## 5. Close — concrete agreements (2 min)
Before you leave, get explicit answers and write them down:
- ✅ Agreed **core deliverable**: ______________________
- ✅ **Stretch goal** in/out: ______________________
- ✅ **Sim vs. real** balance: ______________________
- ✅ Next check-in date: ______________________
- ✅ Any reading / people he suggests: ______________________

> "Great — I'll send you a one-page summary of what we agreed and update the exposé accordingly."

(Sending the summary afterwards = you leading the relationship, not just the meeting.)
