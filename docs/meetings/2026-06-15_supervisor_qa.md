# Questions to Ask + Anticipated Q&A

## Part A — Questions YOU ask him (to lead the dialogue & let him steer)
Asking good questions makes *you* look in control and gives *him* ownership of the decisions.

**On scope (the key one):**
1. "I'm proposing the single-block pick-transport-place cycle as the core, with stacking as a
   stretch goal — does that match your expectations for a Bachelor thesis, or would you set
   the bar differently?"
2. "Would you rather I go **deeper on robustness** of one reliable cycle, or **broader** toward
   the stacking/assembly demo?"

**On sim-to-real:**
3. "How much weight do you want on the **real-hardware** results vs. a thoroughly-evaluated
   simulation? Is a strong sim + a partial real demo acceptable if hardware time is limited?"

**On evaluation:**
4. "Which **metrics** matter most to you — task success rate, accuracy, cycle time, or the
   sim-to-real gap itself?"

**On positioning:**
5. "Should I frame the contribution around the **integrated low-cost pipeline**, or around
   **characterising the failure modes** (non-holonomic docking, perception gap)?"

**On logistics:**
6. "Is the RoboCup Smart Manufacturing / EAI-WS framing the right context, or do you prefer a
   more general on-demand-manufacturing framing?"
7. "How often would you like to check in, and in what form?"

---

## Part B — Questions HE may ask you (be ready)

**Q: Why Ackermann and not differential drive?**
> "The LIMO PRO supports both, but I built it as a car-like base because that's the harder,
> more realistic case for warehouse vehicles — it can't turn in place or strafe, which makes
> the final approach to the object a genuine research problem. Differential would be the easy
> fallback if needed."

**Q: Is a single pick-and-place enough for a Bachelor thesis?**
> "The full autonomous loop on real, constrained hardware, plus a quantified sim-to-real study,
> is already substantial. The depth I'm adding is in solving the five concrete challenges I
> found and measuring the gap — that's the contribution, not just a demo."

**Q: Can you actually do the stacking?**
> "Honestly, free-form stacking is high-risk — it needs sub-cm placement that the current
> sensing and arm reach can't guarantee. I'd keep it as a stretch goal, and if I get there I'd
> start with an open-loop two-block stack at a known location rather than perception-driven
> stacking."  *(This honesty builds trust.)*

**Q: What's the biggest risk?**
> "Real-hardware time and the perception sim-to-real gap. I'm de-risking by validating
> everything in simulation first and characterising the gap quantitatively."

**Q: What have you actually got working right now?**
> "Autonomous Ackermann navigation to the table, collision-aware arm planning, colour+depth
> perception, and the mission state machine — all integrated in Gazebo. I can show it." (→ `04`)

**Q: Why is the grasp not 100% yet?**
> "I traced it to a measurable perception bug — the depth camera over-reports the object
> distance at close range, so MoveIt aims past the object. That's a known sim camera near-clip
> limitation and exactly the kind of sim-to-real issue the thesis studies."

**Q: What's novel here?**
> "Not a new algorithm — an honest, integrated, low-cost system study: making Nav2 + MoveIt +
> perception cooperate on a non-holonomic, compute-constrained platform, and documenting what
> breaks when you go from sim to real."

---

## If he pushes for more ambition
Don't resist — *absorb and bound it*:
> "Happy to aim higher — could we agree the single-block cycle is the guaranteed deliverable,
> and the stacking is a documented stretch goal I pursue if the core is solid by week X?"
This keeps you safe while showing ambition.
