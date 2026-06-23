# Scope & Deliverables — the implementable pieces

This is the core of the meeting: a **tiered, evidence-grounded scope** so you commit to something
finishable while keeping the ambitious vision visible. Every claim below traces to a repository
artifact (cited), not to memory. Let the professor set the line between "core" and "stretch."

> **Sources:** `docs/Codex Analysis/01–03`, `thesis_analysis/03_contribution.md`,
> `docs/PARAMETER_AUDIT.md`, and live verification on 2026-06-15.

---

## Honest current status (say this plainly)
- **Implemented:** ≈ **2,550 LOC** of original work across five subsystems
  (orchestration ~1000, visual-servo ~170, perception ~317, MoveIt config ~450, middleware ~323
  — `thesis_analysis/03_contribution.md`).
- **Verified working in simulation:** the custom LIMO–myCobot model; an audited URDF/sensor set;
  **autonomous Ackermann navigation** (Nav2 `SmacPlannerHybrid` + `RegulatedPurePursuit`) drove the
  base ≈ 3 m to the workstation on **2026-06-15**; MoveIt collision-aware planning; HSV+depth
  perception; a mission state machine.
- **Not yet done (the gaps — be upfront):** literature synthesis, a recorded **experiment
  campaign with metrics**, the **place** action + attached-object handling, and a clean
  reproducible build state. (`Codex Analysis/01–02`.)
- **The documented academic risk is *scope inflation*** — the strongest salvage is a
  **simulation-centred integration + evaluation thesis**, with real hardware as bounded future
  work (`Codex Analysis/02 §7–9`).

---

## The honest take on the stacking/assembly idea
Your original vision — pick several boxes and **stack them into an assembly** — is a great demo
story, but the stacking is the hardest, riskiest part:
- it needs **sub-centimetre placement accuracy**, while we have **measured** a close-range depth
  error of ~0.1 m (≈0.27 m perceived vs. 0.18 m true — `PARAMETER_AUDIT.md`) and a non-holonomic
  base that cannot fine-position;
- it needs placement perception, stack stability, and error recovery — each a mini-project.

**Recommendation:** keep stacking **out of the core**. Core = single-block
pick-transport-place; stacking = stretch / future work (optionally an *open-loop two-block stack
at a known location*, which avoids placement perception).

---

## Tiered scope

### Tier 0 — Foundation (already implemented) ✅
Custom LIMO–myCobot Gazebo model (`gazebo/ackermann_with_sensor.xacro`); spec-audited parameters;
Ackermann Nav2 (`nav2_limo.launch.py`, `config/nav2_limo_ackermann.yaml`); MoveIt config
(`limo_cobot_moveit_config`); perception (`box_pose_estimator.py`); orchestrator
(`nav_pick_orchestrator.py`).

### Tier 1 — CORE thesis (commit to this) 🎯
**Single-object cycle in simulation, plus evaluation:**
1. Autonomously **navigate** to a known workstation. *(working)*
2. **Detect** a coloured block and estimate its pose. *(working)*
3. **Pick** it (with a visual-docking final approach to fix the non-holonomic error). *(in progress)*
4. **Transport** to a workbench. *(nav — working)*
5. **Place** it flat at a marked target. *(to implement)*
6. **Evaluate**: success rate, navigation accuracy, grasp/placement reliability, cycle time +
   a **sim-to-real gap analysis** against verified specs.

This is a complete Bachelor thesis and answers all three research questions.

### Tier 2 — Realistic stretch ➕
Accurate placement at a fixture; sequential handling of 2–3 blocks (one at a time);
open-loop two-block stack at a known location.

### Tier 3 — Vision / Future work 🌟
Perception-driven stacking/assembly; multiple object types; closed-loop placement; **full
real-hardware deployment** of the complete loop.

---

## Why this is a strong thesis (lead with these)
- A **complete autonomous loop** on a low-cost, compute-constrained platform.
- A **harder kinematic case** (non-holonomic Ackermann base) than the differential-drive
  platforms common in the literature.
- A **quantified sim-to-real contribution** — most notably: the real Orbbec DaBai is **blind
  closer than 0.30 m**, but the grasp is at **0.21 m**, so the object is *invisible at grasp range
  on real hardware* (`PARAMETER_AUDIT.md §2`). This single finding reframes how such a system must
  be built and is exactly the kind of honest engineering result a thesis should report.

---

## Concrete "pieces I can implement next" (say you're ready to go)
- **Visual-docking** final approach (centre the base on the object before grasping).
- Resolve the **close-range depth bias** (or fuse perception with the known station geometry).
- The **place** action + MoveIt attached-object handling.
- The **evaluation harness** (N trials; log success/accuracy/time).
- The **literature review** (Week-1 deliverable) and reproducible build.
