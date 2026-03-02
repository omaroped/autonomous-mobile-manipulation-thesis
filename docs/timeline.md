# Thesis Timeline — 14-Week Plan

> **Start date:** March 2026
> **Target completion:** June 2026
> **Thesis topic (working):** Mobile Manipulation for On-Demand Manufacturing using the LIMO Cobot

---

## Phase 1 — Foundation (Weeks 1–3)

**Goal:** Solid understanding of the domain, working dev environment, basic simulation running.

| Week | Tasks | Deliverable |
|------|-------|-------------|
| 1 | Literature review kickoff (EAI-WS paper, Nav2, MoveIt2, LIMO docs). Finalize thesis proposal with professor. | Annotated reading list (10+ papers). Approved proposal. |
| 2 | Set up Ubuntu on Orin Nano. Install ROS 2. Verify LIMO base teleop. Set up Gazebo sim. | Working teleop demo. Gazebo world with LIMO spawned. |
| 3 | SLAM & mapping experiments (Cartographer or SLAM Toolbox). Basic Nav2 integration. | First autonomous navigation run (sim + real if possible). |

**Milestone:** `v0.1-foundation` — Robot drives autonomously in a mapped environment.

---

## Phase 2 — Core Development (Weeks 4–8)

**Goal:** Navigation pipeline solid. Manipulation pipeline functional. Perception working for target objects.

| Week | Tasks | Deliverable |
|------|-------|-------------|
| 4 | Nav2 tuning (DWB, recovery behaviors). Waypoint following. | Reliable point-to-point navigation in warehouse layout. |
| 5 | MoveIt2 setup for Mycobot 280. Basic pick-and-place in sim. | Arm moves to commanded poses. Picks a block in simulation. |
| 6 | Perception pipeline — object detection for colored blocks (YOLO / custom). Depth-based pose estimation. | Detects and localizes blocks from depth camera feed. |
| 7 | Integration: navigate to shelf → detect block → pick → navigate to workbench → place. | End-to-end pick-and-transport demo (sim). |
| 8 | Transfer to real hardware. Debug sensor-to-actuator pipeline. Real-world calibration. | First real-world pick-and-transport run. |

**Milestone:** `v0.2-core` — Robot navigates to a shelf, picks a block, delivers it to a workbench.

---

## Phase 3 — Integration & Experiments (Weeks 9–11)

**Goal:** Systematic experiments. Data collection. Performance benchmarking.

| Week | Tasks | Deliverable |
|------|-------|-------------|
| 9 | Define experiment protocol. Run navigation accuracy experiments (N runs, measure success rate, time, drift). | Navigation experiment log with quantitative results. |
| 10 | Run manipulation experiments (grasp success rate, placement accuracy). Assembly task if feasible. | Manipulation experiment log. |
| 11 | Full pipeline stress test. Multi-order scenarios. Edge cases. Failure analysis. | Integration experiment log. Performance summary table. |

**Milestone:** `v0.3-experiments` — Complete dataset of experiment results.

---

## Phase 4 — Writing & Defense (Weeks 12–14)

**Goal:** Thesis document complete and defended.

| Week | Tasks | Deliverable |
|------|-------|-------------|
| 12 | Thesis chapters 1–4 (Introduction, Background, Methodology, System Design). | First draft of core chapters. |
| 13 | Thesis chapters 5–7 (Experiments, Results, Conclusion). Figures and tables finalized. | Complete first draft. |
| 14 | Revisions from professor. Defense slides. Final submission. | Final thesis PDF. Defense presentation. |

**Milestone:** `thesis-final` — Thesis submitted and defended.

---

## Buffer

This is a 14-week plan inside a ~16-week window. The remaining 2 weeks serve as buffer for:
- Hardware delays or failures
- Unexpected debugging
- Professor feedback cycles
- Personal schedule conflicts

Do not schedule the buffer. Let it absorb the inevitable surprises.
