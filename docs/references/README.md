# References — assets mined from the sibling project (`~/Desktop/Thesis`)

Everything valuable from the sibling workspace, copied here so the sibling can be deleted without
losing it. **Nothing here is wired into the live build** — it's reference material + data to
integrate deliberately. (Verdict: *Thesisorg is the base we finish on*; the sibling's value is its
documentation, real data, reusable tooling, and the geometric perception reference.)

## `docs/` — consolidated and archived
The original reference documents copied from the sibling project have been consolidated into the **`docs/engineering/`** folder:
* **[ARCHITECTURE.md](file:///home/omar/Desktop/Thesisorg/docs/engineering/ARCHITECTURE.md)** — Consolidated system overview, arm-mounting transforms, and data flow.
* **[PARAMETERS.md](file:///home/omar/Desktop/Thesisorg/docs/engineering/PARAMETERS.md)** — Consolidated parameter audits and verified robot specifications.
* **[TROUBLESHOOTING.md](file:///home/omar/Desktop/Thesisorg/docs/engineering/TROUBLESHOOTING.md)** — Consolidated 20-problem log and verification playbook.
* **[STATUS.md](file:///home/omar/Desktop/Thesisorg/docs/engineering/STATUS.md)** — Consolidated roadmap, active status board, and progress logs.

The raw original files (`ENGINEERING_DATA_DOSSIER.md`, `Integrating_LIMO-Cobot_Simulation_System.md`, `LIMO_PRO_REFERENCE.md`, `handover_report.md`, `VERIFICATION_GUIDE.md`, `roadmap.md`) have been moved to `docs/_archive/2026-06-19_reorg/`.

The large AI-generated chat conversations remain in this folder for deep-dive reference:
* **[Orchestrating LIMO Mobile Manipulation.md](file:///home/omar/Desktop/Thesisorg/docs/references/docs/Orchestrating%20LIMO%20Mobile%20Manipulation.md)**
* **[Fixing LIMO Autonomous Navigation Failures.md](file:///home/omar/Desktop/Thesisorg/docs/references/docs/Fixing%20LIMO%20Autonomous%20Navigation%20Failures.md)**

## `data/` — REAL measured data (use directly in Chapter 5)
- **`ik_workspace_map.csv`** — ⭐ 750 sampled poses, **194 reachable (25.9 %)**; envelope
  **x∈[0.10,0.24], y∈[−0.05,0.10], z∈[0.02,0.18]**; reachability collapses past x=0.18 m; top-down
  (pitch π/2) is the hardest at 23.2 %. **This is Step 3 (reach map) + Step 8 (data) already done.**
- `Test_report_*.csv`, `processed_full/` — additional run/test data.

## `code_from_sibling/` — reusable code (port deliberately, don't bulk-build)
- **`limo_cobot_tasks/`** — the reach/IK tooling: `search_reachable.py` + `test_ik_reaches.py`
  (generate the reach map), `moveit_client.py` (clean MoveIt wrapper with the **22-seed IK** +
  the **synchronous-call fix** for the rclpy deadlock), `base_controller.py`. *Port `moveit_client`
  first — it's the reusable engine.* ⚠️ has hard-coded `/home/omar/Desktop/Thesis/...` paths to fix.
- **`limo_cobot_bridge/`** — `bridge_node.py` (the config-driven 8-state FSM orchestrator) + its
  `config/*.yaml` (zones, timeouts, tolerances, perception/MTC params). *Adopt the **pattern** for
  our `pick_orchestrator`.* Note: `_state_place()` is a stub; the in-place-rotation drive is
  non-Ackermann.
- **`perception_pcl/`** — the color-free geometric perception (`plane_segmentation`,
  `cluster_extraction`, `object_segmentation`, `get_planning_scene_server/client`, `mtc_node`).
  ▼ This is the **"detect any object regardless of colour, on table or floor"** capability.
  ⚠️ ~3 000 lines, fragile (the part that stalled the sibling) — use as a **reference / selective
  port**, not a wholesale import.

## How this feeds the plan
* **Reach envelope:** mostly *already done* — `data/ik_workspace_map.csv` is the result; re-run `search_reachable.py` only if we change the arm.
* **Thesis writing:** the consolidated engineering data and problem logs are the raw material — fold them into Chapters 3/4/5 (flag the Chapter 5 tables as *provisional, best-case sim* exactly as the status dossier does honestly).
* **Perception upgrade (unknown objects):** `perception_pcl/` is the reference to port the color-free path.

