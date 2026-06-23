# Pass 6 — Thesis Chapter Mapping

This map aligns existing repo artifacts with a standard engineering thesis structure.

| Thesis Chapter | Relevant Repo Artifacts | Evidence/Data Source |
| :--- | :--- | :--- |
| **1. Introduction** | `README.md`, `THESIS_EXPOSE.md` | Problem statement: mobile pick-and-place in smart mfg. |
| **2. Related Work** | `references/papers/` | Masannek et al. (EAI-WS 2026). |
| **3. System Design** | `limo_car/urdf/`, `mycobot_description/urdf/` | Xacro hierarchy, `ros2_control` hardware abstraction. |
| **4. Implementation** | `02_architecture.md`, `pick_orchestrator.py` | Perception, Navigation, and MoveIt 2 integration. |
| **5. Middleware Fixes** | `gazebo_ros2_control_plugin.cpp` (patch) | The "Humble Incompatibility" war story. |
| **6. Evaluation** | `docs/references/data/reach_map.csv`, `PROGRESS_SUMMARY.md` | Reachability heatmaps, nav precision benchmarks. |
| **7. Discussion** | `04_deadends.md`, `PROJECT_NOTES.md` | Analysis of the "Simplification Pivot" and reachability constraints. |
| **8. Conclusion** | `CHANGELOG.md` (TODO markers), `NEXT_10_STEPS.md` | Summary of working pipeline vs. hardware future-work. |

## Strategy for "Thin" Chapters
*   **Evaluation Chapter:** This is usually the thinnest part of a student project.
    *   *Fill with:* IK success heatmaps generated from `docs/references/data/reach_map.csv`.
    *   *Fill with:* A sequence of screenshots showing the planning scene being updated (box added/removed) in RViz.
    *   *Fill with:* Plots of visual error vs. time during the `box_follower` approach.
*   **Design Chapter:** Use the `mermaid` node graph from `02_architecture.md`. Export the final URDF as a 3-D visual or a simplified tree diagram using `urdf_to_graphiz`.

## Artifact-to-Evidence Mapping
*   **The "Contribution" Artifact:** The `limo_cobot_moveit_config` package. It proves the student didn't just use a demo; they built the semantic model for a custom-combined robot.
*   **The "Engineering Depth" Artifact:** The source-code patch in `gazebo_ros2_control`. It demonstrates the ability to diagnose and fix EOL toolchain bugs at the middleware level.
