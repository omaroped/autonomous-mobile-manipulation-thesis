# Pass 1 — Technical Timeline Reconstruction

## Chronological Table
| Date | Change | Subsystem Affected | Source |
| :--- | :--- | :--- | :--- |
| **2026-03-02** | Initial thesis workspace setup. Foundation phase. | Project Mgmt | Git (1abf8e5), CHANGELOG |
| **2026-03-07** | Simulation environment built (Gazebo world, LIMO spawn). | Simulation | CHANGELOG |
| **2026-03-08** | Writing infrastructure (LaTeX) and public repo initialized. | Documentation | CHANGELOG |
| **2026-03-15** | Educational course website designed (Vite/React). | Educational (Pivot) | CHANGELOG |
| **2026-03-19** | myCobot 280 integration. `ros2_control` segfaults encountered. | Arm Manipulation | CHANGELOG |
| **2026-04-02** | Applied `<dynamics>` damping/friction patches to LIMO base. | Base Dynamics | File mtime (`patch_dynamics.py`) |
| **2026-04-03** | Stripped mimic joints/German chars from URDF to fix Gazebo bugs. | Modelling | File mtime (`strip_mimic.py`) |
| **2026-04-05** | Critical fixes (world duplicates, LaTeX). Scaffolding audit. | Integration | CHANGELOG |
| **2026-06-08** | Final integration phase starts. Major `src/ros2_ws` build. | Integration | Build logs |
| **2026-06-10** | MoveIt 2 verifying (Execution SUCCEEDED). | Manipulation | PROGRESS_SUMMARY / Logs |
| **2026-06-12** | Navigation verified (2.9m → 0.22m). End-to-end trials. | Navigation | PROGRESS_SUMMARY / Logs |
| **2026-06-13** | Perception/TF debugging (current focus). | Perception | Build logs / Latest work |

## Strategic Pivots & Spine Work
- **The Simplification Pivot (Late March/April):** Abandoned MoveIt Task Constructor (MTC) and PCL in favor of a "deliberately lightweight" HSV + depth + Action Interface pipeline. This is cited in `PROJECT_NOTES.md` as the reason the project finally reached a working state.
- **The "Spine" (April - June):** Sustained effort on the MoveIt 2 Action Interface integration and `ros2_control` patching. This represents the core "simulation-based integration study".
- **Dead-end (Mar 15):** The Educational Course Website was built but ultimately excluded from thesis deliverables to focus on the technical pipeline.

## Forensic Markers
- **Build Logs:** Intense activity between June 9 and June 13 suggests a "crunch" phase to finalize the end-to-end pipeline before the thesis deadline (mid-June).
- **Sub-repository Mods:** Extensive uncommitted changes in `limo_ros2` and `mycobot_ros2` indicate that "original substantive code" was often written directly into cloned vendor packages.
