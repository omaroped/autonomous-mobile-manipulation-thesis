# Working Rules

Ground rules for keeping this thesis project organized, traceable, and professional.
These apply from day one and should be followed consistently throughout the entire thesis period.

---

## 1. Git Discipline

### Commit Messages

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short description>

[optional body]
```

**Types:**
- `feat` — new capability (code, hardware integration, perception pipeline)
- `exp` — experiment-related changes (configs, scripts, logs)
- `docs` — documentation, thesis writing, meeting notes
- `fix` — bug fixes
- `refactor` — code restructuring without behavior change
- `chore` — maintenance (dependencies, CI, tooling)

**Examples:**
- `feat(nav): implement Nav2 waypoint follower for warehouse track`
- `exp(grasp): add pick-and-place trial config for 2x2 blocks`
- `docs(thesis): draft chapter 3 — system architecture`
- `fix(perception): correct HSV thresholds for blue block detection`

### Branches

- `main` — stable, working state. Every merge here should compile and run.
- `dev` — active development. Merge to main when a milestone is reached.
- `feature/<name>` — new capabilities (e.g., `feature/slam-integration`)
- `experiment/<name>` — experiment branches (e.g., `experiment/grasp-accuracy-v2`)
- `docs/<name>` — writing and documentation work

### Tags

Tag milestones: `v0.1-foundation`, `v0.2-navigation`, `v0.3-manipulation`, etc.
Tag thesis drafts: `thesis-draft-1`, `thesis-draft-2`, `thesis-final`.

---

## 2. Experiments

Every experiment gets its own log file in `experiments/` using the template.

**Naming convention:** `YYYY-MM-DD_short-description.md`
- Example: `2026-03-15_nav2-waypoint-accuracy.md`

**Rules:**
- Always record the Git commit hash at the time of the experiment.
- Always note the hardware configuration (which sensors were active, arm mounted or not, etc.)
- Failed experiments are documented just as thoroughly as successful ones. Negative results matter.
- Raw data goes into `data/raw/<experiment-name>/`. Never modify raw data.
- Processed data goes into `data/processed/<experiment-name>/`.

---

## 3. Meeting Notes

After every meeting with the professor, write up notes in `docs/meetings/` using the template.

**Naming convention:** `YYYY-MM-DD_meeting.md`

**Rules:**
- Write notes within 24 hours of the meeting while things are fresh.
- Always capture action items as checkboxes.
- Note any decisions that affect the thesis scope or direction.

---

## 4. Thesis Writing

- Write in LaTeX in `docs/thesis/`.
- One file per chapter in `docs/thesis/chapters/`.
- Figures go into `docs/thesis/figures/` with descriptive filenames, not `fig1.png`.
- Bibliography entries go into `bibliography.bib` immediately when a paper is read, not later.
- Commit writing progress at least once per working session.

---

## 5. References

- Every paper that gets read gets an entry in `references/reading_list.md`.
- PDFs are stored locally in `references/papers/` but git-ignored (copyright).
- Datasheets go in `references/datasheets/`.

---

## 6. Changelog

Update `CHANGELOG.md` at least once per week, or whenever something significant happens.
This is a human narrative, not a Git log. It should be readable by your professor or anyone reviewing your progress.

---

## 7. Code Quality

- Python: follow PEP 8, use type hints where reasonable.
- C++: follow ROS 2 coding style guidelines.
- Every ROS 2 package must have a functioning `CMakeLists.txt` or `setup.py`.
- No hardcoded paths. Use parameters and launch files.
- Comment non-obvious logic. Don't comment the obvious.

---

## 8. General Principles

- **Document as you go.** Writing things down later never works as well.
- **Commit often.** Small, focused commits are better than large dumps.
- **Never delete data.** Archive it, move it, but don't delete it.
- **Ask when stuck.** Spending more than 2 hours on one problem without progress means it's time to ask for help or change approach.
- **Keep the repo clean.** If something doesn't belong here, it goes somewhere else.
