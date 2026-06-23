# Diagrams

Rendered figures for the teaching guide. Source is Graphviz `.dot` (text → easy to refine);
`.svg` (crisp, for screens/web) and `.png` (for slides/Word) are generated from it.

| Figure | Shows |
|--------|-------|
| `pipeline.svg` | The universal **Sense → Locate → Plan → Act** loop, mapped to our nodes. Start here. |
| `system_architecture.svg` | The full **node/topic data-flow** — who publishes what, who listens. The single most important picture. |
| `tf_tree.svg` | The **TF coordinate-frame tree** — how perception's camera point becomes the arm's target. |
| `grasp_state_machine.svg` | The **grasp sequence** — each step a MoveIt goal or a gripper/weld command, colour-coded. |

## Re-render after editing a `.dot`
```bash
cd docs/teaching/diagrams
for f in system_architecture tf_tree grasp_state_machine pipeline; do
  dot -Tsvg "$f.dot" -o "$f.svg"
  dot -Tpng -Gdpi=150 "$f.dot" -o "$f.png"
done
```
(Graphviz `dot` is already installed: `dot -V`.) For the thesis you can `\includegraphics` the
`.png`/`.svg` directly.
