# Instructions for Claude Code

## Atlas — this repo on the map of all the repos
- This repo is **`quansheng-v3`** on the Atlas (<https://cdomotor-g.github.io/atlas/>). How to work with the map — `atlas.json`, the repositories that rely on this one, asks, pins, status and blockers — is in the shared instructions (`shared/on-the-atlas.md` in the Atlas).
- **This repo has GitHub issues turned off**, so its drift check reports only (`issues: false`) and nobody can file an ask here: other repositories record asks for it in their own `atlas.json`. Check the Atlas panel's *Asks* when you start.

<!-- shared:start · written by tools/shared.mjs in cdomotor-g/atlas; the rules themselves live in its shared/ folder -->
## Shared instructions — loaded from the Atlas
The rules every one of my repositories shares live in one file in the Atlas: how to end a run (the run summary), pushing to `main`, plain words, private data, who clears what, and issue titles — <https://github.com/cdomotor-g/atlas/blob/main/shared/instructions.md>. This repository is on the Atlas map, so <https://github.com/cdomotor-g/atlas/blob/main/shared/on-the-atlas.md> holds as well.
`.claude/settings.json` loads them when a session starts. If they are not in front of you, fetch them before you reply (`curl -fsS https://raw.githubusercontent.com/cdomotor-g/atlas/main/shared/instructions.md`, and the same for `on-the-atlas.md`) and follow them. Where this file says something different, this file wins.
<!-- shared:end -->
