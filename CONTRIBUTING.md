# Working together

1. Read `docs/HANDOFF.md` and `docs/BUILD_PLAN.md` before starting.
2. Fetch the latest remote state. Use a short feature branch for new work, and
   record the task, owner, and branch in the handoff when claiming shared work.
3. Keep commits small and focused: one coherent change with its relevant tests
   and documentation. Commit and push at working milestones and before handoff.
4. Run the checks relevant to the change. Include the commands and results in
   your PR or handoff. Label anything untested, simulated, or provisional.
5. Update `docs/HANDOFF.md`: completed work, next concrete step, limitations,
   owner/branch, and exact reproduction commands. Keep the build plan current
   when architecture or scope changes. Do not put the current commit's hash
   inside itself; use descriptive milestone names and Git history.
6. Push your branch and use a PR for review. Avoid concurrent edits to the same
   files. Never force-push shared history or overwrite a teammate's changes.

Do not commit virtual environments, credentials, local screenshots, recordings,
training datasets, or checkpoints. Store large experiment artifacts separately
and document how to obtain them. Commit small reproducible configs and scripts.

Coordinates and commands need explicit frames and units. Public pose contracts
use `root`, metres, and XYZW quaternions. Discrete build coordinates are grid
indices, not robot-frame positions. Hardware work must define calibration and
feedback requirements; numerical simulation success is not hardware validation.
