# Repository working instructions

- Read `docs/HANDOFF.md` first, then the relevant sections of `docs/BUILD_PLAN.md`.
- Work from this checkout. Files in the parent workspace are historical inputs,
  not the shared source of truth.
- Keep the handoff current at every completed milestone. Record what is working,
  what is provisional, the tests run, and the next actionable task.
- Use focused commits and push completed milestones as requested by the user.
  Use feature branches for subsequent work and avoid force pushes. Inspect remote
  changes before integration; preserve teammates' work.
- Run relevant tests before committing. For simulation changes use:
  `python -m unittest discover -s sim -p 'test_*.py'` and
  `python sim/simulate_assembly.py --check` in the configured environment.
- Do not claim idealized attachment is contact simulation, or scripted motion is
  learned control. Keep simulation, hardware, and ML evaluation results distinct.
- Keep units/frames explicit and preserve the supplied URDF axes. Do not regenerate
  the model or silently change calibration to make a test pass.
- Never commit secrets, virtual environments, generated recordings, or large
  training artifacts. Follow `CONTRIBUTING.md`.
