# Contributing

Thank you for improving this tutorial workspace.

## Principles

- Keep changes focused on Kinova Gen3 + Geomagic Touch integration, ROS Noetic, or Docker ergonomics.
- Match existing launch/script style (ROS naming, Python 3 shebangs).
- If you add hardware-specific assumptions (topics, frames), document them in the README.

## Workflow

1. Fork or branch from `main` / `master`.
2. Build the workspace locally or via Docker (`docker compose build`).
3. Test at least one launch file relevant to your change (`demo_rviz.launch` or `demo_hardware.launch`).
4. Open a pull request describing intent, testing performed, and any new dependencies.

## Submodule `ros_kortex`

Upstream fixes belong in [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex). This repo pins a released tag for reproducibility; bump the submodule deliberately (with release notes in the PR).

## Licensing

By contributing, you agree your contributions are under the same license as the repository (see `LICENSE`), except where a file already carries a different header (e.g. third-party `geomagic_control`).
