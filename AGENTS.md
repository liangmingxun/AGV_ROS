# Repository working rules

## Scope first

- Inspect only the files, tests and recorded runs needed for the requested
  change. Do not reread the whole repository or rerun the full test suite by
  default.
- Preserve unrelated user changes. Do not edit historical packages unless the
  requested change reaches them.

## Proportional verification

Use the smallest verification set that can catch a regression introduced by
the current diff:

- Documentation only: check the edited text, paths and commands; no build.
- Python analysis only: `py_compile` the edited modules and run the directly
  affected Python test file once.
- YAML, launch or shell only: syntax-check the edited file and run the directly
  affected static test. If runtime behavior or a safety gate changed, add its
  one corresponding integration test. Do not rebuild unchanged C++ packages.
- C++ implementation: build the affected package once and run its directly
  affected gtest/rostest target.
- Message, package dependency, CMake, shared controller-core or multi-package
  interface changes: build and test all affected packages.

Run a full workspace or package regression only for a release/merge check, a
shared interface change, a broad refactor, or when the user explicitly asks for
it. Do not repeat an already passing command unless relevant files changed.

ROS integration tests that share a ROS master must run serially. If a parallel
run causes interference, rerun only the failed relevant target serially; do not
rerun every passing target.

For conversion, validation or metrics changes, use at most one known-good bag
and one known-failing bag unless wider historical compatibility is part of the
request. Never rewrite source evidence merely to test a converter; use a
temporary output directory.

## Physical experiments

- Never start a physical vehicle unless the user explicitly requests it.
- Per-run safety and provenance gates are not optional: area clear, reachable
  emergency stop, correct fixture/load state, single command authority, serial
  chassis identity, deployed-code compatibility, physical wheel limit, clock,
  localization, recorder readiness and post-run validity.
- Build/fake regression, Stage A-D qualification, wheel calibration, camera
  calibration, single-car commissioning and speed-ladder trials are not
  repeated before every pilot. Repeat them only after a relevant software,
  hardware, calibration or environment change, or after an anomaly implicates
  that subsystem.
- Do not run unrelated methods or conditions to prepare an M1+R1 pilot.

## Reporting

Report the smallest meaningful verification performed, its result, and any
known unrelated failure. Do not present raw test counts as a requirement for
future turns. Keep successful command output summarized; inspect or quote full
logs only for a failure being diagnosed.
