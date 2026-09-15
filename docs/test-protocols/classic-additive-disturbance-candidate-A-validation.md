# Classic additive disturbance Candidate A validation

Scope: `CLASSIC_ADDITIVE_DISTURBANCE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE`.

This is a fake-only, fixed-work-point validation. It does not modify or replace
the archived Exp2c v4/v5/v5b evidence. Hardware and serial authorization remain
false. Candidate B and parameter search are outside this protocol.

Robot2 receives the fixed Candidate A disturbance after its native controller
wheel demand has been saved and before the common physical limiter and fake
plant. Robot1 and Robot3 are unchanged. CapabilityReport, the 0.160 m/s physical
wheel limit, M1/M1b parameters, the R1 tracker, Stage-D calibration, path and
support geometry are unchanged. Complete M2b continues to call the independent
`M2bController::step()` with the frozen `exp2b_M2b.yaml`.

The disturbance is triggered once when Robot2 measured progress first reaches
2.0 m. Wall-clock elapsed time then drives the 8.0 s profile: 0.5 s quintic
ramp-in, fixed sinusoidal amplitudes 0.030 m/s and 0.350 rad/s at 1 rad/s with
angular phase pi/2, and 0.5 s quintic ramp-out. Wheel separation is exactly
0.139284482 m.

Analysis windows relative to the trigger are fixed: baseline [-3,0),
disturbance [0,8), recovery [8,23), and primary [0,23). An incomplete recovery
is explicitly censored. Stage 0 is one M1 smoke run and is excluded from
statistics. Stage 1 consists of exactly three fresh triads in frozen rotated
order: M1b/M1/M2b, M2b/M1b/M1, and M1/M2b/M1b.

Run with:

```bash
roscd multi_agv_bringup/../..
bash src/multi_agv_bringup/scripts/run_classic_additive_candidate_A_validation_fake.sh
```

Every method run owns a fresh private ROS master, algorithm/controller, fake
plant, estimator, tracker, disturbance state, and recorder. Invalid engineering
runs may be rerun; unfavorable performance is retained. No automatic commit,
push, serial execution, Candidate B, or parameter tuning is allowed.
