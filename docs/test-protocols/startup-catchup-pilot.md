# Calibrated 0.10 m/s startup catch-up pilot

The 20260913_140437 raw run shows startup progress lag, not a universal
low-speed command/feedback scaling failure. Startup-average applied/actual
body wheel speeds for command reference bins 0..0.02, 0.02..0.04,
0.04..0.06 and 0.06..0.10 m/s are respectively:

- Robot1: .0074/.0076, .0274/.0290, .0478/.0481, .0936/.0918.
- Robot2: .0083/.0097, .0308/.0293, .0530/.0541, .0946/.0960.
- Robot3: .0087/.0086, .0321/.0286, .0549/.0533, .0960/.0946.

These are bin averages, not an identified motor dynamic model. Reference
progress nevertheless accumulates faster than measured progress during
acceleration. The old execution envelope disallowed positive catch-up above
the public ramp and attenuated feedback using the same slow ramp.

The normal and reference-v1 0.10 m/s runtime configurations now share a
candidate startup adapter: .008 m/s peak catch-up margin weighted by
4*scale*(1-scale), longitudinal feedback smoothly reaches full weight in
1.6 s, and the temporary channel envelope releases smoothly into existing
capability during the final quarter of the start.
After the 20260913_142040 run, the candidate ramp is extended from 3.2 to
4.0 s: peak public acceleration decreases from .058594 to .046875 m/s^2.
The successful catch-up/feedback settings are retained; steady velocity is
still .10 m/s. This additional change needs physical confirmation.
Heading feedback retains its original ramp. R1 equations, physical wheel
publication/plant limits, steady gains, timing gates and derating conditions
are unchanged. The .008 margin is not a permanent extra speed allowance;
late-start envelope release is separately bounded by existing capability.
Zero-valued new parameters preserve legacy behavior, including historical
tracking-v2/reconciliation-v1 candidates and frozen 0.08 m/s runtimes.

This is a software-verified pilot, not hardware-qualified improvement.
First repeat the no-derating M1 run, comparing raw startup progress maximum,
startup actual speed peak and wheel demand/applied/actual response. Then
repeat paired M1/M2a reference-v1 at .80 with identical startup settings.
Restore the two new parameters to zero to roll back this candidate.
Also restore startup_ramp_seconds to 3.2 to reproduce the prior duration.
Compile the formal algorithm targets on Robot1 before use; these central-only
changes do not require Robot2/3 deployment or chassis restart. Do not rerun historical
bags as if they used this new controller.

## Early-rise candidate after 20260913_150420

Robot1 startup applied/actual mean body wheel speeds in the 0..0.002,
0.002..0.005, 0.005..0.010, 0.010..0.020, 0.020..0.040 and
0.040..0.080 m/s applied-command bins are respectively .000657/.000122,
.003520/.004242, .007563/.008711, .014979/.014430, .029847/.029141,
.061843/.060207 (32/16/18/22/40/90 samples, first three seconds after
public reference exceeds .001). These observations do not identify a fixed
deadband or justify a motor-specific jump command.

Both current 0.10 runtime configurations now use `startup_early_rise: 0.6`:
quintic smoothstep is evaluated at `u=tau+k*tau*(1-tau)*(1-2*tau)`.
The warp is monotone for k in [0,1], preserving zero velocity-scale slope
and curvature at both endpoints. The analytic acceleration uses the chain
rule. At nominal .10 m/s and four seconds, the reference spends .376 s
below .002 m/s rather than .541 s; peak ramp acceleration is about
.03322 m/s^2 rather than .046875. These are reference-curve properties,
not predictions of actual wheel response or tracking improvement.

Linear feedback now reaches full weight at 1.2 rather than 1.6 s. Peak
catch-up margin remains .008 m/s; no extra permanent speed or per-wheel
friction boost is introduced. Heading feedback, steady gains, 120 ms
measured-motion projection and physical/safety limits are unchanged.
The shape and feedback are shared by paired methods, not Robot1-only.
Set early_rise to 0 and feedback seconds to 1.6 to reproduce the 150420
startup candidate. New defaults preserve legacy frozen configurations.

First repeat no-derating M1 under the same conditions. Compare first-five-
second progress RMS, first-ten-second speed overshoot, and subsequent
steady RMS against 150420; do not accept improvement only in filtered
plots or assume disappearance of timing failures. No historical run is
rewritten. Physical benefit is pending this pilot.

Payload processing v13 rejects degenerate rigid fits (zero-spread supports
or unidentifiable rotation), emitting NaN rather than a fabricated pose.
Load position/yaw and measured/equivalent consistency statistics use the
same evaluation-active, localization-valid, algorithm-valid scope as support
tracking. Valid large errors remain included; algorithm/localization
invalidity and physical failure validation are not relaxed. Raw bag,
parameter snapshots and config hashes remain unchanged during reprocessing.
