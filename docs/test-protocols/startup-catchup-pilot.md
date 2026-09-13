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
capability during the final quarter of the unchanged 3.2 s start.
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
Compile the formal algorithm targets on Robot1 before use; synchronize Git
versions across the fleet for deployment checks. Do not rerun historical
bags as if they used this new controller.
