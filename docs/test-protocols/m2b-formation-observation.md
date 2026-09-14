# M2b unloaded formation observation

Use the shared circle interface with `--method M2b --derating-0p80
--reference-v1 --formation-observation --confirm-support-range-0p80-clear`
and all existing physical confirmations. This adds a separate runtime and
authorization. Use `--no-derating` in place of `--derating-0p80` for
the paired no-derating run with identical observation/startup settings.
M2b keeps its complete
upper/lower controller, not shared R1 reference-acceleration wiring.

The estimator observation semantics are the same as M2a: geometric error
alone does not abort; any support-centre pair above 0.80 m latches the range
protection. Initial formation anchoring, valid/timely camera and motion,
finite/converged computation, chassis limiter and task envelope remain.
No mechanical connections or payload are permitted. Clear the expanded
footprint including bodies and keep emergency stop available. Maximum spread
is not collision avoidance. Report geometric errors and completion status
separately; endpoint completion is not successful formation tracking.

Serial M2b startup v2 resets the generator velocity and auxiliary state to
rest after stationary qualification, ramps the leader velocity/acceleration
over the configured four seconds, and integrates leader progress only while
valid. Its acceleration output is integrated on persistent execution command
state, with a temporary startup envelope and gradual planar feedback entry.
Safety zeros reset execution state and re-arm the same ramp. Capability is
still log-only in the M2b equations; Robot2 derating remains a chassis limit,
not an injected reduced reference. Fake/golden method behavior is unchanged.
This is a physical-interface pilot, not evidence of completed qualification.
