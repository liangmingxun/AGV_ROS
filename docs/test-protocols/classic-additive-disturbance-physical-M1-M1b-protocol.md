# Physical Candidate A M1 versus M1b protocol

Status: reserved; do not start until qualification freezes
`lambda_safe_common` and the user separately authorizes formal physical data
collection.

Use the identical path, R1, tracker, Stage-D calibration, sensing chain,
initial conditions and frozen Candidate A structure for both methods. Run at
least three pre-registered pairs in this order:

1. Pair01: M1b, then M1.
2. Pair02: M1, then M1b.
3. Pair03: M1b, then M1.

Every run needs fresh algorithm, disturbance and recorder state; valid camera
and three-chassis communication; recorded initial conditions and battery
voltage. Do not selectively repeat poor performance. Only infrastructure/data
failure or a safety abort invalidates a run.

Report robust margin, risk signal, dynamic boundary and common-reference
reduction; Robot2 controller raw demand, threshold/limiter duration and actual
wheel tracking; heading/lateral/support/rigid-fit/pairwise errors; and task
completion time. Report both M1 benefit and its time cost.
