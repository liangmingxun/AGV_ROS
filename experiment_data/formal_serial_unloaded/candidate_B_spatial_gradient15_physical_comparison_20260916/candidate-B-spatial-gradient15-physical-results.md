# Candidate B spatial gradient15 physical results

Profile: `candidate_B_v2_spatial_gradient15_rho0p85_av0p020_aw0p2625`

Payload semantics: unloaded equivalent payload (not measured payload).

Common spatial interval: `1.83018404..2.90755612 m`, coordinate `load_s_reference`, 5001 samples, trapezoidal spatial integration.

| Method | Support spatial RMSE / m | Rigid-fit spatial RMSE / m | Pairwise spatial RMSE / m | Load position RMS / m | Load yaw RMS / rad | Native wheel peak / (m/s) | Limiter / s | Margin mean | Task / s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1 | 0.018524401 | 0.0077152282 | 0.013812003 | 0.0093883346 | 0.036240638 | 0.16416362 | 0 | 0.9311281 | 54.830063 |
| M1b | 0.019155697 | 0.0086581658 | 0.017977586 | 0.0090627897 | 0.03167755 | 0.19799944 | 0 | 0.81912717 | 53.940103 |
| M2b | 0.018675214 | 0.0071154099 | 0.01273694 | 0.0093279686 | 0.038509959 | 0.19843269 | 0 | 0.82114376 | 53.949925 |

## Spatial-domain improvements

| Comparison | Support | Rigid-fit | Pairwise |
| --- | ---: | ---: | ---: |
| M1 vs M1b | 3.296% | 10.891% | 23.171% |
| M1 vs M2b | 0.808% | -8.430% | -8.441% |

All source values remain in the JSON/CSV outputs. No samples are removed for plotting.
