# Request policy optimization

The hybrid request policy uses:

```text
score = wait_weight * wait_time - distance_weight * distance
```

Higher score means higher priority.

## Recommended protocol

1. Fix the robot navigation enough that most runs finish the requested number of services.
2. Run the same seeds for every policy/weight combination.
3. Compare policies with the fair objective:

```text
fair_wait_objective = wait_time_mean + 0.25 * wait_time_max
```

When there are unfinished requests at the end of a simulation, the ranking uses
`effective_wait`, which combines completed `wait_time` values with unfinished
`pending_wait` values. This makes starvation visible instead of only measuring
requests that were eventually served.

4. Use `wait_time_mean` as a secondary view when fair scores are close.
5. Increase `-MaxWaitWeight` when the experiment should penalize high individual
   waits more strongly.

## Baseline comparison

```powershell
powershell -ExecutionPolicy Bypass -File trabalho/trabalho/scripts/run_policy_comparison.ps1 `
  -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -Runs 5 `
  -MaxCompletedRequests 10
```

This compares `FIFO`, `NEAREST`, and one `HYBRID` configuration.

## Hybrid weight sweep

```powershell
powershell -ExecutionPolicy Bypass -File trabalho/trabalho/scripts/run_hybrid_weight_sweep.ps1 `
  -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -Runs 5 `
  -MaxCompletedRequests 10 `
  -MaxWaitWeight 0.25 `
  -IncludeBaselines
```

Default hybrid value used by the controller:

```text
wait_weight = 1.0
distance_weight = 30.0
```

Default sweep:

```text
wait_weight = 1.0
distance_weight = 0, 5, 10, 20, 30, 40
```

You can provide other values:

```powershell
powershell -ExecutionPolicy Bypass -File trabalho/trabalho/scripts/run_hybrid_weight_sweep.ps1 `
  -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -WaitWeights 1.0 `
  -DistanceWeights 0,5,10,15,20,25,30 `
  -Runs 10 `
  -MaxCompletedRequests 15 `
  -IncludeBaselines
```

## Output files

Each run creates a directory under:

```text
trabalho/trabalho/scripts/results/
```

Important files:

```text
run_config.csv       configuration for every run: seed, weights, policy, log
metrics_raw.csv      every individual metric, annotated with the run config
metrics_summary.csv  grouped totals/means/min/max values
metrics_ranked.csv   experiments ranked by mean wait time
```

Use `metrics_ranked.csv` as the first view. It includes both
`rank_by_fair_wait` and `rank_by_mean_wait`, so you can see whether a policy is
good on average but unfair in the worst case.
