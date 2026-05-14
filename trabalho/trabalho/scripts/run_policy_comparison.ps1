[CmdletBinding()]
param(
    [string]$Webots = "webots",
    [string[]]$Policies = @("FIFO", "NEAREST", "HYBRID"),
    [int]$Runs = 5,
    [int]$SeedBase = 1000,
    [int]$MaxCompletedRequests = 10,
    [double]$MaxSimTime = 240.0,
    [double]$StopDelay = 30.0,
    [double]$FirstPeriod = 5.0,
    [double]$MinPeriod = 8.0,
    [double]$MaxPeriod = 16.0,
    [double]$HybridWaitWeight = 1.0,
    [double]$HybridDistanceWeight = 20.0,
    [switch]$Render
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectDir = Resolve-Path (Join-Path $scriptDir "..")
$worldPath = Resolve-Path (Join-Path $projectDir "worlds\restaurant_final.wbt")
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$resultsRoot = Join-Path $scriptDir "results"
$resultsDir = Join-Path $resultsRoot "policy_comparison_$timestamp"
New-Item -ItemType Directory -Force -Path $resultsDir | Out-Null

$configPath = Join-Path $resultsDir "run_config.csv"
"policy,run,seed,max_completed,max_sim_time,stop_delay,first_period,min_period,max_period,hybrid_wait_weight,hybrid_distance_weight,log" |
    Set-Content -Path $configPath -Encoding UTF8

$envNames = @(
    "SIM_RUN_ID",
    "REQUEST_POLICY",
    "RM_RANDOM_SEED",
    "RM_MAX_COMPLETED_REQUESTS",
    "RM_MAX_SIM_TIME_S",
    "RM_STOP_DELAY_S",
    "RM_FIRST_PERIOD_S",
    "RM_MIN_PERIOD_S",
    "RM_MAX_PERIOD_S",
    "HYBRID_WAIT_WEIGHT",
    "HYBRID_DISTANCE_WEIGHT"
)

$previousEnv = @{}
foreach ($name in $envNames) {
    $previousEnv[$name] = [Environment]::GetEnvironmentVariable($name, "Process")
}

try {
    foreach ($policy in $Policies) {
        $normalizedPolicy = $policy.ToUpperInvariant()

        for ($run = 1; $run -le $Runs; $run++) {
            $seed = $SeedBase + $run
            $runId = "$normalizedPolicy-$run"
            $logName = "$runId.log"
            $logPath = Join-Path $resultsDir $logName

            $env:SIM_RUN_ID = $runId
            $env:REQUEST_POLICY = $normalizedPolicy
            $env:RM_RANDOM_SEED = [string]$seed
            $env:RM_MAX_COMPLETED_REQUESTS = [string]$MaxCompletedRequests
            $env:RM_MAX_SIM_TIME_S = [string]$MaxSimTime
            $env:RM_STOP_DELAY_S = [string]$StopDelay
            $env:RM_FIRST_PERIOD_S = [string]$FirstPeriod
            $env:RM_MIN_PERIOD_S = [string]$MinPeriod
            $env:RM_MAX_PERIOD_S = [string]$MaxPeriod
            $env:HYBRID_WAIT_WEIGHT = [string]$HybridWaitWeight
            $env:HYBRID_DISTANCE_WEIGHT = [string]$HybridDistanceWeight

            "$normalizedPolicy,$run,$seed,$MaxCompletedRequests,$MaxSimTime,$StopDelay,$FirstPeriod,$MinPeriod,$MaxPeriod,$HybridWaitWeight,$HybridDistanceWeight,$logName" |
                Add-Content -Path $configPath -Encoding UTF8

            Write-Host "Running $runId with seed $seed..."

            $webotsArgs = @(
                "--batch",
                "--mode=fast",
                "--stdout",
                "--stderr",
                "--minimize"
            )

            if (-not $Render) {
                $webotsArgs += "--no-rendering"
            }

            $webotsArgs += $worldPath.Path

            & $Webots @webotsArgs *> $logPath

            if ($LASTEXITCODE -ne 0) {
                Write-Warning "Webots exited with code $LASTEXITCODE for $runId. Check $logPath"
            }
        }
    }
}
finally {
    foreach ($name in $envNames) {
        [Environment]::SetEnvironmentVariable($name, $previousEnv[$name], "Process")
    }
}

$parserPath = Join-Path $scriptDir "parse_sim_metrics.py"
python $parserPath --results-dir $resultsDir

Write-Host ""
Write-Host "Results directory:"
Write-Host $resultsDir
Write-Host "Open metrics_summary.csv to compare policies."
