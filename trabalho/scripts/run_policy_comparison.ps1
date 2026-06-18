[CmdletBinding()]
param(
    [string]$Webots = "webots",
    [string]$MapId = "map1",
    [string]$ExperimentMode = "EXP1",
    [string]$DynamicEnvironment = "false",
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
    [double]$HybridDistanceWeight = 30.0,
    [double]$MaxWaitWeight = 0.25,
    [switch]$Render
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Convert-StringList {
    param([string[]]$Values)

    $items = @()

    foreach ($value in $Values) {
        $parts = ([string]$value).Split(",", [System.StringSplitOptions]::RemoveEmptyEntries)

        foreach ($part in $parts) {
            $cleanPart = $part.Trim()
            if ($cleanPart.Length -gt 0) {
                $items += $cleanPart
            }
        }
    }

    return $items
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectDir = Resolve-Path (Join-Path $scriptDir "..")
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$resultsRoot = Join-Path $scriptDir "results"
$resultsDir = Join-Path $resultsRoot "policy_comparison_$timestamp"
$metricsDir = Join-Path $resultsDir "metrics_results"
New-Item -ItemType Directory -Force -Path $resultsDir | Out-Null
New-Item -ItemType Directory -Force -Path $metricsDir | Out-Null

function Resolve-WorldPathForMap {
    param([string]$SelectedMapId)

    switch ($SelectedMapId.ToLowerInvariant()) {
        "map1" { return (Resolve-Path (Join-Path $projectDir "worlds\restaurant_final.wbt")).Path }
        "map2" { return (Resolve-Path (Join-Path $projectDir "worlds\restaurant_mapa2.wbt")).Path }
        default { throw "Unsupported map id: $SelectedMapId" }
    }
}

$worldPath = Resolve-WorldPathForMap -SelectedMapId $MapId

$configPath = Join-Path $resultsDir "run_config.csv"
"experiment_label,map_id,experiment_mode,dynamic_environment,policy,repeat,run,seed,max_completed,max_sim_time,stop_delay,first_period,min_period,max_period,hybrid_wait_weight,hybrid_distance_weight,world,log" |
    Set-Content -Path $configPath -Encoding UTF8

$envNames = @(
    "SIM_RUN_ID",
    "MAP_ID",
    "RESTAURANT_MAP",
    "EXPERIMENT_MODE",
    "DYNAMIC_ENVIRONMENT",
    "REQUEST_POLICY",
    "RM_RANDOM_SEED",
    "RM_MAX_COMPLETED_REQUESTS",
    "RM_MAX_SIM_TIME_S",
    "RM_STOP_DELAY_S",
    "RM_FIRST_PERIOD_S",
    "RM_MIN_PERIOD_S",
    "RM_MAX_PERIOD_S",
    "HYBRID_WAIT_WEIGHT",
    "HYBRID_DISTANCE_WEIGHT",
    "METRICS_DIR"
)

$previousEnv = @{}
foreach ($name in $envNames) {
    $previousEnv[$name] = [Environment]::GetEnvironmentVariable($name, "Process")
}

try {
    foreach ($policy in (Convert-StringList $Policies)) {
        $normalizedPolicy = $policy.ToUpperInvariant()

        for ($run = 1; $run -le $Runs; $run++) {
            $seed = $SeedBase + $run
            $runId = "$normalizedPolicy-$run"
            $logName = "$runId.log"
            $logPath = Join-Path $resultsDir $logName
            $experimentLabel = $normalizedPolicy

            $env:SIM_RUN_ID = $runId
            $env:MAP_ID = $MapId
            $env:RESTAURANT_MAP = $MapId
            $env:EXPERIMENT_MODE = $ExperimentMode.ToUpperInvariant()
            $env:DYNAMIC_ENVIRONMENT = $DynamicEnvironment.ToLowerInvariant()
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
            $env:METRICS_DIR = $metricsDir

            "$experimentLabel,$MapId,$($ExperimentMode.ToUpperInvariant()),$($DynamicEnvironment.ToLowerInvariant()),$normalizedPolicy,$run,$runId,$seed,$MaxCompletedRequests,$MaxSimTime,$StopDelay,$FirstPeriod,$MinPeriod,$MaxPeriod,$HybridWaitWeight,$HybridDistanceWeight,$worldPath,$logName" |
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

            $webotsArgs += $worldPath

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
python $parserPath --results-dir $resultsDir --max-wait-weight $MaxWaitWeight

Write-Host ""
Write-Host "Results directory:"
Write-Host $resultsDir
Write-Host "Open metrics_summary.csv to compare policies."
