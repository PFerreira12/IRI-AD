[CmdletBinding()]
param(
    [string]$Webots = "webots",
    [string[]]$WaitWeights = @("1.0"),
    [string[]]$DistanceWeights = @("0.0", "5.0", "10.0", "20.0", "30.0", "40.0"),
    [int]$Runs = 3,
    [int]$SeedBase = 2000,
    [int]$MaxCompletedRequests = 10,
    [double]$MaxSimTime = 300.0,
    [double]$StopDelay = 30.0,
    [double]$FirstPeriod = 5.0,
    [double]$MinPeriod = 8.0,
    [double]$MaxPeriod = 16.0,
    [double]$MaxWaitWeight = 0.25,
    [switch]$IncludeBaselines,
    [switch]$Render
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Format-WeightLabel {
    param([double]$Value)
    return ([string]$Value).Replace("-", "m").Replace(".", "p").Replace(",", "p")
}

function Convert-WeightList {
    param([string[]]$Values)

    $weights = @()

    foreach ($value in $Values) {
        $parts = ([string]$value).Split(",", [System.StringSplitOptions]::RemoveEmptyEntries)

        foreach ($part in $parts) {
            $cleanPart = $part.Trim()
            if ($cleanPart.Length -eq 0) {
                continue
            }

            $weights += [double]::Parse(
                $cleanPart,
                [System.Globalization.CultureInfo]::InvariantCulture
            )
        }
    }

    return $weights
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectDir = Resolve-Path (Join-Path $scriptDir "..")
$worldPath = Resolve-Path (Join-Path $projectDir "worlds\restaurant_final.wbt")
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$resultsRoot = Join-Path $scriptDir "results"
$resultsDir = Join-Path $resultsRoot "hybrid_weight_sweep_$timestamp"
New-Item -ItemType Directory -Force -Path $resultsDir | Out-Null

$configPath = Join-Path $resultsDir "run_config.csv"
"experiment_label,policy,repeat,run,seed,max_completed,max_sim_time,stop_delay,first_period,min_period,max_period,hybrid_wait_weight,hybrid_distance_weight,log" |
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

$experiments = @()
$normalizedWaitWeights = Convert-WeightList $WaitWeights
$normalizedDistanceWeights = Convert-WeightList $DistanceWeights

if ($IncludeBaselines) {
    $experiments += [pscustomobject]@{
        Label = "FIFO"
        Policy = "FIFO"
        WaitWeight = 1.0
        DistanceWeight = 0.0
    }
    $experiments += [pscustomobject]@{
        Label = "NEAREST"
        Policy = "NEAREST"
        WaitWeight = 1.0
        DistanceWeight = 0.0
    }
}

foreach ($waitWeight in $normalizedWaitWeights) {
    foreach ($distanceWeight in $normalizedDistanceWeights) {
        $waitLabel = Format-WeightLabel $waitWeight
        $distanceLabel = Format-WeightLabel $distanceWeight
        $experiments += [pscustomobject]@{
            Label = "HYBRID_w$waitLabel`_d$distanceLabel"
            Policy = "HYBRID"
            WaitWeight = $waitWeight
            DistanceWeight = $distanceWeight
        }
    }
}

try {
    foreach ($experiment in $experiments) {
        for ($run = 1; $run -le $Runs; $run++) {
            $seed = $SeedBase + $run
            $runId = "$($experiment.Label)-r$run"
            $logName = "$runId.log"
            $logPath = Join-Path $resultsDir $logName

            $env:SIM_RUN_ID = $runId
            $env:REQUEST_POLICY = $experiment.Policy
            $env:RM_RANDOM_SEED = [string]$seed
            $env:RM_MAX_COMPLETED_REQUESTS = [string]$MaxCompletedRequests
            $env:RM_MAX_SIM_TIME_S = [string]$MaxSimTime
            $env:RM_STOP_DELAY_S = [string]$StopDelay
            $env:RM_FIRST_PERIOD_S = [string]$FirstPeriod
            $env:RM_MIN_PERIOD_S = [string]$MinPeriod
            $env:RM_MAX_PERIOD_S = [string]$MaxPeriod
            $env:HYBRID_WAIT_WEIGHT = [string]$experiment.WaitWeight
            $env:HYBRID_DISTANCE_WEIGHT = [string]$experiment.DistanceWeight

            "$($experiment.Label),$($experiment.Policy),$run,$runId,$seed,$MaxCompletedRequests,$MaxSimTime,$StopDelay,$FirstPeriod,$MinPeriod,$MaxPeriod,$($experiment.WaitWeight),$($experiment.DistanceWeight),$logName" |
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
python $parserPath --results-dir $resultsDir --max-wait-weight $MaxWaitWeight

Write-Host ""
Write-Host "Results directory:"
Write-Host $resultsDir
Write-Host "Open metrics_ranked.csv for the optimization ranking."
