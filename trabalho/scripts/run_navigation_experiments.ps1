[CmdletBinding()]
param(
    [string]$Webots = "webots",
    [string]$MapId = "map1",
    [string[]]$MapIds = @(),
    [string[]]$ScenarioLabels = @(),
    [string]$Policy = "NEAREST",
    [int]$Runs = 5,
    [int]$SeedBase = 3000,
    [int]$MaxCompletedRequests = 10,
    [double]$MaxSimTime = 420.0,
    [double]$StopDelay = 5.0,
    [double]$FirstPeriod = 5.0,
    [double]$MinPeriod = 8.0,
    [double]$MaxPeriod = 16.0,
    [double]$HybridWaitWeight = 1.0,
    [double]$HybridDistanceWeight = 30.0,
    [switch]$Render
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectDir = Resolve-Path (Join-Path $scriptDir "..")
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$resultsRoot = Join-Path $scriptDir "results"
$resultsDir = Join-Path $resultsRoot "navigation_experiments_$timestamp"
$logsDir = Join-Path $resultsDir "logs"
$metricsDir = Join-Path $resultsDir "metrics_results"
New-Item -ItemType Directory -Force -Path $logsDir | Out-Null
New-Item -ItemType Directory -Force -Path $metricsDir | Out-Null

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

$scenarios = @(
    @{ Label = "EXP1_STATIC";  Mode = "EXP1"; Dynamic = "false" },
    @{ Label = "EXP1_DYNAMIC"; Mode = "EXP1"; Dynamic = "true"  },
    @{ Label = "EXP2_STATIC";  Mode = "EXP2"; Dynamic = "false" },
    @{ Label = "EXP2_DYNAMIC"; Mode = "EXP2"; Dynamic = "true"  }
)

$selectedScenarioLabels = @(
    $ScenarioLabels |
        ForEach-Object { $_ -split "," } |
        ForEach-Object { $_.Trim().ToUpperInvariant() } |
        Where-Object { $_ }
)
if ($selectedScenarioLabels.Count -gt 0) {
    $scenarios = @(
        $scenarios |
            Where-Object { $selectedScenarioLabels -contains ([string]$_.Label).ToUpperInvariant() }
    )
    if ($scenarios.Count -eq 0) {
        throw "No scenarios matched: $($selectedScenarioLabels -join ', ')"
    }
}

$selectedMapIds = @(
    $MapIds |
        ForEach-Object { $_ -split "," } |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ }
)
if ($selectedMapIds.Count -eq 0) {
    $selectedMapIds = @($MapId)
}

function Resolve-WorldPathForMap {
    param([string]$SelectedMapId)

    switch ($SelectedMapId.ToLowerInvariant()) {
        "map1" { return (Resolve-Path (Join-Path $projectDir "worlds\restaurant_final.wbt")).Path }
        "map2" { return (Resolve-Path (Join-Path $projectDir "worlds\restaurant_mapa2.wbt")).Path }
        default { throw "Unsupported map id: $SelectedMapId" }
    }
}

try {
    foreach ($selectedMapId in $selectedMapIds) {
        $worldPath = Resolve-WorldPathForMap -SelectedMapId $selectedMapId

        foreach ($scenario in $scenarios) {
            for ($run = 1; $run -le $Runs; $run++) {
                $seed = $SeedBase + $run
                $label = [string]$scenario.Label
                $mode = [string]$scenario.Mode
                $dynamic = [string]$scenario.Dynamic
                $runId = "$selectedMapId-$label-$run"
                $logName = "$runId.log"
                $logPath = Join-Path $logsDir $logName

                $env:SIM_RUN_ID = $runId
                $env:MAP_ID = $selectedMapId
                $env:RESTAURANT_MAP = $selectedMapId
                $env:EXPERIMENT_MODE = $mode
                $env:DYNAMIC_ENVIRONMENT = $dynamic
                $env:REQUEST_POLICY = $Policy.ToUpperInvariant()
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

                "$label,$selectedMapId,$mode,$dynamic,$($Policy.ToUpperInvariant()),$run,$runId,$seed,$MaxCompletedRequests,$MaxSimTime,$StopDelay,$FirstPeriod,$MinPeriod,$MaxPeriod,$HybridWaitWeight,$HybridDistanceWeight,$worldPath,logs/$logName" |
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
}
finally {
    foreach ($name in $envNames) {
        [Environment]::SetEnvironmentVariable($name, $previousEnv[$name], "Process")
    }
}

$runsPath = Join-Path $metricsDir "runs.csv"
$finalRunsPath = Join-Path $resultsDir "final_runs.csv"
$summaryPath = Join-Path $resultsDir "metrics_summary.csv"

if (Test-Path $runsPath) {
    $rows = Import-Csv $runsPath
    $finalRows = $rows |
        Group-Object simulation_id |
        ForEach-Object {
            $_.Group |
                Sort-Object { [int]$_.completed_requests } |
                Select-Object -Last 1
        }

    $finalRows |
        Sort-Object map_id, experiment, dynamic_environment, { [int]$_.simulation_id } |
        Export-Csv -Path $finalRunsPath -NoTypeInformation -Encoding UTF8

    $summary = $finalRows |
        Group-Object map_id, experiment, dynamic_environment |
        ForEach-Object {
            $group = $_.Group
            [PSCustomObject]@{
                map_id = $group[0].map_id
                experiment = $group[0].experiment
                dynamic_environment = $group[0].dynamic_environment
                runs = $group.Count
                completed_total = ($group | Measure-Object completed_requests -Sum).Sum
                avg_completed = ($group | Measure-Object completed_requests -Average).Average
                avg_success_rate = ($group | Measure-Object success_rate -Average).Average
                avg_mission_time = ($group | Measure-Object mission_time -Average).Average
                avg_distance = ($group | Measure-Object distance -Average).Average
                avg_speed = ($group | Measure-Object avg_speed -Average).Average
                avg_collisions = ($group | Measure-Object collisions -Average).Average
                avg_wait = ($group | Measure-Object avg_wait -Average).Average
                avg_delivery = ($group | Measure-Object avg_delivery -Average).Average
                avg_return = ($group | Measure-Object avg_return -Average).Average
            }
        }

    $summary | Export-Csv -Path $summaryPath -NoTypeInformation -Encoding UTF8
}
else {
    Write-Warning "No runs.csv found at $runsPath"
}

Write-Host ""
Write-Host "Results directory:"
Write-Host $resultsDir
Write-Host "Open final_runs.csv and metrics_summary.csv to compare navigation experiments."
