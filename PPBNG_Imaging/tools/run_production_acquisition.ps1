param(
  [Parameter(Mandatory = $true, Position = 0)]
  [ValidatePattern('^[A-Za-z0-9_-]{1,48}$')]
  [string]$DatasetName,

  [Parameter(Position = 1)]
  [ValidateRange(1, 720)]
  [int]$DurationMinutes = 120,

  [switch]$DisableGnss
)

$ErrorActionPreference = 'Stop'
Remove-Item Env:ROS_LOCALHOST_ONLY -ErrorAction SilentlyContinue
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = 'LOCALHOST'
$env:ROS_DOMAIN_ID = '47'
$env:ROS_STATIC_PEERS = '192.168.5.200'
$workspace = Split-Path -Parent $PSScriptRoot
$ros2 = Join-Path $PSScriptRoot 'ros2.cmd'
$controlPort = 45847
$config = Join-Path $workspace 'src\ppbng_bringup\config\ppbng_config.yaml'
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$runLog = Join-Path (Join-Path $workspace 'logs') "production_$stamp"
$launchOut = Join-Path $runLog 'launch.stdout.log'
$launchErr = Join-Path $runLog 'launch.stderr.log'
$launchProcess = $null
$startedRequestAccepted = $false
$stopRequested = $false
$exitCode = 1
$callSequence = 0
$session = $null
$recordingStartedAt = $null
$consoleAvailable = $true
$oldControlC = $null
$operatorWarningsSeen = $false
$operatorWarningsText = ''
. (Join-Path $PSScriptRoot 'production_operator_alerts.ps1')
try { $oldControlC = [Console]::TreatControlCAsInput } catch { $consoleAvailable = $false }

function Invoke-ProductionControl {
  param([string[]]$CommandArguments, [string]$Expected, [int]$TimeoutSeconds = 90, [switch]$Quiet)
  $script:callSequence++
  $name = $CommandArguments[0]
  $out = Join-Path $runLog ("service_{0:D2}_{1}.stdout.log" -f $script:callSequence, $name)
  $err = Join-Path $runLog ("service_{0:D2}_{1}.stderr.log" -f $script:callSequence, $name)
  if (-not $Quiet) { Write-Host "Calling production control: $name ..." }
  $startedAt = [DateTime]::UtcNow
  $nextNotice = $startedAt.AddSeconds(5)
  $client = $null
  while (-not $client) {
    if (([DateTime]::UtcNow - $startedAt).TotalSeconds -ge $TimeoutSeconds) {
      throw "Timed out after $TimeoutSeconds seconds connecting to production control: $name"
    }
    if ($launchProcess -and $launchProcess.HasExited) {
      throw "ROS launch exited while waiting for production control: $name"
    }
    while ($consoleAvailable -and [Console]::KeyAvailable) {
      [void][Console]::ReadKey($true)
      Write-Host 'Input ignored while a safety-critical ROS transaction is pending.' -ForegroundColor Yellow
    }
    $candidate = $null
    try {
      $candidate = [Net.Sockets.TcpClient]::new()
      $attempt = $candidate.BeginConnect('127.0.0.1', $controlPort, $null, $null)
      if ($attempt.AsyncWaitHandle.WaitOne(500)) {
        $candidate.EndConnect($attempt)
        $client = $candidate
      } else {
        $candidate.Close()
      }
    } catch {
      if ($candidate) { $candidate.Close() }
    }
    if (-not $client -and [DateTime]::UtcNow -ge $nextNotice) {
      $elapsed = [Math]::Floor(([DateTime]::UtcNow - $startedAt).TotalSeconds)
      Write-Host "Still waiting for production control $name ($elapsed s elapsed)..."
      $nextNotice = [DateTime]::UtcNow.AddSeconds(5)
    }
    if (-not $client) { Start-Sleep -Milliseconds 200 }
  }
  try {
    $remainingMs = [Math]::Max(1000, [Math]::Min([int]::MaxValue,
      [int](($TimeoutSeconds - ([DateTime]::UtcNow - $startedAt).TotalSeconds) * 1000)))
    $client.ReceiveTimeout = $remainingMs
    $client.SendTimeout = $remainingMs
    $stream = $client.GetStream()
    $encoding = [Text.UTF8Encoding]::new($false)
    $writer = [IO.StreamWriter]::new($stream, $encoding, 1024, $true)
    $reader = [IO.StreamReader]::new($stream, $encoding, $false, 1024, $true)
    $writer.WriteLine(($CommandArguments -join "`t"))
    $writer.Flush()
    $responseLines = [Collections.Generic.List[string]]::new()
    while ($true) {
      $line = $reader.ReadLine()
      if ($null -eq $line) { throw 'Production control bridge closed before END.' }
      if ($line -eq 'END') { break }
      $responseLines.Add($line)
    }
    $text = ($responseLines -join [Environment]::NewLine).Trim()
  } finally {
    if ($writer) { $writer.Dispose() }
    if ($reader) { $reader.Dispose() }
    $client.Close()
  }
  [IO.File]::WriteAllText($out, $text + [Environment]::NewLine)
  [IO.File]::WriteAllText($err, '')
  if ($text -and -not $Quiet) { Write-Host $text }
  if ($text -notmatch $Expected) {
    throw "Production control failed or refused: $name"
  }
  return $text
}

function Read-AcquisitionState {
  $text = Invoke-ProductionControl @('status') 'success=true' 30 -Quiet
  $health = ConvertFrom-OperatorStatus $text
  if ($health.WarningCount -gt 0) {
    $script:operatorWarningsSeen = $true
    $script:operatorWarningsText = $health.Warnings
    Write-OperatorAlert $health.Warnings
  }
  return $health.State
}

function Wait-PreparationServicesReady {
  param([int]$TimeoutSeconds = 180)
  $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  $nextNotice = [DateTime]::UtcNow
  do {
    foreach ($startupLog in @($launchOut, $launchErr)) {
      if (Test-Path -LiteralPath $startupLog) {
        $failure = Select-String -LiteralPath $startupLog -Pattern 'process has died' -SimpleMatch | Select-Object -First 1
        if ($failure) {
          throw "A ROS node crashed before acquisition started: $($failure.Line.Trim()). Read $startupLog"
        }
      }
    }
    if ($launchProcess -and $launchProcess.HasExited) {
      throw "ROS launch exited while waiting for device services. Read $launchErr"
    }
    $text = Invoke-ProductionControl @('status') 'success=true' 60
    if ($text -match 'prepare_services_ready=true') {
      Write-Host 'All required device preparation services are discovered and ready.' -ForegroundColor Green
      return
    }
    if ([DateTime]::UtcNow -ge $nextNotice) {
      $missing = [regex]::Match($text, 'missing_prepare_services=([^;\r\n]*)')
      $detail = if ($missing.Success -and $missing.Groups[1].Value) {
        $missing.Groups[1].Value
      } else {
        'service discovery is still converging'
      }
      Write-Host "Waiting for required device services: $detail"
      $nextNotice = [DateTime]::UtcNow.AddSeconds(10)
    }
    Start-Sleep -Seconds 2
  } while ([DateTime]::UtcNow -lt $deadline)
  throw 'Timed out waiting for all required device preparation services.'
}

function Wait-AcquisitionState {
  param([int]$Expected, [string]$Description, [int]$TimeoutSeconds = 180)
  $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  $last = -1
  do {
    if ($launchProcess -and $launchProcess.HasExited) {
      throw "ROS launch exited while waiting for $Description. Read $launchErr"
    }
    $last = Read-AcquisitionState
    if ($last -eq $Expected) { return }
    if ($last -eq 9) { throw "Acquisition manager entered FAULT while waiting for $Description." }
    Start-Sleep -Seconds 2
  } while ([DateTime]::UtcNow -lt $deadline)
  throw "Timed out waiting for $Description (expected state $Expected, last state $last)."
}

function Show-DatasetSummary {
  param([string]$SessionDirectory, [datetime]$StartedAt)
  if (-not $SessionDirectory -or -not (Test-Path -LiteralPath $SessionDirectory -PathType Container)) {
    Write-Warning 'Dataset summary skipped because the session directory was not resolved.'
    return
  }
  $files = @(Get-ChildItem -LiteralPath $SessionDirectory -File -Recurse -ErrorAction SilentlyContinue)
  $totalBytes = [double](($files | Measure-Object -Property Length -Sum).Sum)
  $elapsedSeconds = [Math]::Max(1.0, ([DateTime]::UtcNow - $StartedAt).TotalSeconds)
  $mibPerSecond = $totalBytes / 1MB / $elapsedSeconds
  $driveName = ([System.IO.Path]::GetPathRoot($SessionDirectory)).TrimEnd('\').TrimEnd(':')
  $drive = Get-PSDrive -Name $driveName -ErrorAction SilentlyContinue
  Write-Host "`nRead-only dataset summary:" -ForegroundColor Cyan
  Write-Host ("  Files: {0}" -f $files.Count)
  Write-Host ("  Data size: {0:N2} GiB" -f ($totalBytes / 1GB))
  Write-Host ("  Average recording write rate: {0:N2} MiB/s" -f $mibPerSecond)
  if ($drive) { Write-Host ("  {0}: free space: {1:N2} GiB" -f $driveName, ($drive.Free / 1GB)) }
}

if (-not $consoleAvailable) { throw 'An interactive terminal is required for HSI dark/sample confirmation.' }
foreach ($path in @($ros2, $config)) {
  if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Required file missing: $path" }
}
$configText = Get-Content -LiteralPath $config -Raw
foreach ($requiredText in @('configured: true', 'hardware_enabled: true',
    'backend: "uno_r4_ascii"', 'pps_required: false')) {
  if ($configText -notmatch [regex]::Escape($requiredText)) {
    throw "Production config is not authorized/ready: missing '$requiredText' in ppbng_config.yaml"
  }
}
if ($configText -match 'TO_BE_CONFIRMED|COM__REQUIRED__') {
  throw 'Production config still contains an unresolved COM/device placeholder.'
}
if (@(Get-Process -Name 'production_acquisition_manager_node' -ErrorAction SilentlyContinue).Count) {
  throw 'A production acquisition manager is already running. Stop it before starting a new task.'
}
New-Item -ItemType Directory -Path $runLog -Force | Out-Null
Copy-Item -LiteralPath $config -Destination (Join-Path $runLog 'ppbng_config.yaml')
# Freeze a run-specific copy and align capacity planning with the actual requested
# duration. Keep every device setting and all reserve/headroom safeguards unchanged.
$effectiveConfigText = Set-RunPlannedDuration -ConfigText $configText -Minutes $DurationMinutes
if ($DisableGnss) {
  $effectiveConfigText = Set-RunGnssDisabled -ConfigText $effectiveConfigText
  Write-Host 'Indoor test: GNSS is disabled for this run only. No position data will be recorded.' -ForegroundColor Yellow
}
$config = Join-Path $runLog 'machine_config.effective.yaml'
[IO.File]::WriteAllText($config, $effectiveConfigText, [Text.UTF8Encoding]::new($false))
Write-Host "Frozen run config: $config (storage plan: $DurationMinutes minute(s))"

try {
  [Console]::TreatControlCAsInput = $true
  Write-Host 'Starting all ROS 2 system nodes. Do not press keys until the HSI dark prompt appears.' -ForegroundColor Cyan
  Write-Host "Live launch log: $launchOut"
  $launchProcess = Start-Process -FilePath $ros2 -ArgumentList @(
    'launch', 'ppbng_bringup', 'production.launch.py', 'hardware_enabled:=true',
    "machine_config:=$config", 'machine_id:=PPBNG-WINDOWS-IPC') -WindowStyle Hidden `
    -RedirectStandardOutput $launchOut -RedirectStandardError $launchErr -PassThru
  # production.launch.py registers the manager/control endpoints first, then
  # introduces the hardware graph after 10 seconds.  ROS environment setup and
  # vendor DLL loading vary substantially between cold and warm starts, so a
  # fixed sleep is unsafe.  Wait for every required PrepareDevice server to be
  # positively discovered before sending the first lifecycle request.
  Wait-PreparationServicesReady 240
  if (Test-Path -LiteralPath $launchOut) {
    $earlyFailure = Select-String -LiteralPath $launchOut -Pattern 'process has died' -SimpleMatch
    if ($earlyFailure) {
      throw "A mandatory ROS node exited during startup: $($earlyFailure.Line.Trim())"
    }
  }
  Write-Host 'ROS 2 launch is alive; contacting the acquisition manager...' -ForegroundColor Cyan
  $startText = Invoke-ProductionControl @('start', "start-$stamp", $DatasetName) `
    'accepted=true' 120
  $startedRequestAccepted = $true
  $sessionMatch = [regex]::Match($startText, 'session_directory=(?:''|")([^''"]+)')
  if (-not $sessionMatch.Success) {
    $sessionMatch = [regex]::Match($startText, '(?m)^session_directory(?:=|:)\s*([^\r\n]+)')
  }
  $session = if ($sessionMatch.Success) { $sessionMatch.Groups[1].Value.Trim(" '") } else { '(see launch log)' }
  Write-Host "`nInitializing devices in safe order. Dual-HSI SDK startup may take 30-60 seconds." -ForegroundColor Cyan
  Wait-AcquisitionState 2 'WAITING_FOR_DARK' 240

  Write-Host "`nBoth HSI cameras are ready with shutters closed. Cover both lenses, then press ENTER." -ForegroundColor Cyan
  [void](Read-Host)
  Invoke-ProductionControl @('dark', "dark-$stamp") 'accepted=true' 90 | Out-Null
  Write-Host 'Writing separate dark-reference files for both HSI cameras...'
  Wait-AcquisitionState 4 'WAITING_FOR_SAMPLE' 180

  Write-Host "`nDark capture is complete and triggers are stopped. Remove both covers, verify safety, then press ENTER." -ForegroundColor Cyan
  [void](Read-Host)
  Invoke-ProductionControl @('sample', "sample-$stamp") 'accepted=true' 120 | Out-Null
  Wait-AcquisitionState 6 'RECORDING' 180

  Write-Host "`nFull acquisition started. PPS is disabled; time quality remains explicitly marked." -ForegroundColor Green
  $recordingStartedAt = [DateTime]::UtcNow
  Write-Host "Dataset: $session"
  Write-Host "Planned duration: $DurationMinutes minute(s). Press ENTER or Ctrl+C for an early ordered stop."
  $deadline = [DateTime]::UtcNow.AddMinutes($DurationMinutes)
  $nextReport = [DateTime]::UtcNow
  $nextHealthCheck = [DateTime]::UtcNow
  while ([DateTime]::UtcNow -lt $deadline) {
    if ($launchProcess.HasExited) { throw 'ROS production launch exited unexpectedly.' }
    if ([Console]::KeyAvailable) {
      $key = [Console]::ReadKey($true)
      if ($key.Key -eq [ConsoleKey]::Enter -or
          ($key.Key -eq [ConsoleKey]::C -and
           ($key.Modifiers -band [ConsoleModifiers]::Control))) { break }
    }
    if ([DateTime]::UtcNow -ge $nextHealthCheck) {
      $state = Read-AcquisitionState
      if ($state -eq 9) { throw 'Acquisition manager entered FAULT during recording.' }
      if ($state -ne 6 -and $state -ne -1) { throw "Unexpected acquisition state during recording: $state" }
      $nextHealthCheck = [DateTime]::UtcNow.AddSeconds(5)
    }
    if ([DateTime]::UtcNow -ge $nextReport) {
      $remaining = [Math]::Max(0, [Math]::Ceiling(($deadline - [DateTime]::UtcNow).TotalMinutes))
      if ($operatorWarningsSeen) {
        Write-Host "RECORDING WITH ALERTS; approximately $remaining minute(s) remaining. Dataset requires review." -ForegroundColor Red
      } else {
        Write-Host "Recording; no device warnings reported by manager. Approximately $remaining minute(s) remaining."
      }
      $nextReport = [DateTime]::UtcNow.AddMinutes(1)
    }
    Start-Sleep -Milliseconds 250
  }

  Invoke-ProductionControl @('stop', "stop-$stamp", 'operator_or_duration_complete') `
    'accepted=true' 90 | Out-Null
  $stopRequested = $true
  Wait-AcquisitionState 0 'ordered stop completion' 180
  if ($operatorWarningsSeen) {
    $exitCode = 2
    Write-OperatorAlert $operatorWarningsText
    Write-Host 'STOPPED WITH DATA WARNINGS (exit 2): ordered shutdown completed; not a clean acquisition PASS.' -ForegroundColor Red
  } else {
    $exitCode = 0
    Write-Host "`nOrdered shutdown complete; no device warnings reported. Validate the dataset before accepting it." -ForegroundColor Green
  }
  Show-DatasetSummary -SessionDirectory $session -StartedAt $recordingStartedAt
} catch {
  Write-Host "PRODUCTION ACQUISITION FAILED: $($_.Exception.Message)" -ForegroundColor Red
} finally {
  if ($startedRequestAccepted -and -not $stopRequested -and $launchProcess -and
      -not $launchProcess.HasExited) {
    try {
      $cleanupState = Read-AcquisitionState
      if ($cleanupState -eq 9) {
        Write-Warning 'The manager already completed its fault-stop sequence; all opened devices were closed.'
      } elseif ($cleanupState -ne 0) {
        Invoke-ProductionControl @('stop', "cleanup-$stamp", 'runner_cleanup') `
          'accepted=true' 90 | Out-Null
        Wait-AcquisitionState 0 'cleanup completion' 180
      }
    } catch {
      Write-Warning "Ordered ROS cleanup was not confirmed: $($_.Exception.Message)"
      $exitCode = 1
    }
  }
  if ($launchProcess -and -not $launchProcess.HasExited) {
    taskkill.exe /PID $launchProcess.Id /T /F 2>$null | Out-Null
    [void]$launchProcess.WaitForExit(10000)
  }
  if ($consoleAvailable) { [Console]::TreatControlCAsInput = $oldControlC }
  Write-Host "Run logs: $runLog"
}

exit $exitCode
