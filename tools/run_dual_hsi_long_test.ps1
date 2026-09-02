param(
  [Parameter(Mandatory = $true, Position = 0)]
  [ValidatePattern('^[A-Za-z0-9_-]{1,48}$')]
  [string]$DatasetName,

  [Parameter(Position = 1)]
  [ValidateRange(1, 720)]
  [int]$DurationMinutes = 120,

  [Parameter(Position = 2)]
  [ValidateSet('both', 'fx10e', 'swir')]
  [string]$CameraSelection = 'both',

  [switch]$Unattended
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$ros2 = Join-Path $PSScriptRoot 'ros2.cmd'
$config = Join-Path $workspace 'src\ppbng_bringup\config\ppbng_config.yaml'
$outputRoot = Join-Path $workspace 'data'
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$sessionId = "${DatasetName}_${stamp}"
$session = Join-Path $outputRoot $sessionId
$segments = Join-Path $session 'segments'
$control = Join-Path $session 'control'
$launchStdout = Join-Path $control 'dual_hsi_launch.stdout.log'
$launchStderr = Join-Path $control 'dual_hsi_launch.stderr.log'
$launchProcess = $null
$startedCameras = [System.Collections.Generic.List[string]]::new()
$consoleAvailable = $true
$oldControlC = $null
try {
  $oldControlC = [Console]::TreatControlCAsInput
} catch [System.IO.IOException] {
  $consoleAvailable = $false
}
$exitCode = 1
$runError = $null
$sessionStartUtc = [DateTime]::UtcNow
$nextFaultCheck = [DateTime]::UtcNow
$serviceCallSequence = 0
$cameras = if ($CameraSelection -eq 'both') {
  @('fx10e', 'swir')
} else {
  @($CameraSelection)
}

function Get-CameraService {
  param([Parameter(Mandatory = $true)][string]$CameraName, [Parameter(Mandatory = $true)][string]$Action)
  $namespace = if ($CameraSelection -eq 'both') { $CameraName } else { "hsi_$CameraName" }
  return "/$namespace/$Action"
}

function Get-Sha256Hex {
  param([Parameter(Mandatory = $true)][string]$Path)
  $stream = [IO.File]::OpenRead($Path)
  $algorithm = [Security.Cryptography.SHA256]::Create()
  try {
    return [BitConverter]::ToString($algorithm.ComputeHash($stream)).Replace('-', '')
  } finally {
    $algorithm.Dispose()
    $stream.Dispose()
  }
}

function Invoke-RosService {
  param(
    [Parameter(Mandatory = $true)][string]$Service,
    [Parameter(Mandatory = $true)][string]$Type,
    [Parameter(Mandatory = $true)][string]$Request,
    [Parameter(Mandatory = $true)][string]$Expected
  )
  Write-Host "Calling $Service ..."
  Write-Host 'ROS CLI discovery normally takes 5-10 s; hardware start can take 20-30 s.'
  Write-Host 'Please wait for the response. Keyboard input is ignored during this transaction.'
  $script:serviceCallSequence++
  $safeName = $Service.Trim('/').Replace('/', '_')
  $serviceStdout = Join-Path $control ("service_{0:D2}_{1}.stdout.log" -f `
    $script:serviceCallSequence, $safeName)
  $serviceStderr = Join-Path $control ("service_{0:D2}_{1}.stderr.log" -f `
    $script:serviceCallSequence, $safeName)
  # Run the CLI in a hidden process so repeated Ctrl+C cannot be delivered as
  # literal ^C bytes to ros2.cmd and corrupt a safety-critical stop command.
  # The full output remains in control/ for postmortem diagnosis.
  $quotedRequest = '"' + $Request.Replace('"', '\"') + '"'
  $process = Start-Process -FilePath $ros2 `
    -ArgumentList @('service', 'call', $Service, $Type, $quotedRequest) `
    -WindowStyle Hidden -RedirectStandardOutput $serviceStdout `
    -RedirectStandardError $serviceStderr -PassThru
  # Force Windows PowerShell 5.1 to retain the native process handle. Without
  # this access, ExitCode remains $null even after a successful WaitForExit.
  $processHandle = $process.Handle
  $startedAt = [DateTime]::UtcNow
  $nextWaitMessage = $startedAt.AddSeconds(5)
  while (-not $process.HasExited) {
    $process.Refresh()
    while ($consoleAvailable -and [Console]::KeyAvailable) {
      [void][Console]::ReadKey($true)
      Write-Host 'Input ignored: waiting for the current ROS service transaction to finish safely.' -ForegroundColor Yellow
    }
    if ([DateTime]::UtcNow -ge $nextWaitMessage) {
      $elapsed = [Math]::Floor(([DateTime]::UtcNow - $startedAt).TotalSeconds)
      Write-Host "Still waiting for $Service response ($elapsed s elapsed)..."
      $nextWaitMessage = [DateTime]::UtcNow.AddSeconds(5)
    }
    Start-Sleep -Milliseconds 200
  }
  [void]$process.WaitForExit()
  # Windows PowerShell 5.1 can retain a stale/null ExitCode on a redirected
  # Start-Process object until it is explicitly refreshed after WaitForExit.
  $process.Refresh()
  $nativeExitCode = $process.ExitCode
  $stdoutText = if (Test-Path -LiteralPath $serviceStdout) {
    Get-Content -LiteralPath $serviceStdout -Raw
  } else { '' }
  $stderrText = if (Test-Path -LiteralPath $serviceStderr) {
    Get-Content -LiteralPath $serviceStderr -Raw
  } else { '' }
  $text = ($stdoutText + [Environment]::NewLine + $stderrText).Trim()
  if ($text) { Write-Host $text }
  $responseMatched = $text -match $Expected
  if ($nativeExitCode -ne 0 -or -not $responseMatched) {
    throw "ROS service failed or returned a refusal: $Service " +
      "(exit=$nativeExitCode, expected_response_matched=$responseMatched)"
  }
}

function Wait-RosServices {
  param([string[]]$Names, [int]$TimeoutSeconds = 45)
  $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  $lastListExitCode = 0
  do {
    $savedPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
      $listed = @(& $ros2 service list 2>$null)
      $nativeExitCode = $LASTEXITCODE
    } finally {
      $ErrorActionPreference = $savedPreference
    }
    $lastListExitCode = $nativeExitCode
    if ($nativeExitCode -eq 0) {
      $missing = @($Names | Where-Object { $_ -notin $listed })
      if ($missing.Count -eq 0) { return }
    }
    if ($launchProcess -and $launchProcess.HasExited) {
      throw "ROS launch exited before its services became ready (exit code $($launchProcess.ExitCode))."
    }
    Start-Sleep -Milliseconds 500
  } while ([DateTime]::UtcNow -lt $deadline)
  throw "Timed out waiting for ROS services (last service-list exit=$lastListExitCode): $($missing -join ', ')"
}

foreach ($required in @($ros2, $config)) {
  if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
    throw "Required file missing: $required"
  }
}

$existing = @(Get-CimInstance Win32_Process | Where-Object {
  $_.Name -eq 'hsi_production_node.exe'
})
if ($existing.Count -ne 0) {
  throw 'An HSI production node is already running. Stop it before starting a new test.'
}
if (Test-Path -LiteralPath $session) {
  throw "Refusing to reuse an existing dataset directory: $session"
}

New-Item -ItemType Directory -Path $segments | Out-Null
New-Item -ItemType Directory -Path $control | Out-Null
Copy-Item -LiteralPath $config -Destination (Join-Path $control 'ppbng_config.yaml')
$identity = [System.Collections.Generic.List[string]]::new()
$identity.Add("session_id=$sessionId")
$identity.Add("start_utc=$($sessionStartUtc.ToString('o'))")
foreach ($artifact in @(
  $config,
  (Join-Path $workspace 'install\ppbng_hsi\lib\ppbng_hsi\hsi_production_node.exe'),
  (Join-Path $workspace 'install\ppbng_hsi\lib\ppbng_hsi\ppbng_verify_dataset.exe')
)) {
  if (Test-Path -LiteralPath $artifact -PathType Leaf) {
    $hash = Get-Sha256Hex -Path $artifact
    $identity.Add("sha256=$hash path=$artifact")
  }
}
[IO.File]::WriteAllLines((Join-Path $control 'build_identity.txt'), $identity)

try {
  if (-not $Unattended -and -not $consoleAvailable) {
    throw 'Interactive HSI capture requires a real console. Use -Unattended for SSH/background execution.'
  }
  if ($consoleAvailable) {
    [Console]::TreatControlCAsInput = $true
  }
  $launchArguments = if ($CameraSelection -eq 'both') {
    @('launch', 'ppbng_bringup', 'hsi_dual_camera.launch.py',
      'hardware_enabled:=true', "config_file:=$config")
  } else {
    @('launch', 'ppbng_bringup', 'hsi_camera.launch.py', "camera:=$CameraSelection",
      'hardware_enabled:=true', "config_file:=$config")
  }
  $launchProcess = Start-Process -FilePath $ros2 `
    -ArgumentList $launchArguments `
    -WindowStyle Hidden -RedirectStandardOutput $launchStdout `
    -RedirectStandardError $launchStderr -PassThru

  # The Windows ROS underlay/workspace batch hooks use transient setup files.
  # Do not launch a second ros2.cmd while the background launch is still
  # importing those hooks, or the two setup transactions can corrupt one
  # another's Python environment before any node is created.
  Start-Sleep -Seconds 10

  $services = @($cameras | ForEach-Object {
    $cameraName = $_
    @('prepare', 'arm', 'start', 'begin_dark', 'start_sample', 'stop') |
      ForEach-Object { Get-CameraService $cameraName $_ }
  })
  Wait-RosServices -Names $services

  $portableSession = $session.Replace('\', '/')
  foreach ($camera in $cameras) {
    $request = "{request_id: 'prepare-$camera-$stamp', session_id: '$sessionId', session_directory: '$portableSession'}"
    Invoke-RosService (Get-CameraService $camera 'prepare') 'ppbng_interfaces/srv/PrepareDevice' `
      $request 'accepted\s*(?:=|:)\s*true'
    Invoke-RosService (Get-CameraService $camera 'arm') 'std_srvs/srv/Trigger' '{}' `
      'success\s*(?:=|:)\s*true'
  }

  # Starts are intentionally sequential. The installed vendor runtimes are not
  # safe when their complete initialization transactions overlap.
  foreach ($camera in $cameras) {
    Invoke-RosService (Get-CameraService $camera 'start') 'std_srvs/srv/Trigger' '{}' `
      'success\s*(?:=|:)\s*true'
    $startedCameras.Add($camera)
  }

  Write-Host "`nRequested HSI camera(s) are ready with shutters closed: $($cameras -join ', ')." -ForegroundColor Green
  if ($Unattended) {
    Write-Host 'Unattended mode: acquiring dark references behind the closed internal shutters.'
  } else {
    Write-Host 'Cover the requested camera lens(es) completely, then press ENTER to acquire dark references.'
    [void](Read-Host)
  }

  foreach ($camera in $cameras) {
    Invoke-RosService (Get-CameraService $camera 'begin_dark') 'std_srvs/srv/Trigger' '{}' `
      'success\s*(?:=|:)\s*true'
  }
  Write-Host 'Collecting the configured 5-second dark references...'
  Start-Sleep -Seconds 12

  if ($Unattended) {
    Write-Host "`nUnattended mode: opening shutters and starting the current scene."
  } else {
    Write-Host "`nRemove the lens cover(s), confirm the scene is ready, then press ENTER."
    [void](Read-Host)
  }
  foreach ($camera in $cameras) {
    Invoke-RosService (Get-CameraService $camera 'start_sample') 'std_srvs/srv/Trigger' '{}' `
      'success\s*(?:=|:)\s*true'
  }

  Write-Host "`nHSI sampling started for $DurationMinutes minute(s): $($cameras -join ', ')." -ForegroundColor Green
  Write-Host "Dataset: $session"
  if ($Unattended) {
    Write-Host 'Unattended run will stop automatically at the deadline and close all requested cameras safely.'
  } else {
    Write-Host 'Press ENTER or Ctrl+C in this terminal to stop early and close all requested cameras safely.'
  }
  $deadline = [DateTime]::UtcNow.AddMinutes($DurationMinutes)
  $nextReport = [DateTime]::UtcNow
  while ([DateTime]::UtcNow -lt $deadline) {
    $launchProcess.Refresh()
    if ($launchProcess.HasExited) {
      throw "ROS launch exited unexpectedly with code $($launchProcess.ExitCode)."
    }
    if (-not $Unattended -and $consoleAvailable -and [Console]::KeyAvailable) {
      $key = [Console]::ReadKey($true)
      if ($key.Key -eq [ConsoleKey]::Enter -or
        ($key.Key -eq [ConsoleKey]::C -and
        ($key.Modifiers -band [ConsoleModifiers]::Control))) {
        Write-Host 'Operator requested an early stop.'
        break
      }
    }
    if ([DateTime]::UtcNow -ge $nextFaultCheck) {
      foreach ($camera in $cameras) {
        $eventPath = Join-Path $segments "${camera}_events.ndjson"
        if (Test-Path -LiteralPath $eventPath -PathType Leaf) {
          $lastEvent = Get-Content -LiteralPath $eventPath -Tail 1 -ErrorAction SilentlyContinue
          if ($lastEvent -match 'SAMPLE FAIL-FAST') {
            $runError = "$camera reported a sample continuity fault; acquisition was stopped immediately."
            Write-Host "`nCRITICAL: $runError" -ForegroundColor Red
            Write-Host "Diagnostic evidence: $eventPath" -ForegroundColor Yellow
            $exitCode = 1
            break
          }
        }
      }
      if ($runError) { break }
      $nextFaultCheck = [DateTime]::UtcNow.AddSeconds(1)
    }
    if ([DateTime]::UtcNow -ge $nextReport) {
      $remaining = [Math]::Max(0, [Math]::Ceiling(($deadline - [DateTime]::UtcNow).TotalMinutes))
      Write-Host "HSI launch is still running; approximately $remaining minute(s) remaining."
      $nextReport = [DateTime]::UtcNow.AddMinutes(1)
    }
    Start-Sleep -Milliseconds 200
  }
  if (-not $runError) { $exitCode = 0 }
}
catch {
  $runError = $_.Exception.Message
  Write-Host "LONG TEST FAILED BEFORE COMPLETION: $runError" -ForegroundColor Red
  try {
    [IO.File]::WriteAllText((Join-Path $control 'script_error.txt'), $runError)
  } catch {}
}
finally {
  Write-Host "`nStopping requested HSI stream(s) and releasing vendor SDK handles..."
  if ($startedCameras.Count -ne 0) {
    foreach ($camera in @($startedCameras)) {
      try {
        Invoke-RosService (Get-CameraService $camera 'stop') 'std_srvs/srv/Trigger' '{}' `
          'success\s*(?:=|:)\s*true'
      } catch {
        Write-Warning "Safe stop was not confirmed for $camera`: $($_.Exception.Message)"
        $exitCode = 1
      }
    }
  }
  if ($launchProcess -and -not $launchProcess.HasExited) {
    $savedPreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
      & taskkill.exe /PID $launchProcess.Id /T /F 2>$null | Out-Null
    } finally {
      $ErrorActionPreference = $savedPreference
    }
    [void]$launchProcess.WaitForExit(10000)
  }
  if ($consoleAvailable) {
    [Console]::TreatControlCAsInput = $oldControlC
  }

  $specimLogRoot = 'C:\ProgramData\Specim'
  if (Test-Path -LiteralPath $specimLogRoot -PathType Container) {
    $vendorLogDestination = Join-Path $control 'specim_logs'
    try {
      $vendorLogs = @(Get-ChildItem -LiteralPath $specimLogRoot -File -Recurse -ErrorAction Stop |
        Where-Object { $_.LastWriteTimeUtc -ge $sessionStartUtc })
      foreach ($vendorLog in $vendorLogs) {
        $relative = $vendorLog.FullName.Substring($specimLogRoot.Length).TrimStart('\')
        $destination = Join-Path $vendorLogDestination $relative
        $destinationDirectory = Split-Path -Parent $destination
        New-Item -ItemType Directory -Path $destinationDirectory -Force | Out-Null
        Copy-Item -LiteralPath $vendorLog.FullName -Destination $destination -Force
      }
      if ($vendorLogs.Count -ne 0) {
        Write-Host "Copied $($vendorLogs.Count) read-only vendor diagnostic log(s) into the dataset."
      }
    } catch {
      Write-Warning "Could not copy all Specim diagnostic logs: $($_.Exception.Message)"
    }
  }

  if (Test-Path -LiteralPath $session) {
    Write-Host "Saved dataset: $session"
    $indices = @(Get-ChildItem -LiteralPath $segments -Filter '*.index.csv' -File -ErrorAction SilentlyContinue)
    if ($indices.Count -eq 0) {
      Write-Warning 'No HSI data files were created; integrity verification was skipped.'
    } else {
      foreach ($camera in $cameras) {
        Write-Host "`nRead-only quick integrity verification: $camera"
        Write-Host 'This checks all metadata/extents plus sampled first/last and interval CRCs.'
        $savedPreference = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        try {
          & (Join-Path $PSScriptRoot 'verify_dataset.cmd') $session '--hsi-only' $camera '--quick'
          $verifyExitCode = $LASTEXITCODE
        } finally {
          $ErrorActionPreference = $savedPreference
        }
        if ($verifyExitCode -ne 0) {
          Write-Warning "$camera dataset verification failed. Preserve the dataset and logs for diagnosis."
          $exitCode = 1
        }
      }
    }
  }
}

exit $exitCode
