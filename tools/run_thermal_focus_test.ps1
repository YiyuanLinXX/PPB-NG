param(
  [Parameter(Mandatory = $true, Position = 0)]
  [ValidatePattern('^[A-Za-z0-9_-]{1,48}$')]
  [string]$DatasetName,
  [string]$DeviceId = '00111C0408CD',
  [string]$ArduinoPort = 'COM9',
  [switch]$NoViewer
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$runtimePython = 'C:\Users\cairlab\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$cameraExe = Join-Path $workspace 'build\ppbng_thermal\Release\ppbng_a6701_focus_recorder.exe'
$healthExe = Join-Path $workspace 'build\ppbng_thermal\Release\ppbng_a6701_health_check.exe'
$watcherScript = Join-Path $workspace 'tools\watch_a6701_focus_png.py'
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$output = Join-Path $workspace "hardware_test_data\thermal_focus_${DatasetName}_${stamp}"
$control = Join-Path $workspace "hardware_test_data\focus_control_${DatasetName}_${stamp}"
$stdoutLog = Join-Path $control 'camera.stdout.log'
$stderrLog = Join-Path $control 'camera.stderr.log'
$watcherStdout = Join-Path $control 'watcher.stdout.log'
$watcherStderr = Join-Path $control 'watcher.stderr.log'
$heartbeat = Join-Path $control 'HOST_HEARTBEAT'
$stopRequest = Join-Path $control 'STOP_REQUESTED'

foreach ($required in @($runtimePython, $cameraExe, $healthExe, $watcherScript)) {
  if (-not (Test-Path -LiteralPath $required)) { throw "Required file missing: $required" }
}
$existing = @(Get-CimInstance Win32_Process | Where-Object {
  $_.Name -match 'ppbng_a6701_focus_recorder|watch_a6701_focus_png'
})
if ($existing.Count -ne 0) { throw 'A thermal focus process is already running.' }

New-Item -ItemType Directory -Path $control | Out-Null
[IO.File]::WriteAllText($heartbeat, (Get-Date).ToString('o'))
$serial = [System.IO.Ports.SerialPort]::new(
  $ArduinoPort, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.DtrEnable = $true
$serial.RtsEnable = $true
$serial.ReadTimeout = 500
$serial.WriteTimeout = 1000
$camera = $null
$watcher = $null
$oldControlC = [Console]::TreatControlCAsInput
$exitCode = 1

try {
  [Console]::TreatControlCAsInput = $true
  $serial.Open()
  Start-Sleep -Seconds 2
  [void]$serial.ReadExisting()
  $serial.Write("STOP`n")
  Start-Sleep -Milliseconds 300
  [void]$serial.ReadExisting()
  $serial.Write("STATUS`n")
  Start-Sleep -Milliseconds 500
  $status = $serial.ReadExisting().Trim()
  Write-Host "Arduino: $status"
  if ($status -notmatch '^STOPPED,' -or $status -notmatch 'watchdog_ms=3000') {
    throw 'Arduino watchdog firmware is not active. Upload the updated sketch before running this test.'
  }

  $camera = Start-Process -FilePath $cameraExe `
    -ArgumentList @($DeviceId, $output, $control) `
    -WindowStyle Hidden -RedirectStandardOutput $stdoutLog -RedirectStandardError $stderrLog -PassThru
  $watcher = Start-Process -FilePath $runtimePython `
    -ArgumentList @($watcherScript, $output) `
    -WindowStyle Hidden -RedirectStandardOutput $watcherStdout -RedirectStandardError $watcherStderr -PassThru

  $armed = $false
  for ($i = 0; $i -lt 120; ++$i) {
    [IO.File]::SetLastWriteTimeUtc($heartbeat, [DateTime]::UtcNow)
    Start-Sleep -Milliseconds 100
    $camera.Refresh()
    if ($camera.HasExited) { break }
    if ((Test-Path -LiteralPath $stdoutLog) -and
      ((Get-Content -LiteralPath $stdoutLog -Raw -ErrorAction SilentlyContinue) -match
        'ARMED_WAITING_FOR_EXTERNAL_PULSES')) {
      $armed = $true
      break
    }
  }
  if (-not $armed) { throw 'Camera did not reach the external-trigger armed state.' }

  $serial.Write("START`n")
  Write-Host "`nThermal focus capture is running."
  Write-Host "Dataset: $output"
  Write-Host 'Adjust the lens and target as needed. Press ENTER or Ctrl+C here to stop safely.'

  $viewerOpened = $false
  $lastKeepalive = [DateTime]::UtcNow.AddSeconds(-2)
  while ($true) {
    [IO.File]::SetLastWriteTimeUtc($heartbeat, [DateTime]::UtcNow)
    if (([DateTime]::UtcNow - $lastKeepalive).TotalMilliseconds -ge 800) {
      $serial.Write("KEEPALIVE`n")
      $lastKeepalive = [DateTime]::UtcNow
    }
    $serialText = $serial.ReadExisting()
    if ($serialText) { Write-Host -NoNewline $serialText }
    $camera.Refresh()
    if ($camera.HasExited) { throw "Camera recorder exited unexpectedly with code $($camera.ExitCode)." }

    $latest = Join-Path $output 'latest_preview.png'
    $viewer = Join-Path $output 'viewer.html'
    if (-not $viewerOpened -and -not $NoViewer -and
      (Test-Path -LiteralPath $latest) -and (Test-Path -LiteralPath $viewer)) {
      Start-Process -FilePath $viewer | Out-Null
      $viewerOpened = $true
    }
    if ([Console]::KeyAvailable) {
      $key = [Console]::ReadKey($true)
      if ($key.Key -eq [ConsoleKey]::Enter -or
        ($key.Key -eq [ConsoleKey]::C -and ($key.Modifiers -band [ConsoleModifiers]::Control))) {
        break
      }
    }
    Start-Sleep -Milliseconds 100
  }
  $exitCode = 0
}
catch {
  Write-Error $_
}
finally {
  Write-Host "`nStopping trigger and camera safely..."
  if ($serial.IsOpen) {
    try { $serial.Write("STOP`n"); Start-Sleep -Milliseconds 300; Write-Host $serial.ReadExisting().Trim() } catch {}
  }
  [IO.File]::WriteAllText($stopRequest, (Get-Date).ToString('o'))
  [IO.File]::SetLastWriteTimeUtc($heartbeat, [DateTime]::UtcNow)
  if ($camera -and -not $camera.HasExited) {
    if (-not $camera.WaitForExit(10000)) {
      Write-Warning 'Camera recorder has not exited yet; Arduino is stopped. Waiting for recorder heartbeat timeout.'
      [void]$camera.WaitForExit(8000)
    }
  }
  if ($watcher -and -not $watcher.HasExited) { [void]$watcher.WaitForExit(10000) }
  if ($serial.IsOpen) {
    try { $serial.Write("STATUS`n"); Start-Sleep -Milliseconds 300; Write-Host "Arduino final: $($serial.ReadExisting().Trim())" } catch {}
    $serial.Close()
  }
  $serial.Dispose()
  [Console]::TreatControlCAsInput = $oldControlC

  if (Test-Path -LiteralPath $output) {
    Copy-Item -LiteralPath $stdoutLog -Destination (Join-Path $output 'camera.stdout.log') -ErrorAction SilentlyContinue
    Copy-Item -LiteralPath $stderrLog -Destination (Join-Path $output 'camera.stderr.log') -ErrorAction SilentlyContinue
    Copy-Item -LiteralPath $watcherStdout -Destination (Join-Path $output 'watcher.stdout.log') -ErrorAction SilentlyContinue
    Copy-Item -LiteralPath $watcherStderr -Destination (Join-Path $output 'watcher.stderr.log') -ErrorAction SilentlyContinue
  }
  if ($camera -and $camera.HasExited) { Write-Host "Camera recorder exit: $($camera.ExitCode)" }
  if (Test-Path -LiteralPath $output) { Write-Host "Saved dataset: $output" }

  if ($camera -and $camera.HasExited) {
    Write-Host "`nPost-capture A6701 health check:"
    & $healthExe $DeviceId
    if ($LASTEXITCODE -ne 0) { Write-Warning 'Post-capture health check failed.'; $exitCode = 1 }
  } else {
    Write-Warning 'Camera recorder exit was not confirmed; do not start another camera process.'
    $exitCode = 1
  }
}

exit $exitCode
