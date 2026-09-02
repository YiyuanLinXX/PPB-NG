param(
  [Parameter(Mandatory = $true, Position = 0)]
  [ValidatePattern('^[A-Za-z0-9_-]{1,48}$')]
  [string]$DatasetName,

  [Parameter(Position = 1)]
  [ValidateRange(1, 720)]
  [int]$DurationMinutes = 30
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$ros2 = Join-Path $PSScriptRoot 'ros2.cmd'
$config = Join-Path $workspace 'src\ppbng_bringup\config\ppbng_config.yaml'
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$sessionId = "${DatasetName}_${stamp}"
$session = Join-Path (Join-Path $workspace 'data') $sessionId
$segments = Join-Path $session 'segments'
$control = Join-Path $session 'control'
$launchStdout = Join-Path $control 'four_camera_launch.stdout.log'
$launchStderr = Join-Path $control 'four_camera_launch.stderr.log'
$launchProcess = $null
$serviceCallSequence = 0
$exitCode = 1
$runError = $null
$started = [System.Collections.Generic.List[string]]::new()
$prepared = [System.Collections.Generic.List[string]]::new()
$timingStopConfirmed = $false
$consoleAvailable = $true
$oldControlC = $null
try { $oldControlC = [Console]::TreatControlCAsInput } catch { $consoleAvailable = $false }

function Get-Sha256Hex {
  param([string]$Path)
  $stream = [IO.File]::OpenRead($Path)
  $algorithm = [Security.Cryptography.SHA256]::Create()
  try { return [BitConverter]::ToString($algorithm.ComputeHash($stream)).Replace('-', '') }
  finally { $algorithm.Dispose(); $stream.Dispose() }
}

function Invoke-RosService {
  param([string]$Service, [string]$Type, [string]$Request, [string]$Expected,
        [int]$MaxWaitSeconds = 75)
  Write-Host "Calling $Service ..."
  $script:serviceCallSequence++
  $safeName = $Service.Trim('/').Replace('/', '_')
  $stdoutPath = Join-Path $control ("service_{0:D2}_{1}.stdout.log" -f $script:serviceCallSequence, $safeName)
  $stderrPath = Join-Path $control ("service_{0:D2}_{1}.stderr.log" -f $script:serviceCallSequence, $safeName)
  $quotedRequest = '"' + $Request.Replace('"', '\"') + '"'
  $process = Start-Process -FilePath $ros2 -ArgumentList @(
      'service', 'call', $Service, $Type, $quotedRequest) -WindowStyle Hidden `
      -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath -PassThru
  $handle = $process.Handle
  $startedAt = [DateTime]::UtcNow
  $nextMessage = $startedAt.AddSeconds(5)
  while (-not $process.HasExited) {
    $process.Refresh()
    if (([DateTime]::UtcNow - $startedAt).TotalSeconds -ge $MaxWaitSeconds) {
      Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
      throw "Timed out after $MaxWaitSeconds seconds waiting for $Service"
    }
    if ($launchProcess -and $launchProcess.HasExited) {
      Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
      throw "ROS launch exited while waiting for $Service"
    }
    while ($consoleAvailable -and [Console]::KeyAvailable) {
      [void][Console]::ReadKey($true)
      Write-Host 'Input ignored while a safety-critical service call is in progress.' -ForegroundColor Yellow
    }
    if ([DateTime]::UtcNow -ge $nextMessage) {
      $elapsed = [Math]::Floor(([DateTime]::UtcNow - $startedAt).TotalSeconds)
      Write-Host "Still waiting for $Service ($elapsed s elapsed)..."
      $nextMessage = [DateTime]::UtcNow.AddSeconds(5)
    }
    Start-Sleep -Milliseconds 200
  }
  [void]$process.WaitForExit(); $process.Refresh()
  $out = if (Test-Path $stdoutPath) { Get-Content $stdoutPath -Raw } else { '' }
  $err = if (Test-Path $stderrPath) { Get-Content $stderrPath -Raw } else { '' }
  $text = ($out + [Environment]::NewLine + $err).Trim()
  if ($text) { Write-Host $text }
  if ($process.ExitCode -ne 0 -or $text -notmatch $Expected) {
    throw "ROS service failed or refused: $Service (exit=$($process.ExitCode))"
  }
}

function Wait-RosServices {
  param([string[]]$Names, [int]$TimeoutSeconds = 60)
  $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  do {
    $saved = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    try { $listed = @(& $ros2 service list 2>$null); $code = $LASTEXITCODE }
    finally { $ErrorActionPreference = $saved }
    if ($code -eq 0) {
      $missing = @($Names | Where-Object { $_ -notin $listed })
      if ($missing.Count -eq 0) { return }
    }
    if ($launchProcess -and $launchProcess.HasExited) {
      throw "ROS launch exited before services were ready (exit=$($launchProcess.ExitCode))."
    }
    Start-Sleep -Milliseconds 500
  } while ([DateTime]::UtcNow -lt $deadline)
  throw "Timed out waiting for ROS services: $($missing -join ', ')"
}

function Wait-HsiDarkComplete {
  param([double]$ConfiguredSeconds)
  $deadline = [DateTime]::UtcNow.AddSeconds([Math]::Max(45, [Math]::Ceiling($ConfiguredSeconds) + 30))
  $complete = @{}
  foreach ($camera in @('fx10e', 'swir')) { $complete[$camera] = $false }
  do {
    foreach ($camera in @('fx10e', 'swir')) {
      if ($complete[$camera]) { continue }
      $eventPath = Join-Path $segments "${camera}_events.ndjson"
      if (Test-Path -LiteralPath $eventPath -PathType Leaf) {
        $tail = @(Get-Content -LiteralPath $eventPath -Tail 20 -ErrorAction SilentlyContinue)
        if ($tail -match 'dark_complete') { $complete[$camera] = $true; Write-Host "$camera dark reference complete." }
      }
    }
    if ($complete['fx10e'] -and $complete['swir']) { return }
    if ($launchProcess -and $launchProcess.HasExited) { throw 'ROS launch exited during HSI dark acquisition.' }
    Start-Sleep -Milliseconds 250
  } while ([DateTime]::UtcNow -lt $deadline)
  $missing = @(@('fx10e', 'swir') | Where-Object { -not $complete[$_] })
  throw "Timed out waiting for HSI dark completion evidence: $($missing -join ', ')"
}

function Prepare-And-Arm {
  param([string]$Device)
  $portable = $session.Replace('\', '/')
  $request = "{request_id: 'prepare-$Device-$stamp', session_id: '$sessionId', session_directory: '$portable'}"
  Invoke-RosService "/$Device/prepare" 'ppbng_interfaces/srv/PrepareDevice' $request `
    'accepted\s*(?:=|:)\s*true'
  Invoke-RosService "/$Device/arm" 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'
}

function Safe-Service {
  param([string]$Service)
  try { Invoke-RosService $Service 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true' }
  catch { Write-Warning "Safe operation was not confirmed for $Service`: $($_.Exception.Message)"; $script:exitCode = 1 }
}

function Stop-TriggerBeforeCameras {
  $script:timingStopConfirmed = $false
  try {
    Invoke-RosService '/timing/disarm_keep_config' 'std_srvs/srv/Trigger' '{}' `
      'success\s*(?:=|:)\s*true' 8
    $script:timingStopConfirmed = $true
  } catch {
    Write-Warning "UNO STOPPED response was not confirmed: $($_.Exception.Message)"
    Write-Warning 'Terminating only the UNO ROS node, then waiting beyond its 3-second firmware watchdog.'
    $script:exitCode = 1
    $unoProcesses = @(Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
      Where-Object { $_.Name -eq 'uno_r4_ascii_trigger_node.exe' })
    foreach ($process in $unoProcesses) {
      Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Seconds 4
  }
}

$requiredArtifacts = @(
  $ros2, $config,
  (Join-Path $workspace 'install\ppbng_hsi\lib\ppbng_hsi\hsi_production_node.exe'),
  (Join-Path $workspace 'install\ppbng_rgb\lib\ppbng_rgb\ppbng_rgb_camera_node.exe'),
  (Join-Path $workspace 'install\ppbng_thermal\lib\ppbng_thermal\ppbng_thermal_camera_node.exe'),
  (Join-Path $workspace 'install\ppbng_timing\lib\ppbng_timing\uno_r4_ascii_trigger_node.exe')
)
foreach ($required in $requiredArtifacts) {
  if (-not (Test-Path -LiteralPath $required -PathType Leaf)) { throw "Required file missing: $required" }
}
$configText = Get-Content -LiteralPath $config -Raw
$timingPortMatch = [regex]::Match($configText, '(?ms)^timing:\s*.*?^\s{2}port:\s*["'']?([^"''\r\n#]+)')
if (-not $timingPortMatch.Success -or
    $timingPortMatch.Groups[1].Value.Trim() -match 'TO_BE_CONFIRMED|^$') {
  $ports = @(Get-CimInstance Win32_SerialPort -ErrorAction SilentlyContinue |
    ForEach-Object { "$($_.DeviceID) ($($_.Name))" })
  $portHint = if ($ports.Count) { $ports -join '; ' } else { 'no serial ports reported by Windows' }
  throw "Set timing.port in ppbng_config.yaml to the UNO R4 COM port. Detected: $portHint"
}
$darkMatch = [regex]::Match($configText, '(?m)^\s{2}hsi_dark_duration_seconds:\s*([0-9]+(?:\.[0-9]+)?)')
if (-not $darkMatch.Success) { throw 'session.hsi_dark_duration_seconds is missing or invalid.' }
$darkDurationSeconds = [double]::Parse($darkMatch.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
if (-not $consoleAvailable) { throw 'This workflow requires an interactive console for HSI dark-reference prompts.' }
$existingNames = @('hsi_production_node.exe', 'ppbng_rgb_camera_node.exe',
  'ppbng_thermal_camera_node.exe', 'uno_r4_ascii_trigger_node.exe')
$existing = @(Get-CimInstance Win32_Process | Where-Object { $_.Name -in $existingNames })
if ($existing.Count -ne 0) { throw 'A camera/timing production node is already running. Stop it first.' }
if (Test-Path -LiteralPath $session) { throw "Refusing to reuse dataset directory: $session" }

New-Item -ItemType Directory -Path $segments | Out-Null
New-Item -ItemType Directory -Path $control | Out-Null
Copy-Item -LiteralPath $config -Destination (Join-Path $control 'ppbng_config.yaml')
$identity = [System.Collections.Generic.List[string]]::new()
$identity.Add("session_id=$sessionId")
$identity.Add("start_utc=$([DateTime]::UtcNow.ToString('o'))")
foreach ($artifact in @(
  $config,
  (Join-Path $workspace 'install\ppbng_hsi\lib\ppbng_hsi\hsi_production_node.exe'),
  (Join-Path $workspace 'install\ppbng_rgb\lib\ppbng_rgb\ppbng_rgb_camera_node.exe'),
  (Join-Path $workspace 'install\ppbng_thermal\lib\ppbng_thermal\ppbng_thermal_camera_node.exe'),
  (Join-Path $workspace 'install\ppbng_timing\lib\ppbng_timing\uno_r4_ascii_trigger_node.exe')
)) {
  if (Test-Path $artifact -PathType Leaf) { $identity.Add("sha256=$(Get-Sha256Hex $artifact) path=$artifact") }
}
[IO.File]::WriteAllLines((Join-Path $control 'build_identity.txt'), $identity)

try {
  [Console]::TreatControlCAsInput = $true
  $launchProcess = Start-Process -FilePath $ros2 -ArgumentList @(
      'launch', 'ppbng_bringup', 'four_camera.launch.py', 'hardware_enabled:=true',
      "config_file:=$config") -WindowStyle Hidden -RedirectStandardOutput $launchStdout `
      -RedirectStandardError $launchStderr -PassThru
  Start-Sleep -Seconds 10
  $devices = @('fx10e', 'swir', 'rgb', 'thermal', 'timing')
  $services = @($devices | ForEach-Object {
    $name = $_; @('prepare', 'arm', 'start', 'stop') | ForEach-Object { "/$name/$_" }
  }) + @('/fx10e/begin_dark', '/swir/begin_dark', '/fx10e/start_sample',
    '/swir/start_sample', '/timing/arm_immediate', '/timing/disarm_keep_config',
    '/rgb/status', '/thermal/status', '/timing/status')
  Wait-RosServices $services

  foreach ($device in $devices) { Prepare-And-Arm $device; $prepared.Add($device) }

  # Open/identify the trigger controller first, but force both pins LOW.
  $started.Add('timing')
  Invoke-RosService '/timing/start' 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'
  # Vendor SDK initialization is deliberately sequential; FX10e and SWIR stay
  # in separate processes after initialization.
  foreach ($camera in @('fx10e', 'swir')) {
    $started.Add($camera)
    Invoke-RosService "/$camera/start" 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'
  }

  Write-Host "`n双 HSI 已初始化，内部 shutter 保持关闭。请完全盖住两台 HSI 镜头，然后按 ENTER 采集暗场。" -ForegroundColor Cyan
  [void](Read-Host)
  foreach ($camera in @('fx10e', 'swir')) {
    Invoke-RosService "/$camera/begin_dark" 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'
  }
  Write-Host '正在采集配置文件规定的暗场；等待两台相机分别写出 dark_complete 证据...'
  Wait-HsiDarkComplete $darkDurationSeconds
  Write-Host "`n暗场已完成。请取下镜头盖、确认四台相机视野和机器人均已准备好，然后按 ENTER。" -ForegroundColor Cyan
  [void](Read-Host)

  # Snapshot cameras enter acquisition before any electrical trigger exists.
  foreach ($camera in @('rgb', 'thermal')) {
    $started.Add($camera)
    Invoke-RosService "/$camera/start" 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'
  }
  # HSI line streams start continuously. The shared snapshot trigger is armed
  # last, so no RGB/thermal frame can precede camera readiness.
  foreach ($camera in @('fx10e', 'swir')) {
    Invoke-RosService "/$camera/start_sample" 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'
  }
  Invoke-RosService '/timing/arm_immediate' 'std_srvs/srv/Trigger' '{}' 'success\s*(?:=|:)\s*true'

  Write-Host "`n四相机采集已开始；RGB 与 thermal 共用 D11/D12 原子触发边沿。" -ForegroundColor Green
  Write-Host "Dataset: $session"
  Write-Host '按 ENTER 或 Ctrl+C 可提前结束；程序会先停触发，再安全关闭四台相机。'
  $deadline = [DateTime]::UtcNow.AddMinutes($DurationMinutes)
  $nextReport = [DateTime]::UtcNow
  $nextHealthCheck = [DateTime]::UtcNow.AddSeconds(30)
  while ([DateTime]::UtcNow -lt $deadline) {
    $launchProcess.Refresh()
    if ($launchProcess.HasExited) { throw "ROS launch exited unexpectedly (exit=$($launchProcess.ExitCode))." }
    $processCounts = @{
      hsi = @(Get-Process -Name 'hsi_production_node' -ErrorAction SilentlyContinue).Count
      rgb = @(Get-Process -Name 'ppbng_rgb_camera_node' -ErrorAction SilentlyContinue).Count
      thermal = @(Get-Process -Name 'ppbng_thermal_camera_node' -ErrorAction SilentlyContinue).Count
      timing = @(Get-Process -Name 'uno_r4_ascii_trigger_node' -ErrorAction SilentlyContinue).Count
    }
    if ($processCounts.hsi -lt 2 -or $processCounts.rgb -lt 1 -or
        $processCounts.thermal -lt 1 -or $processCounts.timing -lt 1) {
      throw "A required child process exited: hsi=$($processCounts.hsi), rgb=$($processCounts.rgb), thermal=$($processCounts.thermal), timing=$($processCounts.timing)"
    }
    if ([Console]::KeyAvailable) {
      $key = [Console]::ReadKey($true)
      if ($key.Key -eq [ConsoleKey]::Enter -or
          ($key.Key -eq [ConsoleKey]::C -and ($key.Modifiers -band [ConsoleModifiers]::Control))) {
        Write-Host 'Operator requested an early stop.'; break
      }
    }
    foreach ($camera in @('fx10e', 'swir')) {
      $events = Join-Path $segments "${camera}_events.ndjson"
      if (Test-Path $events) {
        $last = Get-Content $events -Tail 1 -ErrorAction SilentlyContinue
        if ($last -match 'SAMPLE FAIL-FAST') { throw "$camera reported a sample continuity fault: $events" }
      }
    }
    if ([DateTime]::UtcNow -ge $nextHealthCheck) {
      foreach ($device in @('rgb', 'thermal', 'timing')) {
        Invoke-RosService "/$device/status" 'std_srvs/srv/Trigger' '{}' `
          'success\s*(?:=|:)\s*true' 15
      }
      $nextHealthCheck = [DateTime]::UtcNow.AddSeconds(60)
    }
    if ([DateTime]::UtcNow -ge $nextReport) {
      $remaining = [Math]::Max(0, [Math]::Ceiling(($deadline - [DateTime]::UtcNow).TotalMinutes))
      Write-Host "采集运行正常；约剩余 $remaining 分钟。"
      $nextReport = [DateTime]::UtcNow.AddMinutes(1)
    }
    Start-Sleep -Milliseconds 200
  }
  $exitCode = 0
} catch {
  $runError = $_.Exception.Message
  Write-Host "FOUR-CAMERA TEST FAILED: $runError" -ForegroundColor Red
  try { [IO.File]::WriteAllText((Join-Path $control 'script_error.txt'), $runError) } catch {}
} finally {
  Write-Host "`nStopping trigger first, then closing all cameras safely..."
  if ($prepared.Contains('timing')) { Stop-TriggerBeforeCameras }
  foreach ($camera in @('rgb', 'thermal', 'fx10e', 'swir')) {
    if ($prepared.Contains($camera)) { Safe-Service "/$camera/stop" }
  }
  if ($prepared.Contains('timing') -and $timingStopConfirmed) { Safe-Service '/timing/stop' }
  if ($launchProcess -and -not $launchProcess.HasExited) {
    $saved = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    try { & taskkill.exe /PID $launchProcess.Id /T /F 2>$null | Out-Null }
    finally { $ErrorActionPreference = $saved }
    [void]$launchProcess.WaitForExit(10000)
  }
  if ($consoleAvailable) { [Console]::TreatControlCAsInput = $oldControlC }
  Write-Host "Saved dataset: $session"
  $indices = @(Get-ChildItem $segments -Filter '*.index.csv' -File -ErrorAction SilentlyContinue)
  if ($indices.Count -gt 0) {
    foreach ($camera in @('fx10e', 'swir')) {
      Write-Host "Read-only quick HSI verification: $camera"
      $saved = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
      try { & (Join-Path $PSScriptRoot 'verify_dataset.cmd') $session '--hsi-only' $camera '--quick'; $verify = $LASTEXITCODE }
      finally { $ErrorActionPreference = $saved }
      if ($verify -ne 0) { Write-Warning "$camera verification failed; preserve dataset and logs."; $exitCode = 1 }
    }
  } else { Write-Warning 'No HSI index files found; automatic verification skipped.' }
  foreach ($stream in @('rgb', 'thermal')) {
    $files = @(Get-ChildItem $segments -Filter "${stream}_*.ppbseg" -File -ErrorAction SilentlyContinue)
    $bytes = ($files | Measure-Object Length -Sum).Sum
    if ($files.Count -eq 0 -or $bytes -le 0) {
      Write-Warning "No non-empty $stream .ppbseg data was found."; $exitCode = 1
    } else { Write-Host "$stream snapshot evidence: $($files.Count) file(s), $bytes bytes." }
  }
  $timingEvidence = Join-Path $segments 'uno_r4_timing_events.bin'
  if (-not (Test-Path $timingEvidence -PathType Leaf) -or (Get-Item $timingEvidence).Length -le 0) {
    Write-Warning 'UNO timing evidence is missing or empty.'; $exitCode = 1
  }
}

exit $exitCode
