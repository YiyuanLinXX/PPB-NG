# Pure parsing/display helpers. Loading this file never starts ROS or hardware.
function Set-RunGnssDisabled {
  param([string]$ConfigText)
  # Restrict the override to the top-level gnss section; leave every peer unchanged.
  $pattern = '(?m)(^gnss:\s*\r?\n(?:(?:[ \t]+[^\r\n]*|)[\r]?\n)*?  enabled:)[ \t]*(?:true|false)([ \t]*(?:#[^\r\n]*)?)(?=\r?$)'
  if ([regex]::Matches($ConfigText, $pattern).Count -ne 1) {
    throw 'Expected exactly one gnss.enabled boolean in the configuration.'
  }
  [regex]::Replace($ConfigText, $pattern, '${1} false${2}')
}

function Set-RunPlannedDuration {
  param([string]$ConfigText, [int]$Minutes)
  if ($Minutes -lt 1 -or $Minutes -gt 720) { throw 'Duration must be 1-720 minutes.' }
  $pattern = '(?m)^  planned_duration_seconds:[^\r\n]*'
  if ([regex]::Matches($ConfigText, $pattern).Count -ne 1) {
    throw 'Expected exactly one session planned_duration_seconds field in the configuration.'
  }
  [regex]::Replace($ConfigText, $pattern, ('  planned_duration_seconds: {0}' -f ($Minutes * 60)))
}

function ConvertFrom-OperatorStatus {
  param([string]$Text)
  $stateMatch = [regex]::Match($Text, '(?:^|[;=])state=([0-9]+)(?:;|$)')
  $countMatch = [regex]::Match($Text, ';operator_warning_count=([0-9]+);')
  $hexMatch = [regex]::Match($Text, ';operator_warnings_hex=([0-9a-fA-F]*);')
  if (-not $stateMatch.Success -or -not $countMatch.Success -or -not $hexMatch.Success) {
    throw 'Operator health status is missing/malformed. Rebuild ppbng_runtime; do not assume all sensors are healthy.'
  }
  $hex = $hexMatch.Groups[1].Value
  if ($hex.Length % 2 -ne 0) { throw 'Malformed operator warning payload.' }
  $bytes = [byte[]]::new($hex.Length / 2)
  for ($index = 0; $index -lt $bytes.Length; $index++) {
    $bytes[$index] = [Convert]::ToByte($hex.Substring($index * 2, 2), 16)
  }
  $warnings = [Text.UTF8Encoding]::new($false, $true).GetString($bytes)
  $count = [int]$countMatch.Groups[1].Value
  if (($count -gt 0) -ne ($warnings.Length -gt 0)) { throw 'Inconsistent operator warning count/payload.' }
  [pscustomobject]@{ State = [int]$stateMatch.Groups[1].Value; WarningCount = $count; Warnings = $warnings }
}

function Write-OperatorAlert {
  param([string]$Details)
  Write-Host ''
  Write-Host ('!!!!!!!!!!!!!!!! DATA ACQUISITION ALERT {0} !!!!!!!!!!!!!!!!' -f (Get-Date -Format 'HH:mm:ss')) -ForegroundColor White -BackgroundColor DarkRed
  Write-Host $Details -ForegroundColor Red
  Write-Host 'Data may be incomplete. Other streams may still be recording. Inspect the affected device.' -ForegroundColor Red
  Write-Host 'Press ENTER or Ctrl+C once for an ordered stop. This alert does not command robot motion.' -ForegroundColor Red
  Write-Host '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!' -ForegroundColor White -BackgroundColor DarkRed
}
