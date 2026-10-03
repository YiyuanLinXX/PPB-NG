#Requires -RunAsAdministrator

[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$interfaceAlias = 'Ethernet'
$expectedMac = '20-46-A1-0D-A7-DA'
$expectedHostAddress = '192.168.10.1'
$pleoraComponent = 'PT_ebUniversalProForEthernet'
$componentsToDisable = @('pgr_lwf', 'TD_CorGigeFilter')

$adapter = Get-NetAdapter -Name $interfaceAlias
if ($adapter.MacAddress -ne $expectedMac) {
    throw "FX10e adapter MAC mismatch on '$interfaceAlias': expected $expectedMac, found $($adapter.MacAddress)"
}

$hostAddress = Get-NetIPAddress -InterfaceAlias $interfaceAlias -AddressFamily IPv4 `
    -IPAddress $expectedHostAddress -ErrorAction SilentlyContinue
if (-not $hostAddress -or $hostAddress.PrefixLength -ne 24) {
    throw "FX10e host address must be $expectedHostAddress/24 before changing filter bindings"
}

$cameraProcesses = Get-Process -ErrorAction SilentlyContinue |
    Where-Object { $_.ProcessName -match 'HyperFusion|Lumo|eBUS|GEV|Pv' }
if ($cameraProcesses) {
    $names = ($cameraProcesses.ProcessName | Sort-Object -Unique) -join ', '
    throw "Close camera software before changing filter bindings. Running: $names"
}

$pleora = Get-NetAdapterBinding -Name $interfaceAlias -ComponentID $pleoraComponent
if (-not $pleora.Enabled) {
    Enable-NetAdapterBinding -Name $interfaceAlias -ComponentID $pleoraComponent
}

foreach ($component in $componentsToDisable) {
    $binding = Get-NetAdapterBinding -Name $interfaceAlias -ComponentID $component
    if ($binding.Enabled) {
        Disable-NetAdapterBinding -Name $interfaceAlias -ComponentID $component
    }
}

Start-Sleep -Seconds 3

$finalPleora = Get-NetAdapterBinding -Name $interfaceAlias -ComponentID $pleoraComponent
$finalPointGrey = Get-NetAdapterBinding -Name $interfaceAlias -ComponentID 'pgr_lwf'
$finalDalsa = Get-NetAdapterBinding -Name $interfaceAlias -ComponentID 'TD_CorGigeFilter'
if (-not $finalPleora.Enabled -or $finalPointGrey.Enabled -or $finalDalsa.Enabled) {
    throw 'Final filter-binding verification failed'
}

$finalAddress = Get-NetIPAddress -InterfaceAlias $interfaceAlias -AddressFamily IPv4 `
    -IPAddress $expectedHostAddress -ErrorAction SilentlyContinue
if (-not $finalAddress -or $finalAddress.PrefixLength -ne 24) {
    throw 'FX10e IPv4 address changed unexpectedly while applying filter bindings'
}

Write-Host 'FX10e filter bindings configured successfully.' -ForegroundColor Green
Get-NetAdapterBinding -Name $interfaceAlias |
    Where-Object { $_.ComponentID -in @($pleoraComponent, 'pgr_lwf', 'TD_CorGigeFilter') } |
    Format-Table DisplayName, ComponentID, Enabled -AutoSize
Get-NetAdapter -Name $interfaceAlias |
    Format-Table Name, Status, MacAddress, LinkSpeed -AutoSize
Get-NetIPAddress -InterfaceAlias $interfaceAlias -AddressFamily IPv4 |
    Format-Table IPAddress, PrefixLength, AddressState -AutoSize
