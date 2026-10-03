#Requires -RunAsAdministrator

[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$plans = @(
    [pscustomobject]@{
        Role       = 'FX10e'
        Alias      = 'Ethernet'
        ExpectedMac = '20-46-A1-0D-A7-DA'
        Address    = '192.168.10.1'
        Prefix     = 24
        AllowedOld = @('169.254.0.1')
    },
    [pscustomobject]@{
        Role       = 'A6701 thermal'
        Alias      = 'Ethernet 3'
        ExpectedMac = '20-46-A1-0D-A7-D9'
        Address    = '192.168.20.1'
        Prefix     = 24
        AllowedOld = @('169.254.10.1', '192.168.4.150')
    }
)

$cameraProcesses = Get-Process -ErrorAction SilentlyContinue |
    Where-Object { $_.ProcessName -match 'HyperFusion|Lumo|eBUS|GEV|Pv' }
if ($cameraProcesses) {
    $names = ($cameraProcesses.ProcessName | Sort-Object -Unique) -join ', '
    throw "Close camera software before changing NIC addresses. Running: $names"
}

foreach ($plan in $plans) {
    $adapter = Get-NetAdapter -Name $plan.Alias
    if ($adapter.MacAddress -ne $plan.ExpectedMac) {
        throw "$($plan.Role) adapter MAC mismatch on '$($plan.Alias)': expected $($plan.ExpectedMac), found $($adapter.MacAddress)"
    }

    $addresses = @(Get-NetIPAddress -InterfaceAlias $plan.Alias -AddressFamily IPv4 |
        Where-Object { $_.PrefixOrigin -eq 'Manual' })
    $allowed = @($plan.AllowedOld) + @($plan.Address)
    $unexpected = @($addresses | Where-Object { $_.IPAddress -notin $allowed })
    if ($unexpected) {
        $values = ($unexpected.IPAddress | Sort-Object -Unique) -join ', '
        throw "Refusing to alter '$($plan.Alias)': unexpected manual IPv4 address(es): $values"
    }
}

foreach ($plan in $plans) {
    Set-NetIPInterface -InterfaceAlias $plan.Alias -AddressFamily IPv4 -Dhcp Disabled

    foreach ($oldAddress in $plan.AllowedOld) {
        $existing = Get-NetIPAddress -InterfaceAlias $plan.Alias -AddressFamily IPv4 `
            -IPAddress $oldAddress -ErrorAction SilentlyContinue
        if ($existing) {
            Remove-NetIPAddress -InterfaceAlias $plan.Alias -IPAddress $oldAddress -Confirm:$false
        }
    }

    $desired = Get-NetIPAddress -InterfaceAlias $plan.Alias -AddressFamily IPv4 `
        -IPAddress $plan.Address -ErrorAction SilentlyContinue
    if (-not $desired) {
        New-NetIPAddress -InterfaceAlias $plan.Alias -IPAddress $plan.Address `
            -PrefixLength $plan.Prefix | Out-Null
    } elseif ($desired.PrefixLength -ne $plan.Prefix) {
        throw "$($plan.Role) address exists with prefix /$($desired.PrefixLength), expected /$($plan.Prefix)"
    }
}

$defaultRoutes = @(Get-NetRoute -AddressFamily IPv4 -DestinationPrefix '0.0.0.0/0' |
    Where-Object { $_.InterfaceAlias -in $plans.Alias })
if ($defaultRoutes) {
    throw 'A sensor-only NIC unexpectedly has a default route. No default route was created by this script.'
}

Write-Host 'PPB-NG sensor NIC plan applied successfully.' -ForegroundColor Green
Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.InterfaceAlias -in $plans.Alias } |
    Sort-Object InterfaceIndex, IPAddress |
    Format-Table InterfaceIndex, InterfaceAlias, IPAddress, PrefixLength, PrefixOrigin, AddressState -AutoSize

Write-Host ''
Write-Host 'Expected final host assignments:'
Write-Host '  FX10e:        Ethernet   192.168.10.1/24  MAC 20-46-A1-0D-A7-DA'
Write-Host '  A6701 thermal: Ethernet 3 192.168.20.1/24  MAC 20-46-A1-0D-A7-D9'
Write-Host 'The camera-side persistent addresses must still be configured separately.'
