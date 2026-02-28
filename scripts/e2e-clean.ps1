param(
    [string]$WslPrefix = 'ros-e2e-',
    [string]$VmPrefix = 'ros-e2e-',
    [string]$WslInstallRoot = 'C:\ROS_E2E\wsl',
    [string]$VmTempRoot = '',
    [int]$DeleteResultsOlderThanDays = 14
)

$ErrorActionPreference = 'Continue'

$repoRoot = Split-Path -Parent $PSScriptRoot
$resultsRoot = Join-Path $repoRoot 'e2e\results'

Write-Host "Cleaning WSL distros with prefix '$WslPrefix'..."
$rawWsl = wsl.exe -l -v 2>$null
foreach ($line in ($rawWsl -split "`n")) {
    $clean = ($line -replace "`0", '').Trim()
    if ([string]::IsNullOrWhiteSpace($clean)) { continue }
    if ($clean -match '^NAME\s+STATE\s+VERSION$') { continue }
    $clean = $clean.TrimStart('*').Trim()
    $parts = $clean -split '\s{2,}'
    if ($parts.Count -lt 1) { continue }
    $name = $parts[0].Trim()
    if (-not $name.StartsWith($WslPrefix)) { continue }

    Write-Host " - unregistering $name"
    wsl.exe --terminate $name | Out-Null
    wsl.exe --unregister $name | Out-Null
}

if (Test-Path $WslInstallRoot) {
    Write-Host "Cleaning WSL install root: $WslInstallRoot"
    Get-ChildItem -Path $WslInstallRoot -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name.StartsWith($WslPrefix) } |
        Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
}

if ([string]::IsNullOrWhiteSpace($VmTempRoot)) {
    $VmTempRoot = Join-Path $env:TEMP 'ros-dev-toolkit-e2e-vm'
}

Write-Host "Cleaning Hyper-V VMs with prefix '$VmPrefix'..."
if (Get-Command Get-VM -ErrorAction SilentlyContinue) {
    Get-VM -ErrorAction SilentlyContinue |
        Where-Object { $_.Name.StartsWith($VmPrefix) } |
        ForEach-Object {
            Write-Host " - removing VM $($_.Name)"
            Stop-VM -Name $_.Name -Force -TurnOff -ErrorAction SilentlyContinue | Out-Null
            Remove-VM -Name $_.Name -Force -ErrorAction SilentlyContinue | Out-Null
        }
}

if (Test-Path $VmTempRoot) {
    Write-Host "Cleaning VM temp root: $VmTempRoot"
    Remove-Item -Path $VmTempRoot -Recurse -Force -ErrorAction SilentlyContinue
}

if (Test-Path $resultsRoot -and $DeleteResultsOlderThanDays -gt 0) {
    Write-Host "Deleting E2E result folders older than $DeleteResultsOlderThanDays days..."
    $cutoff = (Get-Date).AddDays(-$DeleteResultsOlderThanDays)
    Get-ChildItem -Path $resultsRoot -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.LastWriteTime -lt $cutoff -and $_.Name -ne '.gitkeep' } |
        Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host "E2E cleanup finished."
