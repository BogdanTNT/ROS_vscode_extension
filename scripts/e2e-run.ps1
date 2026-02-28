param(
    [ValidateSet('smoke', 'full')]
    [string]$Profile = 'smoke',
    [string]$MatrixFile = '',
    [string]$RowId = '',
    [string]$ResultsRoot = 'e2e/results',
    [switch]$SkipUiClick,
    [switch]$EnableVm,
    [switch]$KeepEnvironment
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$runner = Join-Path $repoRoot 'e2e\runner\run-matrix.mjs'

if (-not (Test-Path $runner)) {
    throw "Matrix runner not found: $runner"
}

$args = @($runner, '--profile', $Profile, '--results-root', $ResultsRoot)
if (-not [string]::IsNullOrWhiteSpace($MatrixFile)) {
    $args += @('--matrix', $MatrixFile)
}
if (-not [string]::IsNullOrWhiteSpace($RowId)) {
    $args += @('--row', $RowId)
}
if ($SkipUiClick) {
    $args += @('--skip-ui-click', 'true')
}
if ($EnableVm) {
    $args += @('--enable-vm', 'true')
}
if ($KeepEnvironment) {
    $args += @('--keep-environment', 'true')
}

Write-Host "Running E2E profile '$Profile'..."
node @args
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
