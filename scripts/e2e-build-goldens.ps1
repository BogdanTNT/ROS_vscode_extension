param(
    [ValidateSet('all', 'wsl', 'vm')]
    [string]$Mode = 'all',
    [string]$DateTag = '',
    [switch]$SkipProvision
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($DateTag)) {
    $DateTag = Get-Date -Format 'yyyyMMdd'
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$goldenRoot = Join-Path $repoRoot 'goldens'
$wslRoot = Join-Path $goldenRoot 'wsl'
$vmRoot = Join-Path $goldenRoot 'vm'
$null = New-Item -Path $wslRoot -ItemType Directory -Force
$null = New-Item -Path $vmRoot -ItemType Directory -Force

$wslGoldens = @(
    @{ Source = 'Ubuntu-20.04'; RosDistro = 'noetic'; RosVersion = '1'; FilePrefix = 'ubuntu2004-noetic' },
    @{ Source = 'Ubuntu-22.04'; RosDistro = 'humble'; RosVersion = '2'; FilePrefix = 'ubuntu2204-humble' },
    @{ Source = 'Ubuntu-24.04'; RosDistro = 'jazzy'; RosVersion = '2'; FilePrefix = 'ubuntu2404-jazzy' }
)

function Convert-ToWslPath([string]$PathValue) {
    $normalized = $PathValue.Replace('\', '/')
    if ($normalized -match '^([A-Za-z]):/(.*)$') {
        return "/mnt/$($matches[1].ToLower())/$($matches[2])"
    }
    return $normalized
}

if ($Mode -eq 'all' -or $Mode -eq 'wsl') {
    foreach ($golden in $wslGoldens) {
        $output = Join-Path $wslRoot "$($golden.FilePrefix)-v$DateTag.tar"
        $latest = Join-Path $wslRoot "$($golden.FilePrefix)-latest.tar"
        Write-Host "Building WSL golden: $($golden.Source) -> $output"

        if (-not $SkipProvision) {
            $provisionScript = Join-Path $repoRoot 'e2e\provision\linux\provision-ros.sh'
            if (-not (Test-Path $provisionScript)) {
                throw "Provision script missing: $provisionScript"
            }
            $provisionWslPath = Convert-ToWslPath $provisionScript
            $provisionCmd = "bash $provisionWslPath --ros-distro $($golden.RosDistro) --ros-version $($golden.RosVersion)"
            wsl.exe -d $golden.Source --user root --exec bash -lc $provisionCmd
            if ($LASTEXITCODE -ne 0) {
                throw "Failed to provision $($golden.Source)"
            }
        }

        wsl.exe --shutdown | Out-Null
        wsl.exe --export $golden.Source $output
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to export WSL distro $($golden.Source)"
        }
        Copy-Item -Path $output -Destination $latest -Force
    }
}

if ($Mode -eq 'all' -or $Mode -eq 'vm') {
    $packerTemplate = Join-Path $repoRoot 'e2e\packer\hyperv\ubuntu.pkr.hcl'
    if (-not (Get-Command packer -ErrorAction SilentlyContinue)) {
        throw "Packer CLI not found in PATH. Install Packer first."
    }
    if (-not (Test-Path $packerTemplate)) {
        throw "Packer template missing: $packerTemplate"
    }

    $vmVariants = @(
        @{ VarFile = 'ubuntu2004-noetic.pkrvars.hcl'; FilePrefix = 'ubuntu2004-noetic' },
        @{ VarFile = 'ubuntu2204-humble.pkrvars.hcl'; FilePrefix = 'ubuntu2204-humble' },
        @{ VarFile = 'ubuntu2404-jazzy.pkrvars.hcl'; FilePrefix = 'ubuntu2404-jazzy' }
    )

    foreach ($variant in $vmVariants) {
        $varPath = Join-Path $repoRoot "e2e\packer\hyperv\$($variant.VarFile)"
        if (-not (Test-Path $varPath)) {
            throw "Packer var file missing: $varPath"
        }

        $outputDir = Join-Path $vmRoot "$($variant.FilePrefix)-v$DateTag"
        $null = New-Item -Path $outputDir -ItemType Directory -Force

        packer build `
            -var-file=$varPath `
            -var "output_directory=$outputDir" `
            $packerTemplate
        if ($LASTEXITCODE -ne 0) {
            throw "Packer build failed for $($variant.FilePrefix)"
        }

        $builtVhd = Get-ChildItem -Path $outputDir -Filter *.vhdx -Recurse | Select-Object -First 1
        if (-not $builtVhd) {
            throw "No VHDX output found for $($variant.FilePrefix) in $outputDir"
        }

        $latestVhd = Join-Path $vmRoot "$($variant.FilePrefix)-latest.vhdx"
        $versionedVhd = Join-Path $vmRoot "$($variant.FilePrefix)-v$DateTag.vhdx"
        Copy-Item -Path $builtVhd.FullName -Destination $versionedVhd -Force
        Copy-Item -Path $versionedVhd -Destination $latestVhd -Force
    }
}

Write-Host "Golden build complete. Mode=$Mode DateTag=$DateTag"
