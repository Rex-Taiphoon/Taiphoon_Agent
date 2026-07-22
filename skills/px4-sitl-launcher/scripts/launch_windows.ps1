<#
.SYNOPSIS
    Launch PX4 SITL simulators in visible terminal windows.
.DESCRIPTION
    Launches PX4 SITL instances with MicroXRCEAgent in separate windows.
    Each window shows live PX4 output and stays open until closed.
    Auto-detects WSL distro and converts paths to Linux format.
.PARAMETER Count
    Number of drones (default: 3).
.PARAMETER WslDistro
    WSL distro name (default: auto-detect).
.PARAMETER UseWindowsTerminal
    Use Windows Terminal tabs instead of separate windows.
#>
param(
    [int]$Count = 3,
    [string]$WslDistro = "",
    [switch]$UseWindowsTerminal = $false
)

function Get-WslDistro {
    $candidates = @("Ubuntu-22.04", "Ubuntu-20.04", "Ubuntu-24.04", "Ubuntu")
    foreach ($name in $candidates) {
        $null = wsl -d $name echo "ok" 2>&1
        if ($LASTEXITCODE -eq 0) {
            return $name
        }
    }
    return $null
}

function ConvertTo-LinuxPath {
    param([string]$WindowsPath)
    $result = $WindowsPath -replace '^\\\\wsl\.localhost\\[^\\]+\\', '/'
    $result = $result -replace '\\', '/'
    return $result
}

if (-not $WslDistro) {
    $detected = Get-WslDistro
    if (-not $detected) {
        Write-Error "No WSL Ubuntu found, specify -WslDistro"
        exit 1
    }
    $WslDistro = $detected
}

$WIN_SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Definition
$SCRIPT_DIR = ConvertTo-LinuxPath -WindowsPath $WIN_SCRIPT_DIR
$FOREGROUND_SCRIPT = "${SCRIPT_DIR}/start_single_instance_foreground.sh"

Write-Host "=== Starting ${Count} PX4 SITL instances ===" -ForegroundColor Green
Write-Host "WSL Distro: ${WslDistro}" -ForegroundColor Cyan
Write-Host "Script path: ${FOREGROUND_SCRIPT}" -ForegroundColor Cyan
Write-Host "Each window shows live PX4 output (Ctrl+C to stop)" -ForegroundColor Yellow

if ($UseWindowsTerminal) {
    for ($i = 0; $i -lt $Count; $i++) {
        $wslCmd = "wsl -d ${WslDistro} bash ${FOREGROUND_SCRIPT} ${i}"
        Start-Process wt -ArgumentList "new-tab", "--title", "PX4-${i}", $WslDistro, $wslCmd -NoNewWindow
        Write-Host "  Instance ${i}: Windows Terminal tab" -ForegroundColor Yellow
        Start-Sleep -Seconds 3
    }
} else {
    for ($i = 0; $i -lt $Count; $i++) {
        $cmd = "wsl -d ${WslDistro} bash ${FOREGROUND_SCRIPT} ${i}"
        Write-Host "  Running: ${cmd}" -ForegroundColor Gray
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = "wsl"
        $psi.Arguments = "-d ${WslDistro} bash ${FOREGROUND_SCRIPT} ${i}"
        $psi.UseShellExecute = $true
        $psi.CreateNoWindow = $false
        [System.Diagnostics.Process]::Start($psi) | Out-Null
        Write-Host "  Instance ${i}: separate window" -ForegroundColor Yellow
        Start-Sleep -Seconds 3
    }
}

Write-Host "=== All ${Count} instances launched ===" -ForegroundColor Green
Write-Host "Close each window with Ctrl+C to stop an instance" -ForegroundColor Green