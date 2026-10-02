# @file install.ps1
# @brief 安裝 Windows 原生工具至使用者目錄並登錄使用者 PATH
# @author Taiphoon Agent Team
# @date 2026-10-02
$ErrorActionPreference = 'Stop'
$installDirectory = Join-Path $env:LOCALAPPDATA 'TaiphoonAgent\bin'
if (-not (Test-Path -LiteralPath $installDirectory))
{
    New-Item -ItemType Directory -Path $installDirectory -ErrorAction Stop | Out-Null
}
foreach ($fileName in @('taiphoon_agent.exe', 'release.manifest', 'sitl-entrypoint.sh', 'README.md'))
{
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $fileName) -Destination $installDirectory
}
$imageDirectory = Join-Path $installDirectory 'images'
if (-not (Test-Path -LiteralPath $imageDirectory))
{
    New-Item -ItemType Directory -Path $imageDirectory -ErrorAction Stop | Out-Null
}
Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'images') -Filter '*.tar.gz' | Copy-Item -Destination $imageDirectory
$userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
if (($userPath -split ';') -notcontains $installDirectory)
{
    [Environment]::SetEnvironmentVariable('Path', "$installDirectory;$userPath", 'User')
}
if (($env:Path -split ';') -notcontains $installDirectory)
{
    $env:Path = "$installDirectory;$env:Path"
}
Write-Host '已安裝。執行：taiphoon_agent start'
