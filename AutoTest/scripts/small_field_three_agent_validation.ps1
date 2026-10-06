<#
@file small_field_three_agent_validation.ps1
@brief Windows 原生 Docker 三機小場域驗證；不使用 WSL 或容器 Docker socket
@author Taiphoon Agent Team
@date 2026-10-06
#>
[CmdletBinding()]
param(
    [ValidateSet(1, 2, 3)]
    [int]$FailedAgent = 2,
    [ValidateSet('formation', 'crossing')]
    [string]$Scenario = 'formation',
    [string]$LogRoot
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
if (-not $LogRoot) {
    $LogRoot = Join-Path $projectRoot ('AutoTest/Log/small-field-' + (Get-Date -Format 'yyyyMMdd'))
}
$caseRoot = Join-Path $LogRoot ($Scenario + '-failed-' + $FailedAgent + '-' + (Get-Date -Format 'HHmmss'))
New-Item -ItemType Directory -Path $caseRoot | Out-Null
$dockerPath = (Get-Command docker.exe -ErrorAction Stop).Source
$composePath = (Get-Command docker-compose.exe -ErrorAction Stop).Source
$gitPath = (Get-Command git.exe -ErrorAction Stop).Source
$bashPath = Join-Path (Split-Path (Split-Path $gitPath -Parent) -Parent) 'bin/bash.exe'
if (-not (Test-Path -LiteralPath $bashPath)) {
    throw '找不到 Windows Git Bash；本腳本不會改用 WSL。'
}
$observerName = 'taiphoon-small-field-observer-' + $FailedAgent + '-' + (Get-Date -Format 'HHmmss')
$buildName = $observerName + '-build'
$overridePath = Join-Path $caseRoot 'compose.windows.yaml'
$lines = @('services:')
foreach ($agentId in 1..3) {
    $lines += @("  uav_${agentId}_agent:", '    image: taiphoon-agent:dev',
        '    volumes: !override', '      - type: bind',
        ('        source: ' + $projectRoot.Replace('\', '/')),
        '        target: /home/taiphoon/Taiphoon_Agent')
    foreach ($part in @('build', 'install', 'log')) {
        $lines += @('      - type: volume', "        source: validation_${part}_clean",
            "        target: /home/taiphoon/Taiphoon_Agent/${part}",
            '        volume:', '          nocopy: true')
    }
    $lines += @('    environment:', '      CMAKE_BUILD_PARALLEL_LEVEL: 2', '      MAKEFLAGS: -j2')
}
$lines += @('volumes:', '  validation_build_clean:', '  validation_install_clean:', '  validation_log_clean:')
[System.IO.File]::WriteAllText($overridePath, ($lines -join "`n") + "`n")
$baseCompose = Join-Path $projectRoot 'docker-compose.yml'
$composeArgs = @('-f', $baseCompose, '-f', $overridePath)
$oldComposeFile = $env:COMPOSE_FILE
$oldSshSock = $env:SSH_AUTH_SOCK
$oldMsysPathConv = $env:MSYS_NO_PATHCONV
$oldDockerNative = $env:DOCKER_NATIVE
$oldComposeNative = $env:COMPOSE_NATIVE
$env:COMPOSE_FILE = $baseCompose + ';' + $overridePath
$env:SSH_AUTH_SOCK = 'C:/Windows/Temp/unused-ssh-agent'
$env:MSYS_NO_PATHCONV = '1'
$env:DOCKER_NATIVE = $dockerPath.Replace('\', '/')
$env:COMPOSE_NATIVE = $composePath.Replace('\', '/')
$nativeShell = @'
export MSYS_NO_PATHCONV=1
docker() {
    if [ "$1" = compose ]; then shift; "$COMPOSE_NATIVE" "$@";
    else "$DOCKER_NATIVE" "$@"; fi
}
python3() { "$DOCKER_NATIVE" exec uav_1_agent python3 "$@"; }
export -f docker python3
'@

Push-Location $projectRoot
$observerCode = 1
$agentWatchers = @()
try {
    & $composePath @composeArgs run --rm --name $buildName --no-deps --entrypoint bash uav_1_agent -lc `
        'source /opt/ros/humble/setup.bash && source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && colcon build --symlink-install --executor sequential' `
        *> (Join-Path $caseRoot 'build.log')
    if ($LASTEXITCODE -ne 0) { throw '容器內建置失敗；不執行飛行測試。' }
    & $composePath @composeArgs up -d uav_1_agent uav_2_agent uav_3_agent
    if ($LASTEXITCODE -ne 0) { throw '三台 Agent 啟動失敗。' }
    foreach ($agentId in 1..3) {
        $containerId = & $dockerPath inspect --format '{{.Id}}' "uav_${agentId}_agent"
        $agentWatchers += Start-Job -ArgumentList $dockerPath, $containerId, (Join-Path $caseRoot "agent${agentId}.log") -ScriptBlock {
            param($dockerCli, $watchedId, $logPath)
            & $dockerCli wait $watchedId | Out-Null
            if ($LASTEXITCODE -eq 0) {
                & $dockerCli logs $watchedId *> $logPath
                & $dockerCli rm $watchedId | Out-Null
            }
        }
    }
    Start-Sleep -Seconds 25
    & $bashPath -lc ($nativeShell + "`nbash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count 3 --rmw cyclonedds --initial-spacing-m 3.2") `
        *> (Join-Path $caseRoot 'launch.log')
    if ($LASTEXITCODE -ne 0) { throw 'PX4 SITL 啟動失敗。' }
    Start-Sleep -Seconds 10
    & $composePath @composeArgs run -d --name $observerName --no-deps --entrypoint bash uav_1_agent -lc `
        "python3 -u AutoTest/scripts/small_field_three_agent_monitor.py --failed-agent $FailedAgent --scenario $Scenario"
    if ($LASTEXITCODE -ne 0) { throw 'ROS2 驗證容器啟動失敗。' }
    $observerCodeText = & $dockerPath wait $observerName
    $observerCode = [int]($observerCodeText | Select-Object -Last 1)
    & $dockerPath logs $observerName *> (Join-Path $caseRoot 'monitor.jsonl')
    foreach ($agentId in 1..3) {
        & $dockerPath logs "drone${agentId}" *> (Join-Path $caseRoot "px4${agentId}.log")
    }
    # 凍結已完成的模擬後複製原始日誌；失效機不能只靠過期 AgentState 驗證。
    foreach ($agentId in 1..3) {
        & $dockerPath pause "drone${agentId}" | Out-Null
        $instanceId = $agentId - 1
        & $dockerPath cp "drone${agentId}:/root/.local/share/px4/rootfs/${instanceId}/fs/log" (Join-Path $caseRoot "drone${agentId}") `
            2>> (Join-Path $caseRoot 'log-copy-errors.txt')
    }
    [System.IO.File]::WriteAllText((Join-Path $caseRoot 'result.json'),
        (@{ failed_agent = $FailedAgent; live_observer_exit = $observerCode;
            scenario = $Scenario;
            px4_log_check_required = $true; rtl_executed = $false } | ConvertTo-Json))
    Write-Output "SMALL_FIELD_LOG=$caseRoot LIVE_EXIT=$observerCode PX4_LOG_CHECK_REQUIRED=True"
}
finally {
    foreach ($agentId in 1..3) {
        & $dockerPath unpause "drone${agentId}" 2>$null | Out-Null
    }
    & $bashPath -lc ($nativeShell + "`nbash AutoTest/scripts/px4-sitl-launcher/stop_all.sh") `
        *> (Join-Path $caseRoot 'cleanup.log')
    & $composePath @composeArgs stop uav_1_agent uav_2_agent uav_3_agent `
        *>> (Join-Path $caseRoot 'cleanup.log')
    $agentWatchers | Wait-Job -Timeout 25 | Receive-Job *>> (Join-Path $caseRoot 'cleanup.log')
    & $dockerPath rm $observerName 2>> (Join-Path $caseRoot 'cleanup.log') | Out-Null
    $env:COMPOSE_FILE = $oldComposeFile
    $env:SSH_AUTH_SOCK = $oldSshSock
    $env:MSYS_NO_PATHCONV = $oldMsysPathConv
    $env:DOCKER_NATIVE = $oldDockerNative
    $env:COMPOSE_NATIVE = $oldComposeNative
    Pop-Location
}
exit $observerCode
