<#
.SYNOPSIS
    啟動可供本機原生 Taiphoon_QGCS 連線的模擬 Agent（Windows Docker Desktop）。

.DESCRIPTION
    自動取得主機區網 IPv4，以 docker-compose.yml + docker-compose.local-qgcs.yml
    啟動 uav_1_agent～uav_N_agent，並印出 PX4 SITL 與 QGCS 的啟動指令。
    另套用 compose.no-ssh-agent.yml，移除 Windows 無法使用的 SSH agent 掛載。
    不執行 docker build、不修改防火牆或 Docker Desktop 設定，也不發送飛行命令。

.PARAMETER Count
    要啟動的 Agent 數量（1～4）。

.PARAMETER HostIp
    主機區網 IPv4；省略時自動選擇具有預設閘道的網卡位址。

.PARAMETER Stop
    停止本腳本啟動的 Agent 容器（保留映像與 volumes）。

.EXAMPLE
    ./AutoTest/scripts/start_local_qgcs_agents.ps1 -Count 3
#>
param(
    [ValidateRange(1, 4)]
    [int]$Count = 4,
    [string]$HostIp = '',
    [switch]$Stop
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$baseCompose = Join-Path $projectRoot 'docker-compose.yml'
$qgcsCompose = Join-Path $projectRoot 'docker-compose.local-qgcs.yml'
$noSshCompose = Join-Path $PSScriptRoot 'compose.no-ssh-agent.yml'
$composeArgs = @('-f', $baseCompose, '-f', $qgcsCompose, '-f', $noSshCompose)
$services = 1..$Count | ForEach-Object { "uav_${_}_agent" }

function Get-HostLanIpv4
{
    $configs = Get-NetIPConfiguration | Where-Object {
        $null -ne $_.IPv4DefaultGateway -and $_.NetAdapter.Status -eq 'Up'
    }
    foreach ($config in $configs)
    {
        foreach ($address in $config.IPv4Address)
        {
            if ($address.IPAddress -notlike '169.254.*')
            {
                return $address.IPAddress
            }
        }
    }
    throw '找不到具有預設閘道的區網 IPv4，請以 -HostIp 指定。'
}

if ([string]::IsNullOrWhiteSpace($HostIp))
{
    $HostIp = Get-HostLanIpv4
}
$parsedIp = $null
if (-not [System.Net.IPAddress]::TryParse($HostIp, [ref]$parsedIp) -or
    $parsedIp.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork)
{
    throw "主機位址不是有效 IPv4：$HostIp"
}

# SSH agent 掛載已由 compose.no-ssh-agent.yml 移除；未設定時仍給佔位值，
# 只為避免 compose 對 docker-compose.yml 每處 ${SSH_AUTH_SOCK} 印出未設定警告。
$oldSshSock = $env:SSH_AUTH_SOCK
$oldHostIp = $env:TAIPHOON_HOST_IP
if ([string]::IsNullOrWhiteSpace($env:SSH_AUTH_SOCK))
{
    $env:SSH_AUTH_SOCK = '/tmp/taiphoon-no-ssh-agent'
}
$env:TAIPHOON_HOST_IP = $HostIp

Push-Location $projectRoot
try
{
    if ($Stop)
    {
        docker compose @composeArgs stop @services
        if ($LASTEXITCODE -ne 0) { throw 'Agent 容器停止失敗。' }
        return
    }

    docker network inspect Taiphoon_Net *> $null
    if ($LASTEXITCODE -ne 0)
    {
        throw 'Docker 網路 Taiphoon_Net 不存在，請先執行：docker network create Taiphoon_Net'
    }

    docker compose @composeArgs up -d @services
    if ($LASTEXITCODE -ne 0) { throw 'Agent 容器啟動失敗。' }

    Write-Host ''
    Write-Host "Agent 已以本機 QGCS 模式啟動（主機區網 IPv4：$HostIp，數量：$Count）。"
    Write-Host '1. 在 Git Bash 啟動 PX4 SITL（先 Agent、後 PX4）：'
    Write-Host "   MSYS_NO_PATHCONV=1 bash AutoTest/scripts/px4-sitl-launcher/launch_all.sh --count $Count --rmw cyclonedds"
    Write-Host '2. 啟動 Taiphoon_QGCS（本次參數不寫入設定檔）：'
    Write-Host "   Taiphoon_QGCS.exe --interface $HostIp --external-address 192.168.65.254 --no-multicast --peer $HostIp --participant-index 50"
    Write-Host '停止：./AutoTest/scripts/start_local_qgcs_agents.ps1 -Stop'
}
finally
{
    Pop-Location
    $env:SSH_AUTH_SOCK = $oldSshSock
    $env:TAIPHOON_HOST_IP = $oldHostIp
}
