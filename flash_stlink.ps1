<#
.SYNOPSIS
    使用 ST-Link 直接烧录本工程固件 (STM32F407, SSD1963 PID 调参)。

.DESCRIPTION
    调用 ST-LINK_CLI.exe 通过 SWD 连接目标, 烧录 MDK-ARM\LCD\LCD.hex
    (LVGL 固件), 烧录完成后复位并运行。

    用法 (在工程根目录的 PowerShell 中):
        .\flash_stlink.ps1                 # 烧录默认 hex 并复位运行
        .\flash_stlink.ps1 -Erase          # 烧录前先整片擦除
        .\flash_stlink.ps1 -NoRun          # 烧录后只复位, 不自动运行
        .\flash_stlink.ps1 -HexPath .\x.hex   # 指定其他固件
        .\flash_stlink.ps1 -CliPath "D:\...\ST-LINK_CLI.exe"  # 指定 CLI 路径

.PARAMETER HexPath
    要烧录的 hex 文件路径, 默认 <脚本目录>\MDK-ARM\LCD\LCD.hex

.PARAMETER CliPath
    ST-LINK_CLI.exe 的完整路径。缺省时自动在常见安装目录中查找。

.PARAMETER Erase
    烧录前对芯片做整片擦除 (-ME)。

.PARAMETER NoRun
    烧录并复位后不自动运行程序。

.EXAMPLE
    .\flash_stlink.ps1 -Erase -NoRun
#>
[CmdletBinding()]
param(
    [string]$HexPath = "$PSScriptRoot\MDK-ARM\LCD\LCD.hex",
    [string]$CliPath = "",
    [switch]$Erase,
    [switch]$NoRun
)

$ErrorActionPreference = 'Stop'

function Write-Line([string]$msg, [string]$color = 'Gray') {
    Write-Host $msg -ForegroundColor $color
}

# ---- 1. 定位 ST-LINK_CLI.exe ----
$cliCandidates = @(
    "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe",
    "C:\Program Files\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
)
if ($CliPath) { $cliCandidates = @($CliPath) }
$cli = $cliCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $cli) {
    Write-Line "[错误] 未找到 ST-LINK_CLI.exe。请安装 STM32 ST-LINK Utility," "Red"
    Write-Line "       或用 -CliPath 指定其完整路径。" "Red"
    exit 1
}
Write-Line "[信息] ST-LINK_CLI : $cli" "Cyan"

# ---- 2. 检查固件是否存在 ----
if (-not (Test-Path -LiteralPath $HexPath)) {
    Write-Line "[错误] 未找到固件文件: $HexPath" "Red"
    Write-Line "       请先用 Keil 编译生成 MDK-ARM\LCD\LCD.hex, 或用 -HexPath 指定其他固件。" "Red"
    exit 1
}
Write-Line "[信息] 固件文件    : $HexPath" "Cyan"

# ---- 3. 探测连接的 ST-Link ----
Write-Line "[信息] 探测 ST-Link 探针:" "Gray"
& $cli -List
Write-Line ""

# ---- 4. 组装并执行烧录命令 ----
$flashArgs = @("-c", "SWD", "UR")
if ($Erase) { $flashArgs += "-ME" }
$flashArgs += "-P", $HexPath
if ($NoRun) {
    $flashArgs += "-Rst"
} else {
    $flashArgs += "-Rst", "-Run"
}

Write-Line "[执行] $cli $($flashArgs -join ' ')" "DarkYellow"
& $cli @flashArgs
$code = $LASTEXITCODE

if ($code -eq 0) {
    Write-Line ""
    Write-Line "[完成] 烧录成功, 程序已复位运行。" "Green"
} else {
    Write-Line ""
    Write-Line "[错误] 烧录失败, 退出码 $code。请检查:" "Red"
    Write-Line "       1. ST-Link 是否已连接到开发板 (SWDIO/SWCLK/GND/3V3)" "Red"
    Write-Line "       2. 开发板是否上电、目标是否被连接线占用" "Red"
    Write-Line "       3. 是否被其他调试器/软件锁定" "Red"
}
exit $code
