<#
.SYNOPSIS
    一键编译 STM32F407CtrE (PID 调参) Keil 工程, 生成可烧录的 LCD.hex,
    并可串联现有 flash_stlink.ps1 直接烧录。

.DESCRIPTION
    通过 UV4.exe 命令行调用 Keil 编译 MDK-ARM\LCD.uvprojx,
    成功后输出 MDK-ARM\LCD\LCD.hex 的路径/大小/时间。
    使用 -Flash 时, 编译成功后自动调用同目录 flash_stlink.ps1 烧录并运行。

    用法 (在工程根目录的 PowerShell 中):
        .\build.ps1                  # 增量编译 (只编译改动)
        .\build.ps1 -Rebuild         # 全量重新编译
        .\build.ps1 -Clean           # 清理编译输出后退出
        .\build.ps1 -Flash           # 编译成功后自动烧录并运行
        .\build.ps1 -Flash -Erase    # 烧录前整片擦除
        .\build.ps1 -Flash -NoRun    # 烧录后只复位不运行
        .\build.ps1 -Uv4 "D:\app\keil\UV4\UV4.exe"   # 指定 UV4 路径

.PARAMETER Rebuild
    全量重新编译 (UV4 -r), 否则增量编译 (UV4 -b)。

.PARAMETER Clean
    删除 MDK-ARM\LCD 输出目录及日志后退出, 不编译。

.PARAMETER Flash
    编译成功后调用 flash_stlink.ps1 烧录。

.PARAMETER Erase
    透传给烧录脚本: 烧录前整片擦除。

.PARAMETER NoRun
    透传给烧录脚本: 烧录后只复位不运行。

.PARAMETER Uv4
    UV4.exe 完整路径, 缺省时先查注册表, 再查常见目录。

.EXAMPLE
    .\build.ps1 -Rebuild -Flash
#>
[CmdletBinding()]
param(
    [switch]$Rebuild,
    [switch]$Clean,
    [switch]$Flash,
    [switch]$Erase,
    [switch]$NoRun,
    [string]$Uv4 = ""
)

$ErrorActionPreference = 'Stop'

function Write-Step([string]$msg) { Write-Host "`n>>> $msg" -ForegroundColor Cyan }
function Write-Ok([string]$msg)   { Write-Host "    $msg" -ForegroundColor Green }
function Write-Warn2([string]$msg){ Write-Host "    $msg" -ForegroundColor Yellow }
function Write-Err([string]$msg)  { Write-Host "    $msg" -ForegroundColor Red }

# ---- 路径 (脚本位于工程根目录) ----
$Root      = $PSScriptRoot
$MdkDir    = Join-Path $Root 'MDK-ARM'
$Proj      = Join-Path $MdkDir 'LCD.uvprojx'
$OutDir    = Join-Path $MdkDir 'LCD'
$Hex       = Join-Path $OutDir 'LCD.hex'
$Log       = Join-Path $MdkDir 'build.log'
$FlashScript = Join-Path $Root 'flash_stlink.ps1'

# ============ -Clean: 仅清理 ============
if ($Clean) {
    Write-Step "清理编译输出"
    if (Test-Path -LiteralPath $OutDir) {
        Remove-Item -LiteralPath $OutDir -Recurse -Force
        Write-Ok "已删除: $OutDir"
    } else {
        Write-Warn2 "输出目录不存在, 无需清理: $OutDir"
    }
    if (Test-Path -LiteralPath $Log) { Remove-Item -LiteralPath $Log -Force }
    exit 0
}

# ---- 校验工程文件 ----
if (-not (Test-Path -LiteralPath $Proj)) {
    Write-Err "未找到 Keil 工程: $Proj"
    exit 1
}

# ---- 定位 UV4.exe: 显式参数 > 注册表 > 常见路径 ----
if (-not $Uv4) {
    $reg = $null
    foreach ($rk in @('HKLM:\SOFTWARE\WOW6432Node\Keil\Products\MDK',
                      'HKLM:\SOFTWARE\Keil\Products\MDK')) {
        if (Test-Path $rk) {
            $p = (Get-ItemProperty $rk).Path
            if ($p) { $reg = Split-Path $p -Parent }   # D:\app\keil\ARM -> D:\app\keil
        }
    }
    $candidates = @()
    if ($reg) { $candidates += (Join-Path $reg 'UV4\UV4.exe') }
    $candidates += @(
        'D:\app\keil\UV4\UV4.exe',
        'C:\Keil_v5\UV4\UV4.exe',
        'D:\Keil_v5\UV4\UV4.exe',
        'C:\Keil\UV4\UV4.exe'
    )
    $Uv4 = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
}
if (-not $Uv4 -or -not (Test-Path -LiteralPath $Uv4)) {
    Write-Err "未找到 UV4.exe, 请用 -Uv4 指定其完整路径。"
    exit 1
}

Write-Host "==== STM32F407CtrE 编译 ====" -ForegroundColor White
Write-Host "UV4   : $Uv4"
Write-Host "工程  : $Proj"
Write-Host "模式  : $(if ($Rebuild) {'全量 Rebuild'} else {'增量 Build'})"

# ---- 记录编译前 hex 时间, 用于判断是否更新 ----
$hexBefore = $null
if (Test-Path -LiteralPath $Hex) { $hexBefore = (Get-Item -LiteralPath $Hex).LastWriteTime }

# ---- 调用 UV4: -r rebuild / -b build, -o 日志 ----
$mode = if ($Rebuild) { '-r' } else { '-b' }
Write-Step "开始编译 (日志: $Log)"

# UV4 是 GUI 程序, 必须用 Start-Process -Wait 等待编译结束, 否则会立即返回
$argList = @($mode, "`"$Proj`"", '-j0', '-o', "`"$Log`"")
$proc = Start-Process -FilePath $Uv4 -ArgumentList $argList -Wait -PassThru -WorkingDirectory $MdkDir
$code = $proc.ExitCode

# UV4 返回码: 0=成功无警告, 1=成功有警告, 2/3=错误, 11=找不到工程,
#             12=文件不存在, 15=写文件错误, 20=设备错误, 其他=异常
$success = ($code -eq 0 -or $code -eq 1)

# 输出日志尾部 (UV4 日志为简短文本)
if (Test-Path -LiteralPath $Log) {
    Write-Host ""
    Get-Content -LiteralPath $Log -Tail 40 | ForEach-Object {
        if ($_ -match 'error|错误') { Write-Host "    $_" -ForegroundColor Red }
        elseif ($_ -match 'warning|警告') { Write-Host "    $_" -ForegroundColor Yellow }
        else { Write-Host "    $_" -ForegroundColor DarkGray }
    }
}

if (-not $success) {
    Write-Err "编译失败, UV4 退出码 $code。"
    switch ($code) {
        11 { Write-Err "找不到工程文件。" }
        12 { Write-Err "工程引用的文件不存在。" }
        20 { Write-Err "设备型号错误。" }
        default { Write-Err "请根据上面日志排查。" }
    }
    exit $code
}

# ---- 校验 hex ----
if (-not (Test-Path -LiteralPath $Hex)) {
    Write-Err "编译报告成功但未找到 hex: $Hex"
    exit 2
}
$hexItem = Get-Item -LiteralPath $Hex
$sizeKB = [math]::Round($hexItem.Length / 1KB, 1)

Write-Step "编译完成"
if ($code -eq 0) { Write-Ok "状态: 成功 (无警告)" }
else { Write-Warn2 "状态: 成功 (有警告, 退出码 1)" }
Write-Ok "固件: $Hex"
Write-Ok "大小: $sizeKB KB   生成时间: $($hexItem.LastWriteTime)"
if ($hexBefore -and $hexItem.LastWriteTime -le $hexBefore) {
    Write-Warn2 "提示: hex 时间戳未更新 (可能无文件改动, 属正常增量编译)。"
}

# ============ -Flash: 串联烧录脚本 ============
if ($Flash) {
    Write-Step "串联烧录脚本 flash_stlink.ps1"
    if (-not (Test-Path -LiteralPath $FlashScript)) {
        Write-Err "未找到烧录脚本: $FlashScript"
        exit 1
    }
    $flashArgs = @('-HexPath', $Hex)
    if ($Erase) { $flashArgs += '-Erase' }
    if ($NoRun) { $flashArgs += '-NoRun' }
    $argStr = $flashArgs -join ' '
    Write-Host "    -> flash_stlink.ps1 $argStr" -ForegroundColor DarkYellow
    & $FlashScript @flashArgs
    exit $LASTEXITCODE
}

exit 0
