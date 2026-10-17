<# : batch portion (valid Batch AND valid PowerShell block comment!)
@echo off
setlocal
cd /d "%~dp0"
title FRC 10413 FLUX - Match Log Downloader
echo ============================================================
echo   FRC Team 10413 (FLUX Robotics) - Match Log Downloader
echo ============================================================
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$script = Get-Content -LiteralPath '%~f0' -Raw; Invoke-Expression $script" -- %*
echo.
pause
exit /b
#>

# ==============================================================================
# Single-File Zero-Dependency Windows Log Downloader & Match Correlator
# Just put this ONE file (pull-logs.bat) on the Driver Station Desktop and
# double-click it!
# ==============================================================================

$ErrorActionPreference = "Continue"
Add-Type -AssemblyName System.IO.Compression.FileSystem

$Clean = $false
$HostIp = ""
foreach ($a in $args) {
    if ($a -match "^(-Clean|--clean|-c)$") { $Clean = $true }
    elseif ($a -notmatch "^-") { $HostIp = $a }
}

function Get-LogMetadata {
    param([System.IO.FileInfo]$File)

    $name  = $File.Name
    $ext   = $File.Extension.ToLower()
    $mtime = $File.LastWriteTime
    $startTs = $null
    $suffix  = ""
    $kind    = "CONSOLE"

    if ($ext -eq ".wpilog") {
        $kind = "WPILOG"
        if ($name -match "(?i)akit_(\d{2})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2})(?:_([A-Za-z0-9_-]+))?\.wpilog$") {
            try {
                $startTs = Get-Date -Year (2000 + [int]$Matches[1]) -Month ([int]$Matches[2]) -Day ([int]$Matches[3]) `
                                    -Hour ([int]$Matches[4]) -Minute ([int]$Matches[5]) -Second ([int]$Matches[6])
            } catch {}
            if ($Matches[7]) { $suffix = $Matches[7] }
        }
    }
    elseif ($ext -eq ".hoot") {
        $kind = "HOOT"
        if ($name -match "(?i)(\d{4})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2}).*\.hoot$") {
            try {
                $startTs = Get-Date -Year ([int]$Matches[1]) -Month ([int]$Matches[2]) -Day ([int]$Matches[3]) `
                                    -Hour ([int]$Matches[4]) -Minute ([int]$Matches[5]) -Second ([int]$Matches[6])
            } catch {}
        }
    }
    elseif ($ext -eq ".dslog" -or $ext -eq ".dsevents") {
        $kind = $ext.TrimStart('.').ToUpper()
        if ($name -match "(?i)(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2})\.(dslog|dsevents)$") {
            try {
                $startTs = Get-Date -Year ([int]$Matches[1]) -Month ([int]$Matches[2]) -Day ([int]$Matches[3]) `
                                    -Hour ([int]$Matches[4]) -Minute ([int]$Matches[5]) -Second ([int]$Matches[6])
            } catch {}
        }
    }

    $refTs = if ($null -ne $startTs) { $startTs } else { $mtime }
    return [PSCustomObject]@{
        File    = $File
        Kind    = $kind
        StartTs = $startTs
        EndTs   = $mtime
        RefTs   = $refTs
        Suffix  = $suffix
    }
}

# 1. Auto-detect RoboRIO (USB 172.22.11.2 -> Wi-Fi 10.104.13.2 -> mDNS)
$Candidates = @("172.22.11.2", "10.104.13.2", "roboRIO-10413-FRC.local")
if ($HostIp -ne "") { $Candidates = @($HostIp) }

$SshOpts = @("-o", "ConnectTimeout=3", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=NUL", "-o", "LogLevel=ERROR")

$RioHost = $null
Write-Host "Searching for Team 10413 RoboRIO..." -ForegroundColor Cyan
foreach ($ip in $Candidates) {
    $res = & ssh @SshOpts "lvuser@$ip" "echo ok" 2>$null
    if ($LASTEXITCODE -eq 0 -and ($res -join "") -match "ok") {
        $RioHost = $ip
        break
    }
}

$RobotName = "FUEL"
if ($RioHost) {
    $Comments = (& ssh @SshOpts "lvuser@$RioHost" "cat /etc/Robot.comments 2>/dev/null || echo ''" 2>$null) -join ""
    if ($Comments -match "CORAL") { $RobotName = "CORAL" }
} else {
    Write-Host "WARNING: Could not reach RoboRIO on ($($Candidates -join ', '))." -ForegroundColor Yellow
    Write-Host "  -> Will still collect & correlate local Driver Station logs on this PC." -ForegroundColor Yellow
}

# Determine output directory: right next to pull-logs.bat (e.g. Desktop\match-logs\...)
$ScriptDir = (Get-Location).Path
$DateStr   = Get-Date -Format "yyyy-MM-dd"
$TimeStr   = Get-Date -Format "HHmmss"
$DestDir   = Join-Path $ScriptDir "match-logs\${DateStr}_${RobotName}"
$RawDir    = Join-Path $DestDir "raw"
New-Item -ItemType Directory -Force -Path $RawDir | Out-Null

# 2. Download Robot Logs (.wpilog, .hoot, FRC_UserProgram.log)
$DownloadedFromRio = $false
$RemoteDirs = "/home/lvuser/logs /home/lvuser /U /media/sda1"
if ($RioHost) {
    Write-Host "Connected to RoboRIO ($RobotName) at lvuser@$RioHost" -ForegroundColor Green
    Write-Host "Saving logs to: $DestDir"
    Write-Host "------------------------------------------------------------"

    $RemoteFilesRaw = & ssh @SshOpts "lvuser@$RioHost" "find $RemoteDirs -maxdepth 3 \( -name '*.wpilog' -o -name '*.hoot' \) 2>/dev/null | sort -u" 2>$null
    $RemoteFiles = @($RemoteFilesRaw | Where-Object { $_ -and $_.Trim() -ne "" })

    if ($RemoteFiles.Count -gt 0) {
        Write-Host "[1/4] Downloading $($RemoteFiles.Count) robot log file(s) (.wpilog / .hoot)..." -ForegroundColor Yellow
        foreach ($rfile in $RemoteFiles) {
            $rf = $rfile.Trim()
            & scp -p @SshOpts "lvuser@${RioHost}:`"$rf`"" "$RawDir\"
        }
        $DownloadedFromRio = $true
    } else {
        Write-Host "[1/4] No .wpilog or .hoot files found on RoboRIO." -ForegroundColor Yellow
    }

    Write-Host "[2/4] Pulling RoboRIO Java console/crash log (FRC_UserProgram.log)..." -ForegroundColor Cyan
    & scp -p @SshOpts "lvuser@${RioHost}:/home/lvuser/FRC_UserProgram.log" "$RawDir\FRC_UserProgram_${TimeStr}.log" 2>$null
} else {
    Write-Host "[1/4] Skipped RoboRIO .wpilog/.hoot (not connected)." -ForegroundColor DarkGray
    Write-Host "[2/4] Skipped RoboRIO console log (not connected)." -ForegroundColor DarkGray
}

# 3. Copy local Driver Station Logs (.dslog / .dsevents)
$DsLogDir = "C:\Users\Public\Documents\FRC\Log Files"
if (Test-Path $DsLogDir) {
    Write-Host "[3/4] Copying recent Driver Station logs (.dslog / .dsevents)..." -ForegroundColor Cyan
    $DsDest = Join-Path $RawDir "ds-logs"
    New-Item -ItemType Directory -Force -Path $DsDest | Out-Null
    $Cutoff = (Get-Date).AddDays(-2)
    Get-ChildItem -Path $DsLogDir -Include "*.dslog", "*.dsevents" -File -Recurse |
        Where-Object { $_.LastWriteTime -ge $Cutoff } |
        Copy-Item -Destination $DsDest -Force
} else {
    Write-Host "[3/4] No local Driver Station log folder found at $DsLogDir (skipped)." -ForegroundColor DarkGray
}

# 4. Correlate all logs by match into folders and .zip files
Write-Host "[4/4] Correlating logs by match into folders and .zip archives..." -ForegroundColor Cyan

$AllFiles = @(
    Get-ChildItem -Path $RawDir -Recurse -File |
    Where-Object { $_.Extension -in @(".wpilog", ".hoot", ".dslog", ".dsevents") -or $_.Name -like "FRC_UserProgram*" } |
    ForEach-Object { Get-LogMetadata -File $_ }
)

$MatchesOut = Join-Path $DestDir "matches"
if ($AllFiles.Count -eq 0) {
    Write-Host "No logs found to correlate in $RawDir." -ForegroundColor Yellow
} else {
    $WpiLogs   = @($AllFiles | Where-Object { $_.Kind -eq "WPILOG" } | Sort-Object RefTs, EndTs)
    $OtherLogs = @($AllFiles | Where-Object { $_.Kind -ne "WPILOG" })

    $Bundles = New-Object System.Collections.ArrayList
    $idx = 1
    foreach ($w in $WpiLogs) {
        $startTs = if ($null -ne $w.StartTs) { $w.StartTs } else { $w.EndTs.AddMinutes(-4) }
        $endTs   = if ($w.EndTs -gt $startTs.AddMinutes(1)) { $w.EndTs } else { $startTs.AddMinutes(3) }
        $tsStr   = $startTs.ToString("yyyy-MM-dd_HH-mm-ss")
        $sfx     = if ($w.Suffix -ne "") { $w.Suffix -replace "[^A-Za-z0-9_-]", "_" } else { "Session" }
        $label   = "Match_{0:D2}_{1}_{2}" -f $idx, $tsStr, $sfx

        $fileList = New-Object System.Collections.ArrayList
        [void]$fileList.Add($w)

        [void]$Bundles.Add([PSCustomObject]@{
            Index   = $idx
            WpiLog  = $w
            StartTs = $startTs
            EndTs   = $endTs
            Label   = $label
            Files   = $fileList
        })
        $idx++
    }

    $WindowPadMin = 4.0
    $Unmatched = New-Object System.Collections.ArrayList

    foreach ($lf in $OtherLogs) {
        if ($lf.Kind -eq "CONSOLE") {
            if ($Bundles.Count -gt 0) {
                [void]$Bundles[$Bundles.Count - 1].Files.Add($lf)
            } else {
                [void]$Unmatched.Add($lf)
            }
            continue
        }

        $bestBundle = $null
        $bestDistSec = [double]::MaxValue
        foreach ($b in $Bundles) {
            $winStart = $b.StartTs.AddMinutes(-$WindowPadMin)
            $winEnd   = $b.EndTs.AddMinutes($WindowPadMin)
            $midTicks = ($b.StartTs.Ticks + $b.EndTs.Ticks) / 2
            $midTime  = [datetime]$midTicks

            foreach ($cand in @($lf.RefTs, $lf.EndTs)) {
                if ($cand -ge $winStart -and $cand -le $winEnd) {
                    $dist = [Math]::Abs(($cand - $midTime).TotalSeconds)
                    if ($dist -lt $bestDistSec) {
                        $bestDistSec = $dist
                        $bestBundle  = $b
                    }
                }
            }
        }

        if ($null -ne $bestBundle) {
            [void]$bestBundle.Files.Add($lf)
        } else {
            [void]$Unmatched.Add($lf)
        }
    }

    $SortedUnmatched = @($Unmatched | Sort-Object RefTs)
    foreach ($lf in $SortedUnmatched) {
        $lastBundle = if ($Bundles.Count -gt 0) { $Bundles[$Bundles.Count - 1] } else { $null }
        if ($null -ne $lastBundle -and $null -eq $lastBundle.WpiLog -and [Math]::Abs(($lf.RefTs - $lastBundle.EndTs).TotalMinutes) -le $WindowPadMin) {
            [void]$lastBundle.Files.Add($lf)
            if ($lf.EndTs -gt $lastBundle.EndTs) { $lastBundle.EndTs = $lf.EndTs }
        } else {
            $idx = $Bundles.Count + 1
            $tsStr = $lf.RefTs.ToString("yyyy-MM-dd_HH-mm-ss")
            $label = "Match_{0:D2}_{1}_Unmatched" -f $idx, $tsStr
            $fileList = New-Object System.Collections.ArrayList
            [void]$fileList.Add($lf)
            [void]$Bundles.Add([PSCustomObject]@{
                Index   = $idx
                WpiLog  = $null
                StartTs = $lf.RefTs
                EndTs   = $lf.EndTs
                Label   = $label
                Files   = $fileList
            })
        }
    }

    New-Item -ItemType Directory -Force -Path $MatchesOut | Out-Null

    foreach ($b in $Bundles) {
        $mDir = Join-Path $MatchesOut $b.Label
        if (Test-Path $mDir) { Remove-Item $mDir -Recurse -Force }
        New-Item -ItemType Directory -Force -Path $mDir | Out-Null

        $summaryLines = @(
            "Match Bundle: $($b.Label)",
            "Window Start: $($b.StartTs.ToString('yyyy-MM-dd HH:mm:ss'))",
            "Window End:   $($b.EndTs.ToString('yyyy-MM-dd HH:mm:ss'))",
            "Files Included:"
        )

        foreach ($lf in $b.Files) {
            $destPath = Join-Path $mDir $lf.File.Name
            Copy-Item -Path $lf.File.FullName -Destination $destPath -Force
            $sizeKb = [Math]::Round($lf.File.Length / 1024.0, 1)
            $summaryLines += ("  - [{0,-8}] {1} ({2} KB)" -f $lf.Kind, $lf.File.Name, $sizeKb)
        }

        $summaryFile = Join-Path $mDir "match-summary.txt"
        $summaryLines | Set-Content -Path $summaryFile -Encoding UTF8

        $zipPath = Join-Path $MatchesOut "$($b.Label).zip"
        if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
        [System.IO.Compression.ZipFile]::CreateFromDirectory($mDir, $zipPath)
    }
}

if ($RioHost) {
    Write-Host "------------------------------------------------------------"
    Write-Host "RoboRIO Storage Status:" -ForegroundColor Cyan
    & ssh @SshOpts "lvuser@$RioHost" "df -h /home/lvuser"

    if ($DownloadedFromRio) {
        $doClean = $Clean
        if (-not $doClean) {
            Write-Host ""
            $ans = Read-Host "Delete downloaded .wpilog/.hoot logs from RoboRIO now to free up space? [y/N]"
            if ($ans -match "^[Yy]") { $doClean = $true }
        }
        if ($doClean) {
            Write-Host "Cleaning remote .wpilog and .hoot logs from RoboRIO..." -ForegroundColor Yellow
            & ssh @SshOpts "lvuser@$RioHost" "find $RemoteDirs -maxdepth 3 \( -name '*.wpilog' -o -name '*.hoot' \) -delete 2>/dev/null || true"
            Write-Host "Remote logs cleared! Updated RoboRIO storage:" -ForegroundColor Green
            & ssh @SshOpts "lvuser@$RioHost" "df -h /home/lvuser"
        }
    }
}

Write-Host "------------------------------------------------------------"
Write-Host "Done! Per-match folders and .zip archives saved in:" -ForegroundColor Green
Write-Host "  $MatchesOut" -ForegroundColor White
if (Test-Path $MatchesOut) {
    Get-ChildItem -Path $MatchesOut -Filter "*.zip" |
        Select-Object Name, @{Name="Size (MB)";Expression={[Math]::Round($_.Length / 1MB, 2)}}, LastWriteTime |
        Format-Table -AutoSize
    # Open the folder in Windows Explorer automatically
    Start-Process explorer.exe -ArgumentList "`"$MatchesOut`""
}
