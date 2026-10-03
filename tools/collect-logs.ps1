<#
.SYNOPSIS
  Collects all logs from a test run into one folder: AdvantageKit .wpilog from the
  roboRIO, Driver Station .dslog/.dsevents, and PhotonVision logs.

.EXAMPLE
  .\tools\collect-logs.ps1 -Name visiontiming_test1 -Notes "lag spike ~1:30 into teleop"

.EXAMPLE
  .\tools\collect-logs.ps1 -Count 3 -Hours 4 -Clean
#>
param(
  # Label appended to the output folder name
  [string]$Name = "run",
  # Free-text notes saved as notes.txt
  [string]$Notes = "",
  # Number of newest .wpilog files to pull from the roboRIO
  [int]$Count = 1,
  # Pull DS logs modified within this many hours
  [double]$Hours = 2,
  # Delete the pulled .wpilog files from the roboRIO after a successful copy
  [switch]$Clean,
  [string]$Dest = "$HOME\robot-logs",
  [string]$Robot = "10.104.13.2",
  [string[]]$PhotonHosts = @("photonvision.local"),
  [string]$DsLogDir = "C:\Users\Public\Documents\FRC\Log Files"
)

$ErrorActionPreference = "Stop"
$RioLogDir = "/home/lvuser/logs"
# roboRIO host keys change on every reimage; don't let that block log collection
$SshOpts = @("-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=NUL", "-o", "ConnectTimeout=5")

$outDir = Join-Path $Dest ("{0}_{1}" -f (Get-Date -Format "yyyy-MM-dd_HHmm"), $Name)
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
Write-Host "Collecting into $outDir`n"
$summary = @()

# --- 1. roboRIO AdvantageKit logs ---
Write-Host "[1/3] roboRIO ($Robot)" -ForegroundColor Cyan
try {
  $remote = & ssh @SshOpts "lvuser@$Robot" "ls -t $RioLogDir/*.wpilog 2>/dev/null | head -n $Count"
  if ($LASTEXITCODE -ne 0) { throw "ssh failed (exit $LASTEXITCODE)" }
  $remote = @($remote | Where-Object { $_ })
  if ($remote.Count -eq 0) { throw "no .wpilog files in $RioLogDir" }

  foreach ($f in $remote) {
    Write-Host "  $f"
    & scp @SshOpts "lvuser@${Robot}:$f" $outDir
    if ($LASTEXITCODE -ne 0) { throw "scp failed for $f" }
  }
  $summary += "roboRIO:     $($remote.Count) .wpilog"

  if ($Clean) {
    & ssh @SshOpts "lvuser@$Robot" ("rm -f " + ($remote -join " "))
    Write-Host "  Deleted pulled logs from roboRIO"
  }
} catch {
  Write-Warning "roboRIO: $_"
  $summary += "roboRIO:     FAILED ($_)"
}

# --- 2. Driver Station logs ---
Write-Host "`n[2/3] Driver Station logs ($DsLogDir)" -ForegroundColor Cyan
if (Test-Path $DsLogDir) {
  $since = (Get-Date).AddHours(-$Hours)
  $dsFiles = @(Get-ChildItem -Path $DsLogDir -Recurse -File -Include *.dslog, *.dsevents |
    Where-Object { $_.LastWriteTime -gt $since })
  $dsFiles | ForEach-Object { Write-Host "  $($_.Name)"; Copy-Item $_.FullName $outDir }
  $summary += "DS:          $($dsFiles.Count) files (last $Hours h)"
} else {
  Write-Warning "DS log folder not found - not running on the Driver Station laptop?"
  $summary += "DS:          SKIPPED (folder not found)"
}

# --- 3. PhotonVision logs ---
Write-Host "`n[3/3] PhotonVision" -ForegroundColor Cyan
foreach ($ph in $PhotonHosts) {
  $file = Join-Path $outDir "photonvision_$($ph -replace '[^\w.-]', '_').txt"
  try {
    Invoke-WebRequest -Uri "http://${ph}:5800/api/utils/photonvision-journalctl.txt" `
      -OutFile $file -TimeoutSec 10 -UseBasicParsing
    Write-Host "  $ph -> $(Split-Path $file -Leaf)"
    $summary += "PhotonVision: $ph OK"
  } catch {
    Write-Warning "PhotonVision ${ph}: $($_.Exception.Message) (export manually: web UI > Settings)"
    $summary += "PhotonVision: $ph FAILED"
  }
}

# --- Notes ---
@(
  "Collected: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
  "Name:      $Name"
  "Notes:     $Notes"
  ""
) + $summary | Set-Content (Join-Path $outDir "notes.txt")

Write-Host "`nDone:" -ForegroundColor Green
$summary | ForEach-Object { Write-Host "  $_" }
Invoke-Item $outDir
