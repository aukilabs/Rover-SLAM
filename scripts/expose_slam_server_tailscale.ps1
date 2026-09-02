# Expose WSL slam_server :8000 on all Windows interfaces (incl. Tailscale).
# Requires elevation.
$ErrorActionPreference = 'Stop'

netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=8000 2>$null | Out-Null
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8000 connectaddress=127.0.0.1 connectport=8000

if (-not (Get-NetFirewallRule -DisplayName 'Rover SLAM Server 8000' -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName 'Rover SLAM Server 8000' `
        -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8000 | Out-Null
}

Write-Host '=== portproxy ==='
netsh interface portproxy show all
Write-Host '=== probe Tailscale ==='
try {
    $r = Invoke-WebRequest -Uri 'http://100.124.160.70:8000/api/v1/health' -UseBasicParsing -TimeoutSec 5
    Write-Host "health $($r.StatusCode)"
} catch {
    Write-Host "health FAIL: $($_.Exception.Message)"
    exit 1
}
