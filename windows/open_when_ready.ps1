param([string]$Url = 'http://localhost:8080', [int]$TimeoutSec = 180)
# Wait until the web server inside WSL answers, then open the default browser once.
$deadline = (Get-Date).AddSeconds($TimeoutSec)
while ((Get-Date) -lt $deadline) {
    try {
        $r = Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 2
        if ($r.StatusCode -eq 200) { Start-Process $Url; exit 0 }
    } catch { }
    Start-Sleep -Seconds 2
}
