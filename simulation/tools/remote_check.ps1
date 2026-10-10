param(
    [string]$Server = '192.168.10.149',
    [int]$Port = 42372,
    [string]$User = 'rnd'
)
$ErrorActionPreference = 'Stop'
# Read-only. Uses the user's existing SSH authentication and trusted host record.
# Does not accept an unknown host key, send a password, install software, or change firewall rules.
& ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=8 -p $Port "$User@$Server" 'cat /etc/os-release; nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader; free -h; df -h /; command -v docker; command -v isaac-ros'
if ($LASTEXITCODE -ne 0) { throw "SSH preflight failed with exit code $LASTEXITCODE" }
