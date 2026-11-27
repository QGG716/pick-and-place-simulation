# ECO65 remote GPU inference host on a shared Windows machine.
# Run in an ADMINISTRATOR PowerShell on the new server:
#   powershell -ExecutionPolicy Bypass -File windows_wsl_setup.ps1
# Isolation from colleagues: one NEW WSL distro "eco65-gpu" in C:\eco65-gpu
# (existing distros, Python, conda, CUDA, .wslconfig and the default distro are
# not touched), sshd only inside that distro on port 2222, key login only.
# System-level additions, all named "eco65-gpu": one portproxy entry for 2222,
# one inbound firewall rule for 2222, one startup scheduled task.
$ErrorActionPreference = 'Stop'
$Distro = 'eco65-gpu'; $Root = 'C:\eco65-gpu'; $Port = 2222
$PubKey = 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICI+fzNFCDTguYDOapuHFocxjQBBbf+ekVUm3UxbdCzF eco65-field-pc-gpu-inference-20261113'
$Urls = @(
  'https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cloud-images/wsl/jammy/current/ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz',
  'https://cloud-images.ubuntu.com/wsl/jammy/current/ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz')

Write-Host '== GPU / WSL report'
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
wsl --version
wsl -l -v
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' } | Format-Table InterfaceAlias,IPAddress

New-Item -ItemType Directory -Force -Path $Root | Out-Null
$existing = (wsl -l -q) -replace "`0", '' | Where-Object { $_.Trim() -eq $Distro }
if (-not $existing) {
  $tar = Join-Path $Root 'jammy-rootfs.tar.gz'
  if (-not (Test-Path $tar)) {
    foreach ($u in $Urls) { try { Write-Host "download $u"; curl.exe -fL --retry 3 -o $tar $u; break } catch { Write-Warning $_ } }
  }
  wsl --import $Distro (Join-Path $Root 'wsl') $tar --version 2
}

$setup = @"
set -e
export DEBIAN_FRONTEND=noninteractive
sed -i 's#http://archive.ubuntu.com#https://mirrors.tuna.tsinghua.edu.cn#g; s#http://security.ubuntu.com#https://mirrors.tuna.tsinghua.edu.cn#g' /etc/apt/sources.list || true
apt-get update -q && apt-get install -y -q openssh-server python3-venv python3-pip rsync curl ca-certificates
mkdir -p /run/sshd /root/.ssh /root/autodl-tmp/eco65-gpu-inference && chmod 700 /root/.ssh
grep -qF '$PubKey' /root/.ssh/authorized_keys 2>/dev/null || echo '$PubKey' >> /root/.ssh/authorized_keys
chmod 600 /root/.ssh/authorized_keys
cat > /etc/ssh/sshd_config.d/eco65.conf <<EOF2
Port $Port
PasswordAuthentication no
KbdInteractiveAuthentication no
PermitRootLogin prohibit-password
EOF2
ssh-keygen -A
/usr/sbin/sshd -t && echo SSHD_CONFIG_OK
ls /usr/lib/wsl/lib/nvidia-smi >/dev/null 2>&1 && /usr/lib/wsl/lib/nvidia-smi -L || echo 'WSL_GPU_NOT_VISIBLE'
"@
wsl -d $Distro -u root -- bash -c ($setup -replace "`r", '')

# Startup task: keep sshd running in the distro and point the portproxy at its current IP.
$start = @"
`$ip = (wsl -d $Distro -u root -- hostname -I).Trim().Split(' ')[0]
netsh interface portproxy delete v4tov4 listenport=$Port listenaddress=0.0.0.0 2>`$null | Out-Null
netsh interface portproxy add v4tov4 listenport=$Port listenaddress=0.0.0.0 connectport=$Port connectaddress=`$ip
wsl -d $Distro -u root -- /usr/sbin/sshd -D
"@
Set-Content -Path (Join-Path $Root 'start-sshd.ps1') -Value $start -Encoding UTF8
$action = New-ScheduledTaskAction -Execute 'powershell.exe' -Argument "-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File $Root\start-sshd.ps1"
# WSL distros are registered per Windows user, so the task runs as this user at logon
# (the remote-desktop tool keeps that session logged in); no password is stored.
$me = "$env:USERDOMAIN\$env:USERNAME"
$trigger = New-ScheduledTaskTrigger -AtLogOn -User $me
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit ([TimeSpan]::Zero) -RestartCount 99 -RestartInterval (New-TimeSpan -Minutes 1)
$principal = New-ScheduledTaskPrincipal -UserId $me -LogonType Interactive -RunLevel Highest
Register-ScheduledTask -TaskName 'eco65-gpu-sshd' -Action $action -Trigger $trigger -Settings $settings -Principal $principal -Force | Out-Null
if (-not (Get-NetFirewallRule -DisplayName 'eco65-gpu-sshd' -ErrorAction SilentlyContinue)) {
  New-NetFirewallRule -DisplayName 'eco65-gpu-sshd' -Direction Inbound -Protocol TCP -LocalPort $Port -Action Allow | Out-Null
}
Start-ScheduledTask -TaskName 'eco65-gpu-sshd'
Start-Sleep 5
netsh interface portproxy show v4tov4
Test-NetConnection -ComputerName 127.0.0.1 -Port $Port | Select-Object TcpTestSucceeded
Write-Host 'ECO65_GPU_SETUP_DONE'
