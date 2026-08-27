#Requires -RunAsAdministrator
$ErrorActionPreference = 'Stop'

$target = 'E:\WSL\Ubuntu-24.04'
if (Test-Path -LiteralPath $target) { throw "Target already exists: $target" }

wsl.exe --install Ubuntu-24.04 --location $target --name Ubuntu-24.04-E --version 2 --no-launch --web-download
if ($LASTEXITCODE -ne 0) { throw "Ubuntu installation failed: $LASTEXITCODE" }

wsl.exe -l -v
Read-Host 'Press Enter to close this window'

