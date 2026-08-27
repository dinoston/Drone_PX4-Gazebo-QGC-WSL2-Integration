#Requires -RunAsAdministrator
$ErrorActionPreference = 'Stop'

Write-Host 'Enabling Windows Subsystem for Linux...'
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
if ($LASTEXITCODE -notin 0, 3010) { throw "WSL feature enable failed: $LASTEXITCODE" }

Write-Host 'Enabling Virtual Machine Platform...'
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
if ($LASTEXITCODE -notin 0, 3010) { throw "VirtualMachinePlatform enable failed: $LASTEXITCODE" }

Write-Host 'Restart Windows before continuing.' -ForegroundColor Green
Read-Host 'Press Enter to close this window'

