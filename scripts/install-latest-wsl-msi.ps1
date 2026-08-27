#Requires -RunAsAdministrator
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

$release = Invoke-RestMethod 'https://api.github.com/repos/microsoft/WSL/releases/latest'
$asset = @($release.assets | Where-Object { $_.name -like '*.x64.msi' })
if ($asset.Count -ne 1) { throw 'Could not identify the official x64 WSL MSI.' }

$installer = Join-Path $env:TEMP $asset[0].name
Invoke-WebRequest -Uri $asset[0].browser_download_url -OutFile $installer
$process = Start-Process msiexec.exe -ArgumentList @('/i', $installer, '/passive', '/norestart') -Wait -PassThru
if ($process.ExitCode -notin 0, 3010) { throw "WSL installation failed: $($process.ExitCode)" }

wsl.exe --version
Read-Host 'Press Enter to close this window'

