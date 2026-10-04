#Requires -Version 5.1
#Requires -RunAsAdministrator
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
if (-not [Environment]::Is64BitOperatingSystem -or $env:PROCESSOR_ARCHITECTURE -eq 'ARM64') {
    throw 'Questo profilo richiede Windows x86_64 (immagine linux/amd64).'
}
if (-not (Get-Command winget.exe -ErrorAction SilentlyContinue)) {
    throw 'Installare/aggiornare App Installer (winget) dal Microsoft Store, poi rieseguire.'
}
if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    throw 'wsl.exe non disponibile: aggiornare Windows e abilitare Windows Subsystem for Linux.'
}

function Invoke-Checked {
    param([string]$Program, [string[]]$Arguments)
    & $Program @Arguments
    if ($LASTEXITCODE -eq 3010 -or $LASTEXITCODE -eq 1641) {
        Write-Host 'Riavvio richiesto. Riavviare Windows, poi rieseguire questo script.'
        exit 0
    }
    if ($LASTEXITCODE -ne 0) {
        throw "$Program non riuscito (codice $LASTEXITCODE). Dopo un riavvio, rieseguire lo script."
    }
}

# No forced reboot or unattended acceptance of Docker Desktop's agreement.
Invoke-Checked -Program 'wsl.exe' -Arguments @('--install', '--no-distribution')
Invoke-Checked -Program 'wsl.exe' -Arguments @('--update')
Invoke-Checked -Program 'wsl.exe' -Arguments @('--set-default-version', '2')

$distroOutput = & wsl.exe --list --quiet
if ($LASTEXITCODE -ne 0) {
    throw 'Impossibile elencare le distro WSL. Riavviare Windows, poi rieseguire.'
}
$distros = @($distroOutput | ForEach-Object { ($_ -replace "`0", '').Trim() })
if ($distros -notcontains 'Ubuntu-20.04') {
    Invoke-Checked -Program 'wsl.exe' -Arguments @('--install', '--distribution', 'Ubuntu-20.04', '--no-launch')
}
Invoke-Checked -Program 'wsl.exe' -Arguments @('--set-version', 'Ubuntu-20.04', '2')

# Both the installation and the subsequent Desktop settings are visible to the user.
$dockerPackages = & winget.exe list --exact --id Docker.DockerDesktop --accept-source-agreements
if ($LASTEXITCODE -ne 0) {
    Invoke-Checked -Program 'winget.exe' -Arguments @(
        'install', '--exact', '--id', 'Docker.DockerDesktop', '--source', 'winget',
        '--accept-source-agreements'
    )
} else {
    Write-Host 'Docker Desktop gia installato:'
    $dockerPackages | Write-Host
}

Write-Host ''
Write-Host 'Aprire Ubuntu-20.04 con: wsl -d Ubuntu-20.04 e creare utente/password al primo avvio.'
Write-Host 'Aprire Docker Desktop dal menu Start e completare il primo avvio.'
Write-Host 'Settings > General: Use WSL 2 based engine (se visibile).'
Write-Host 'Settings > Resources > WSL Integration: abilitare Ubuntu-20.04, poi Apply.'
Write-Host 'Dalla shell Ubuntu seguire README.md: bash dev.sh build, up, build-workspace, shell.'
