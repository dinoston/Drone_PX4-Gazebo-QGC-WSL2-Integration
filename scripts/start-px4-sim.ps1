$ErrorActionPreference = 'Stop'
$LinuxUser = 'shinyongsuk' # Change this to your Ubuntu user name.
$DistroName = 'Ubuntu-24.04-E'
$Px4Path = "/home/$LinuxUser/PX4-Autopilot"

$Host.UI.RawUI.WindowTitle = 'PX4 + Gazebo Simulation'
wsl.exe -d $DistroName --cd $Px4Path -- env `
    QT_QPA_PLATFORM=xcb `
    LIBGL_ALWAYS_SOFTWARE=1 `
    MESA_LOADER_DRIVER_OVERRIDE=llvmpipe `
    make px4_sitl gz_x500

Write-Host 'Simulation stopped. Press Enter to close.'
Read-Host

