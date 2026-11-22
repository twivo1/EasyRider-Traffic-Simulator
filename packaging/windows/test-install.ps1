param([Parameter(Mandatory)][string]$Setup, [string]$LogDir = "build/installer-test")
$ErrorActionPreference = 'Stop'
$setupPath = (Resolve-Path $Setup).Path
$logs = (New-Item -ItemType Directory -Force $LogDir).FullName
$id = [guid]::NewGuid().ToString('N')
$dir = Join-Path $env:TEMP "EasyRider test $id\Zażółć gęślą"
$group = "EasyRider"
$link = Join-Path ([Environment]::GetFolderPath('Programs')) "$group\EasyRider.lnk"
$reg = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\twivo1.EasyRider.TrafficSimulator_is1'
if ((Test-Path $reg) -or (Test-Path $link)) { throw 'An EasyRider installation is already registered. Use a disposable Windows profile.' }
function WaitSuccess($proc, $label) {
    $null = $proc.Handle # Retain the native process handle for ExitCode on Windows PowerShell.
    if (-not $proc.WaitForExit(60000)) { $proc.Kill(); throw "$label timed out" }
    if ($proc.ExitCode -ne 0) { throw "$label exited $($proc.ExitCode); logs: $logs" }
}
$previousPath = $env:Path
$qtEnv = @(Get-ChildItem Env: | Where-Object { $_.Name -match '^(QT_|QML)' })
$gui = $null
try {
    foreach ($v in $qtEnv) { [Environment]::SetEnvironmentVariable($v.Name,$null,'Process') }
    $env:Path = "$env:SystemRoot\System32;$env:SystemRoot"
    $env:QT_DEBUG_PLUGINS = '1'
    $env:QT_FORCE_STDERR_LOGGING = '1'
    $argsSetup = @('/VERYSILENT','/SUPPRESSMSGBOXES','/NORESTART','/CURRENTUSER',"/DIR=`"$dir`"","/LOG=`"$logs/install.log`"")
    WaitSuccess (Start-Process $setupPath -ArgumentList $argsSetup -PassThru) 'Install'
    $exe = Join-Path $dir 'EasyRider.exe'
    foreach ($f in 'EasyRider.exe','platforms/qwindows.dll','Qt6Core.dll','Qt6Gui.dll','Qt6Widgets.dll','libgcc_s_seh-1.dll','libwinpthread-1.dll','libstdc++-6.dll','unins000.exe') {
        if (-not (Test-Path "$dir/$f" -PathType Leaf)) { throw "Missing installed file $f" }
    }
    Get-ChildItem -Recurse $dir -File | Select-Object FullName,Length | Out-File "$logs/inventory.txt" -Width 512
    WaitSuccess (Start-Process $exe -WorkingDirectory $dir -ArgumentList '--installation-check' -PassThru -RedirectStandardError "$logs/qt.log" -RedirectStandardOutput "$logs/stdout.log") 'Window and resource check'
    $shell = New-Object -ComObject Shell.Application
    $item = $shell.NameSpace((Split-Path $link)).ParseName('EasyRider.lnk')
    if (-not $item -or $item.GetLink.Path -ne $exe) { throw "Invalid Start menu shortcut $link" }
    Start-Process -FilePath $link
    $deadline = [datetime]::UtcNow.AddSeconds(30)
    do {
        Start-Sleep -Milliseconds 300
        $gui = Get-Process EasyRider -ErrorAction SilentlyContinue | Where-Object { $_.Path -eq $exe } | Select-Object -First 1
    } until (($gui -and $gui.MainWindowHandle -ne 0) -or [datetime]::UtcNow -gt $deadline)
    if (-not $gui -or $gui.MainWindowHandle -eq 0) { throw 'Shortcut did not open a window' }
    Start-Sleep -Seconds 2
    $gui.Refresh()
    $modules = @($gui.Modules)
    $modules.FileName | Out-File "$logs/loaded-modules.txt"
    foreach ($name in 'Qt6Widgets.dll','Qt6Core.dll','Qt6Gui.dll','qwindows.dll','libgcc_s_seh-1.dll','libstdc++-6.dll','libwinpthread-1.dll') {
        $m = $modules | Where-Object ModuleName -eq $name
        if (-not $m -or -not $m.FileName.StartsWith("$dir\",[StringComparison]::OrdinalIgnoreCase)) { throw "Runtime outside installation or absent: $name" }
    }
    $null = $gui.Handle
    if (-not $gui.CloseMainWindow()) { throw 'Window rejected close request' }
    WaitSuccess $gui 'Normal shutdown'
} finally {
    if ($gui -and -not $gui.HasExited) { $gui.Kill(); $gui.WaitForExit() }
    $env:Path = $previousPath
    Remove-Item Env:QT_DEBUG_PLUGINS,Env:QT_FORCE_STDERR_LOGGING -ErrorAction SilentlyContinue
    foreach ($v in $qtEnv) { [Environment]::SetEnvironmentVariable($v.Name,$v.Value,'Process') }
    if (Test-Path "$dir/unins000.exe") {
        WaitSuccess (Start-Process "$dir/unins000.exe" -ArgumentList @('/VERYSILENT','/SUPPRESSMSGBOXES','/NORESTART',"/LOG=`"$logs/uninstall.log`"") -PassThru) 'Uninstall'
    }
}
Start-Sleep -Seconds 3
if ((Test-Path $dir) -or (Test-Path $link) -or (Test-Path $reg)) { throw 'Uninstall left application files, shortcut or registration' }
Write-Host "PASS: installed, loaded private DLLs and resources, opened via Start menu, closed and uninstalled. Logs: $logs"
