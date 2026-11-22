param([Parameter(Mandatory)][string]$BuildDir, [string]$Iscc)
$ErrorActionPreference = 'Stop'
$build = (Resolve-Path $BuildDir).Path
$cache = Get-Content "$build/CMakeCache.txt"
function CacheValue($name) {
    $entry = $cache | Where-Object { $_ -match "^${name}:[^=]+=" }
    if (-not $entry) { throw "Missing CMake cache entry $name" }
    return ($entry -split '=',2)[1]
}
function Checked($exe, [string[]]$arguments) {
    & $exe @arguments
    if ($LASTEXITCODE) { throw "$exe exited $LASTEXITCODE" }
}
if ((CacheValue CMAKE_BUILD_TYPE) -ne 'Release') { throw 'Configure a Release build using Ninja.' }
$qt = (Resolve-Path "$(CacheValue Qt6_DIR)/../../..").Path
$compiler = CacheValue CMAKE_CXX_COMPILER
$env:PATH = "$qt/bin;$(Split-Path $compiler);$env:PATH"
if ((& $compiler -dumpmachine) -ne 'x86_64-w64-mingw32' -or (& $compiler -dumpfullversion) -ne '13.1.0') {
    throw 'Expected MinGW-w64 x64 GCC 13.1.0.'
}
if ((& "$qt/bin/qmake.exe" -query QT_VERSION) -ne '6.9.3') { throw 'Expected Qt 6.9.3.' }
$stage = (New-Item -ItemType Directory "$build/stage-$([guid]::NewGuid().ToString('N'))").FullName
Checked (CacheValue CMAKE_COMMAND) @('--install',$build,'--prefix',$stage)
Checked "$qt/bin/windeployqt.exe" @('--release','--compiler-runtime','--no-translations','--no-system-d3d-compiler',
    '--no-system-dxc-compiler','--no-opengl-sw','--skip-plugin-types','imageformats,iconengines,generic,networkinformation,tls',"$stage/EasyRider.exe")
foreach ($file in 'Qt6Core.dll','Qt6Gui.dll','Qt6Widgets.dll','platforms/qwindows.dll','libgcc_s_seh-1.dll','libstdc++-6.dll','libwinpthread-1.dll') {
    if (-not (Test-Path "$stage/$file" -PathType Leaf)) { throw "Deployment missing $file" }
}
Copy-Item -Recurse "$PSScriptRoot/licenses" "$stage/licenses"
Copy-Item "$qt/sbom/qtbase-6.9.3.spdx" "$stage/licenses/"
if (-not $Iscc) {
    $Iscc = @("$env:LOCALAPPDATA/Programs/Inno Setup 6/ISCC.exe","${env:ProgramFiles(x86)}/Inno Setup 6/ISCC.exe") |
        Where-Object { Test-Path $_ } | Select-Object -First 1
}
if (-not $Iscc) { throw 'Install Inno Setup 6 or pass -Iscc.' }
# Distribute the matching LGPL library source alongside the installer.
$output = (New-Item -ItemType Directory -Force "$build/installer").FullName
$archive = 'qtbase-everywhere-src-6.9.3.tar.xz'
$expected = 'c5a1a2f660356ec081febfa782998ae5ddbc5925117e64f50e4be9cd45b8dc6e'
if (-not (Test-Path "$output/$archive") -or (Get-FileHash "$output/$archive").Hash -ne $expected) {
    $download = "$stage-qtbase.tar.xz"
    Checked curl.exe @('-fLsS','--retry','3','--output',$download,"https://download.qt.io/archive/qt/6.9/6.9.3/submodules/$archive")
    if ((Get-FileHash $download).Hash -ne $expected) { throw 'Qt Base source checksum mismatch' }
    Move-Item -Force $download "$output/$archive"
}
Checked $Iscc @("/DStage=$stage","/O$build/installer","$build/EasyRider.iss")
