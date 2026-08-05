[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

function Invoke-Native {
  param(
    [Parameter(Mandatory = $true)]
    [string]$Command,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Arguments
  )
  & $Command @Arguments
  if ($LASTEXITCODE -ne 0) {
    throw "command failed ($LASTEXITCODE): $Command $($Arguments -join ' ')"
  }
}

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$version = (Get-Content (Join-Path $repositoryRoot "VERSION") -Raw).Trim()
if ($version -notmatch '^\d+\.\d+\.\d+$') {
  throw "VERSION is not an X.Y.Z semantic version: $version"
}
if (-not $env:VCPKG_ROOT) {
  throw "VCPKG_ROOT is required"
}

Push-Location $repositoryRoot
try {
  Invoke-Native bash "./tools/check-version.sh" $version
  $preset = "windows-x64-vcpkg-release"
  $buildDirectory = Join-Path $repositoryRoot "build/$preset"
  $artifactDirectory = Join-Path $repositoryRoot "artifacts"
  Invoke-Native cmake --preset $preset -DKPT_ENABLE_PACKAGING=ON
  Invoke-Native cmake --build --preset $preset --parallel
  Invoke-Native ctest --preset $preset
  New-Item -ItemType Directory -Force $artifactDirectory | Out-Null
  Invoke-Native cpack --config "$buildDirectory/CPackConfig.cmake" `
    -C Release -B $artifactDirectory

  $package = Join-Path $artifactDirectory `
    "kitti-pointcloud-tools-$version-windows-x64.zip"
  if (-not (Test-Path $package -PathType Leaf) -or
      (Get-Item $package).Length -eq 0) {
    throw "Windows package was not created: $package"
  }

  $inspectionDirectory = Join-Path $buildDirectory "package-inspection"
  Invoke-Native cmake -E remove_directory $inspectionDirectory
  New-Item -ItemType Directory -Force $inspectionDirectory | Out-Null
  Expand-Archive -LiteralPath $package -DestinationPath $inspectionDirectory

  $commands = @(
    "pc_gui", "pc_viewer", "pc_player", "pc_convert",
    "pc_batch_convert", "pc_render"
  )
  $executables = @{}
  foreach ($commandName in $commands) {
    $matches = @(Get-ChildItem $inspectionDirectory -Recurse -File `
      -Filter "$commandName.exe")
    if ($matches.Count -ne 1) {
      throw "expected one $commandName.exe in package, found $($matches.Count)"
    }
    $executables[$commandName] = $matches[0].FullName
  }

  $packagedVersions = @(Get-ChildItem $inspectionDirectory -Recurse -File `
    -Filter VERSION)
  if ($packagedVersions.Count -ne 1 -or
      (Get-Content $packagedVersions[0].FullName -Raw).Trim() -ne $version) {
    throw "packaged VERSION does not match $version"
  }

  $vswhere = Join-Path ${env:ProgramFiles(x86)} `
    "Microsoft Visual Studio/Installer/vswhere.exe"
  $dumpbin = & $vswhere -latest -products * `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
    -find "VC/Tools/MSVC/*/bin/Hostx64/x64/dumpbin.exe" |
    Select-Object -First 1
  if (-not $dumpbin) {
    throw "dumpbin.exe was not found"
  }

  foreach ($commandName in $commands) {
    Invoke-Native $executables[$commandName] --help
    $dependencies = & $dumpbin /dependents $executables[$commandName] |
      Out-String
    if ($LASTEXITCODE -ne 0) {
      throw "dumpbin failed for $commandName.exe"
    }
    if ($dependencies -match `
        '(?im)^\s*(?:vcruntime.*d|msvcp.*d|ucrtbased)\.dll\s*$') {
      throw "debug runtime dependency found in $commandName.exe"
    }
  }
  Invoke-Native $executables["pc_gui"] --smoke-test
  Write-Host "Built and verified $package"
}
finally {
  Pop-Location
}
