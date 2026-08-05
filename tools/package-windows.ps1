[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

function Invoke-Native {
  if ($args.Count -eq 0) {
    throw "native command is required"
  }
  $nativeCommand = [string]$args[0]
  $nativeArguments = if ($args.Count -gt 1) {
    @($args[1..($args.Count - 1)])
  } else {
    @()
  }
  & $nativeCommand @nativeArguments
  if ($LASTEXITCODE -ne 0) {
    throw "command failed ($LASTEXITCODE): $nativeCommand $($nativeArguments -join ' ')"
  }
}

function Install-SoftwareOpenGL {
  param(
    [Parameter(Mandatory = $true)]
    [string]$Destination
  )

  # GitHub-hosted Windows runners expose only the Microsoft OpenGL 1.1
  # implementation. Pin Mesa llvmpipe for renderer tests and packaged smoke;
  # these files are injected into test directories, not shipped in the ZIP.
  $mesaVersion = "26.1.6"
  $mesaSha256 = `
    "86b506ad38b8dae9d37bdade656a9003518d717bf4ff5475ff3f746e4ee768eb"
  $archive = Join-Path $env:RUNNER_TEMP "mesa3d-$mesaVersion-release-msvc.7z"
  $extractDirectory = Join-Path $env:RUNNER_TEMP "mesa3d-$mesaVersion-msvc"
  if (-not (Test-Path $archive -PathType Leaf)) {
    $url = "https://github.com/pal1000/mesa-dist-win/releases/download/" + `
      "$mesaVersion/mesa3d-$mesaVersion-release-msvc.7z"
    Invoke-WebRequest -Uri $url -OutFile $archive
  }
  $actualHash = (Get-FileHash -Algorithm SHA256 $archive).Hash.ToLowerInvariant()
  if ($actualHash -ne $mesaSha256) {
    throw "Mesa archive checksum mismatch: $actualHash"
  }
  if (-not (Test-Path (Join-Path $extractDirectory "x64/opengl32.dll"))) {
    Invoke-Native 7z x $archive "x64/opengl32.dll" `
      "x64/libgallium_wgl.dll" "-o$extractDirectory" -y
  }
  New-Item -ItemType Directory -Force $Destination | Out-Null
  Copy-Item (Join-Path $extractDirectory "x64/opengl32.dll") $Destination -Force
  Copy-Item (Join-Path $extractDirectory "x64/libgallium_wgl.dll") `
    $Destination -Force
  foreach ($executable in Get-ChildItem $Destination -Filter "*.exe" -File) {
    New-Item -ItemType File -Force "$($executable.FullName).local" | Out-Null
  }
  $env:GALLIUM_DRIVER = "llvmpipe"
}

function Remove-SoftwareOpenGL {
  param(
    [Parameter(Mandatory = $true)]
    [string]$Destination
  )
  Remove-Item (Join-Path $Destination "opengl32.dll") -Force `
    -ErrorAction SilentlyContinue
  Remove-Item (Join-Path $Destination "libgallium_wgl.dll") -Force `
    -ErrorAction SilentlyContinue
  Get-ChildItem $Destination -Filter "*.exe.local" -File `
    -ErrorAction SilentlyContinue | Remove-Item -Force
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
  if ($env:KPT_PACKAGE_VERIFY -and $env:KPT_PACKAGE_VERIFY -notin @("0", "1")) {
    throw "KPT_PACKAGE_VERIFY must be 0 or 1"
  }
  $packageVerify = $env:KPT_PACKAGE_VERIFY -ne "0"
  $buildTests = if ($packageVerify) { "ON" } else { "OFF" }
  Invoke-Native cmake --preset $preset -DKPT_ENABLE_PACKAGING=ON `
    "-DKPT_BUILD_TESTS=$buildTests"
  Invoke-Native cmake --build --preset $preset --parallel
  $useSoftwareOpenGL = $packageVerify -and `
    $env:KPT_WINDOWS_SOFTWARE_OPENGL -eq "1"
  $runtimeDirectory = Join-Path $buildDirectory "Release"
  if ($useSoftwareOpenGL) {
    Install-SoftwareOpenGL $runtimeDirectory
  }
  if ($packageVerify) {
    Invoke-Native ctest --preset $preset
  }
  if ($useSoftwareOpenGL) {
    Remove-SoftwareOpenGL $runtimeDirectory
  }
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
    if ($packageVerify) {
      Invoke-Native $executables[$commandName] --help
    }
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
  if ($packageVerify) {
    if ($useSoftwareOpenGL) {
      Install-SoftwareOpenGL (Split-Path -Parent $executables["pc_gui"])
    }
    Invoke-Native $executables["pc_gui"] --smoke-test
  }
  if ($packageVerify) {
    Write-Host "Built and verified $package"
  } else {
    Write-Host "Built and audited $package"
  }
}
finally {
  Pop-Location
}
