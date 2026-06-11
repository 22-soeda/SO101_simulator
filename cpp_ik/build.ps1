# Build script for the C++ IK extension (so101_ik_cpp).
#
# Usage:
#   .\venv\Scripts\Activate.ps1
#   .\cpp_ik\build.ps1
#
# Requires the MSVC toolset from Visual Studio, plus
# `pip install pybind11 setuptools` in the venv.

$ErrorActionPreference = "Stop"

# Locate vcvars64.bat from an installed Visual Studio
$vcvars = Get-ChildItem "C:\Program Files\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars64.bat" -ErrorAction SilentlyContinue |
    Select-Object -First 1 -ExpandProperty FullName

if (-not $vcvars) {
    throw "vcvars64.bat not found (is the Visual Studio C++ build tools workload installed?)"
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = Join-Path $scriptDir "..\venv\Scripts\python.exe"

Push-Location $scriptDir
try {
    cmd /c "call `"$vcvars`" && set DISTUTILS_USE_SDK=1 && set MSSdk=1 && `"$python`" setup.py build_ext --inplace"
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
    Write-Host "Build succeeded: cpp_ik\so101_ik_cpp*.pyd"
} finally {
    Pop-Location
}
