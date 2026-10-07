@echo off
setlocal
cd /d "%~dp0"

set "PROJECT_NAME=pcileech_100t484_x1"
set "PROJECT_TCL=vivado_generate_project_captaindma_100t.tcl"
set "VIVADO_EXE="

for /f "delims=" %%V in ('where.exe vivado.bat 2^>nul') do if not defined VIVADO_EXE set "VIVADO_EXE=%%~fV"
for /f "delims=" %%V in ('where.exe vivado.exe 2^>nul') do if not defined VIVADO_EXE set "VIVADO_EXE=%%~fV"

if not defined VIVADO_EXE for %%V in (
    "D:\Xilinx\Vivado\2024.2\bin\vivado.bat"
    "D:\vivado\Vivado\2024.2\bin\vivado.bat"
    "E:\Xilinx\Vivado\2024.2\bin\vivado.bat"
    "C:\Xilinx\Vivado\2024.2\bin\vivado.bat"
    "C:\Xilinx\Vivado\2023.2\bin\vivado.bat"
) do if not defined VIVADO_EXE if exist "%%~fV" set "VIVADO_EXE=%%~fV"

if not defined VIVADO_EXE (
    echo ERROR: Vivado was not found in PATH or the supported fallback locations.
    exit /b 1
)

call "%VIVADO_EXE%" -mode batch -source "%PROJECT_TCL%" -notrace -log "%PROJECT_NAME%_generate.log" -journal "%PROJECT_NAME%_generate.jou" -tclargs --project_name "%PROJECT_NAME%"
set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" (
    echo ERROR: 100T project generation failed with exit code %EXIT_CODE%.
    exit /b %EXIT_CODE%
)

echo [T11] apply pcie_7x identity overlay / patch
call "%VIVADO_EXE%" -mode batch -source "tools\pcie_core_patch.tcl" -notrace -log "pcie_core_patch_100t.log" -journal "pcie_core_patch_100t.jou" -tclargs --project_name "%PROJECT_NAME%"
set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" echo ERROR: T11 pcie_core_patch failed with exit code %EXIT_CODE%.
exit /b %EXIT_CODE%
