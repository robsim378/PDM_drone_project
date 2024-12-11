@echo off

:: Check if the Conda environment is "drones"
if "%CONDA_DEFAULT_ENV%" NEQ "drones" (
    echo You must have the drones environment from gym-pybullet-drones sourced first.
    exit /b 1
)

:: Set PYTHONPATH to the script's directory
for %%I in ("%~dp0.") do set "PYTHONPATH=%%~fI"

:: Install the required Python packages
pip install -r "%PYTHONPATH%\requirements.txt"
