@echo off
echo [INFO] Starting the Robot Simulation...
call venv\Scripts\activate

python main.py

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Execution failed. Please check the logs above.
    pause
)
