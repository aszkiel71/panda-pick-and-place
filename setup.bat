@echo off
echo [INFO] Creating virtual environment (venv)...
python -m venv venv

echo [INFO] Activating environment...
call venv\Scripts\activate

echo [INFO] Installing dependencies from requirements.txt...
pip install -r requirements.txt

echo [SUCCESS] Setup complete! You can now run 'run.bat'.
pause
