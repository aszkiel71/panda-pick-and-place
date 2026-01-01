#!/bin/bash
echo "[INFO] Creating virtual environment..."
python3 -m venv venv

echo "[INFO] Installing dependencies..."
source venv/bin/activate
pip install -r requirements.txt

echo "[SUCCESS] Setup complete! You can now run './run.sh'."
