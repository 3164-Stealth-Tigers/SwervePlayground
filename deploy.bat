:: This script writes a robot ID value to a file named ROBOT_ID. The ID is used to pick a set of constant options
:: (e.g., gear ratios, motors) for the specific robot the code is deployed to.
:: The ID value is the sole argument to this script.

@echo off
echo %* > src\ROBOT_ID
python src\robot.py deploy
del src\ROBOT_ID