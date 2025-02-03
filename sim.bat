:: This script writes a robot ID value to a file named ROBOT_ID. The ID is used to pick a set of constant options
:: (e.g., gear ratios, motors) for the specific robot the code is deployed to.
:: The ID value is the sole argument to this script.

@echo off
set original_dir=%CD%
cd %~dp0\src
>ROBOT_ID echo %1
python -m robotpy sim
del ROBOT_ID
cd %original_dir%