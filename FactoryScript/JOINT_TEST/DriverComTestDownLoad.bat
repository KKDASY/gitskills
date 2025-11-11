@echo off
rem =============================
rem STM32 firmware programming batch script
rem Instructions:
rem 1. Please add the path of STM32CubeProgrammer_CLI.exe to the system environment variable, or replace the command below with the full path
rem 2. Modify FIRMWARE_PATH to your firmware file path (such as hex or bin file)
rem 3. Modify PORT to your download port (such as USB, COM, etc.)
rem 4. Double-click this script to automatically program
rem =============================

set BOOT_PATH=JOINT_BLDR_v1.1.0-R.hex
set APP_PATH=JOINT_APP_TESTED.hex


rem Programming command example (adjust parameters as needed)
STM32_Programmer_CLI.exe -c port=SWD freq=4000 -ob nSWBOOT0=0 -w %BOOT_PATH% --verify --start
STM32_Programmer_CLI.exe -c port=SWD freq=4000  -w %APP_PATH% --verify --start

pause