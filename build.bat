@echo off
echo ========================================
echo   Building LCD_PID Project...
echo ========================================

C:\Keil_v5\UV4\UV4.exe -b "D:\Embeddedpractice\PID_practice\pid_screen\LCD_PID\MDK-ARM\LCD.uvprojx" -o "D:\Embeddedpractice\PID_practice\pid_screen\LCD_PID\MDK-ARM\build.log"

echo.
echo ========================================
echo   Build Output:
echo ========================================
type "D:\Embeddedpractice\PID_practice\pid_screen\LCD_PID\MDK-ARM\build.log"

echo.
if %ERRORLEVEL% equ 0 (
    echo BUILD SUCCESS!
) else (
    echo BUILD FAILED! Error code: %ERRORLEVEL%
)

pause
