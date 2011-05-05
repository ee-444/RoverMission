@echo off

title Arduino Uploading
:start 
echo.
echo Upload to the Arduino Uno or Duemilanove.
echo  Integrates into the AVR Studio4 tools menu
echo  Enjoy ;b
echo. 
echo 1. Arduino Uno
echo 2. Arduino Duemilanove
echo.
:invalid_choice
set /p choice=enter your choice (1 or 2): 
if %choice%==1 goto uno
if %choice%==2 goto duemilanove
echo invalid choice: %choice%
goto invalid_choice
 
:duemilanove
echo.
echo Uploading hexfile to the Arduino Duemilanove:
echo.
avrdude -p m328p -c arduino -P \\.\COM%1 -b 57600 -U flash:w:%2
pause
exit

:uno
echo.
echo Uploading hexfile to the Arduino Uno:
echo.
avrdude -p m328p -c arduino -P \\.\COM%1 -b 115200 -U flash:w:%2
pause
exit