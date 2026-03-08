@echo off
setlocal
set PARAMS=%~1
set OUT=%~2
if "%PARAMS%"=="" set PARAMS=params.json
if "%OUT%"=="" set OUT=frames

echo [MOCK CFD] Starting mock CFD runner...
echo [MOCK CFD] Params: %PARAMS%
echo [MOCK CFD] Output dir: %OUT%

if not exist "%OUT%" mkdir "%OUT%"

set NX=300
set NY=150
set DT=0.5
set FRAMES=10

for /L %%i in (0,1,%FRAMES%) do (
    set /a T=%%i*DT
    echo [MOCK CFD] Generating frame %%i (t=!T!)
    echo # Nx=%NX% Ny=%NY% t=!T! > "%OUT%\frame_%%i.csv"
    for /L %%y in (0,1,%NY%) do (
        set LINE=
        for /L %%x in (0,1,%NX%) do (
            set /A VAL=%%x+%%y
            set LINE=!LINE!,!VAL!
        )
        echo !LINE! >> "%OUT%\frame_%%i.csv"
    )
    timeout /t 1 /nobreak >nul
)

echo [MOCK CFD] Finished generating %FRAMES% frames
exit /b 0
