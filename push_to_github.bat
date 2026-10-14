@echo off
set "PATH=%LOCALAPPDATA%\Programs\Git\cmd;%PATH%"
echo Pushing to GitHub (https://github.com/maheshnatham/AEV_LowLevel_Controller)...
git push -u origin main
pause
