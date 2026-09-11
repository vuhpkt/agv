@echo off
echo [*] Dang khoi phuc file .git/index bi loi/trong...
if exist .git\index (
    del /f /q .git\index
)
git reset
git status
echo.
echo [+] Da khoi phuc xong .git/index! Ban co the tiep tuc lam viec binh thuong.

