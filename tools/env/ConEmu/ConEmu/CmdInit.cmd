@rem !!! Do not change this file in-place, change its copy instead !!!
@rem !!!  Otherwise you will lose your settings after next update  !!!

@echo off

set PATH=%SystemRoot%\system32;%PATH%
set PATH=C:\WINDOWS\system32;%PATH%

rem Simple "ver" prints empty line before Windows version
rem Use this construction to print just a version info
cmd /d /c ver | "%windir%\system32\find.exe" "Windows"

rem Now we form the command prompt

rem This will start prompt with `User@PC `
set ConEmuPrompt0=$E[m$E[32m$E]9;8;"USERNAME"$E\@$E]9;8;"COMPUTERNAME"$E\$S

rem Followed by colored `Path`
set ConEmuPrompt1=%ConEmuPrompt0%$E[92m$P$E[90m
if NOT "%PROCESSOR_ARCHITECTURE%" == "AMD64" (
  if "%PROCESSOR_ARCHITEW6432%" == "AMD64" if "%PROCESSOR_ARCHITECTURE%" == "x86" (
    rem Use another text color if cmd was run from SysWow64
    set ConEmuPrompt1=%ConEmuPrompt0%$E[93m$P$E[90m
  )
)

rem Carriage return and `$` or `>`
rem Spare `$E[90m` was specially added because of GitShowBranch.cmd
if "%ConEmuIsAdmin%" == "ADMIN" (
  set ConEmuPrompt2=$_$E[90m$$
) else (
  set ConEmuPrompt2=$_$E[90m$G
)

rem Finally reset color and add space
set ConEmuPrompt3=$E[m$S$E]9;12$E\

:: ============= SBL ENV Path ==================

set ENV_ROOT=%~dp0..\..

:: set PYTHONPATH and PYTHONHOME 
if /i "%processor_architecture%"=="x86" (
        set PYTHONPATH=%ENV_ROOT%\python376_32
        set PYTHONHOME=%ENV_ROOT%\python376_32
) else if /i "%processor_architecture%"=="amd64" (
    if defined processor_architew6432 (
        set PYTHONPATH=%ENV_ROOT%\python376_32
        set PYTHONHOME=%ENV_ROOT%\python376_32
    ) else (
        set PYTHONPATH=%ENV_ROOT%\python376
        set PYTHONHOME=%ENV_ROOT%\python376
    )
)

set SCONS=%PYTHONPATH%\Scripts
set PATH=%ENV_ROOT%\bin;%PATH%
set PATH=%SBL_EXEC_PATH%;%PATH%
set PATH=%PYTHONHOME%;%PATH%
set PATH=%PYTHONPATH%;%PATH%
set PATH=%SCONS%;%PATH%

start /b python %~dp0..\..\scripts\init_env.py

:: ====== SBL ENV Change Code Page ================

chcp 65001 > nul
echo # NXP MCU SBL
echo #
echo # -  Secure connections for a smarter world
echo #
echo # 2020-2021 Copyright by NXP
echo # www.nxp.com 

Setlocal ENABLEDELAYEDEXPANSION
::启用命令扩展，参加setlocal /?命令
set str1=%~dp0
set str=%str1%
set env_root=%~dp0

:next
if not "%str%"=="" (
set /a num+=1
if "!str:~0,1!"==" " (
    echo.
    echo *******************************************************************
    echo Env 工具所在路径如下：
    echo %env_root:~0,-21%
    echo 警告：以上路径不能包含中文或空格，请将 Env 移动到符合要求的路径中。
    echo *******************************************************************
    endlocal
    goto break_str
)
set "str=%str:~1%"
goto next
)
endlocal

set "str=%~dp0"
>"%tmp%\t.t" echo;WSH.Echo(/[\u4E00-\u9FFF]/.test(WSH.Arguments(0)))
for /f %%a in ('cscript -nologo -e:jscript "%tmp%\t.t" "%str%"') do if %%a neq 0 (goto not_support_chinese) else goto break_str

:not_support_chinese
echo.
echo *******************************************************************
echo Env 工具所在路径如下：
echo %env_root:~0,-29%
echo 警告：以上路径不能包含中文或空格，请将 Env 移动到符合要求的路径中。
echo *******************************************************************

:break_str

chcp 437 > nul

:: ======================================================
if /I "%~1" == "/git" goto git
if /I "%~1" == "-git" goto git
goto no_git

:git
call "%~dp0GitShowBranch.cmd" /i
goto :EOF

:no_git
rem Set new prompt
PROMPT %ConEmuPrompt1%%ConEmuPrompt2%%ConEmuPrompt3%
