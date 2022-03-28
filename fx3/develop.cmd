@echo off
title FX3 command tool by caowei
if not exist "C:\Program Files\Git\bin\bash.exe" (
	echo Install git tools first please!
	pause
	exit 1
)
set ARMGCC_INSTALL_PATH=E:\in_use\FX3\ARM_GCC
set PATH=%PATH%;%cd%\bin;%cd%\ARM_GCC\bin;%cd%\util\cyfwprog;%cd%\util\cyfwstorprog;%cd%\util\elf2img;C:\Program Files\Git\bin\;C:\Program Files\Notepad++
echo useful: 
echo     elf2img -i $(MODULE).elf -o $(MODULE).img -v
echo     FX3FWROOT=E:/in_use/FX3
cmd /k bash
exit 0