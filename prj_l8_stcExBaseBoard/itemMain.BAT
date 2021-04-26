SET C51INC=F:\KEIL\Software\C51\Inc;F:\KEIL\Software\C51\Inc\STC
SET C51LIB=F:\KEIL\Software\C51\Lib
SET CPU_TYPE=STC8G1K08 Series
SET CPU_VENDOR=STC
SET UV2_TARGET=itemMain
SET CPU_XTAL=0x02160EC0
"F:\KEIL\Software\C51\BIN\C51.EXE" @".\Objects\usrMain.__i"
"F:\KEIL\Software\C51\BIN\LX51.EXE" @".\Objects\PRJ_L8~1.LNP"
"F:\KEIL\Software\C51\BIN\OHX51.EXE" ".\Objects\prj_l8_stcExBaseBoard" HEX
