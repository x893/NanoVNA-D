@ECHO OFF

SET PATH=E:\Tools\GNU\make\bin;%PATH%
SET PATH=E:\Tools\GNU\arm-none-eabi-13.2\bin;%PATH%
rem SET PATH=E:\Tools\GNU\arm-none-eabi\bin;%PATH%

make TARGET=F303 clean
make TARGET=F303
rem USE_VERBOSE_COMPILE=yes
