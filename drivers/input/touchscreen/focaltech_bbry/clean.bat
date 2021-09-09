@echo on
::for /r %%f in (*.o.cmd) do attrib -h %%f
::for /r %%f in (*.o.cmd) do del /ah %%f
del /ah *.cmd
del /ah focaltech_test\*.cmd
del /ah focaltech_global\*.cmd
del /ah focaltech_flash\*.cmd
for /r %%f in (*.o) do del %%f
for /r %%f in (*.order) do del %%f
for /r %%f in (*.c~) do del %%f
for /r %%f in (*.h~) do del %%f
pause