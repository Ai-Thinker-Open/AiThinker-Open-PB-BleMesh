del .\*.uvguix.*
del .\*.uvoptx
del .\*.htm
del .\*.asm
rd /s/q Listings
for /f "tokens=*" %%i in ('dir/b/ad *.bin') do rd /s/q "%%i"
del Objects\*.o 
del Objects\*.d
del Objects\*.crf
del Objects\*.htm
del Objects\*.dep
del Objects\*.lnp
del _bld.txt