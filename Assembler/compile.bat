sed -f compile.txt src.asm > base.tmp
sed -n '1p' base.tmp | sed 's/.*8\'d0:\(.*\)/\t8\'d0:\t\1/' > dest.asm
sed '1,1d' base.tmp > t1.tmp
sed '=' t1.tmp > t2.tmp
sed 'N;s/\([0-9]*\)\n.*8\'d0:\(.*\)/\t8\'d\1:\t\2/' t2.tmp >> dest.asm
del *.tmp