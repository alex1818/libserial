rm serial.o
rm libserial.so

gcc -c -Wall -fpic serial.c
gcc -shared -o libserial.so serial.o
