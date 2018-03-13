all: memorybench

memorybench: memorybench.c libx64_static.a
	gcc memorybench.c libx64_static.a -o memorybench
	
libx64_static.a: libx64.asm
	fasm libx64.asm libx64.o
	ar rc libx64_static.a libx64.o
	ranlib libx64_static.a
	
clean:
	rm *.a *.o memorybench -f
