FLAGS =-lLimeSuite -lwiringPi -g -lm -O3 

all:
	gcc -o quad.bin ./quad.c $(FLAGS)
