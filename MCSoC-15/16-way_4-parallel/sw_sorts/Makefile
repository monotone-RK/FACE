# in Mega Elements (data type: integer)
SIZE=256

# random, sorted, reverse, xorshift
TYPE=xorshift

all: 
	gcc -static -O3 -Wall -W merge.c -o merge
	gcc -static -O3 -Wall -W quick.c -o quick
	gcc -static -O3 -Wall -W bitonic.c -o bitonic -lm

run: 
	./merge $(SIZE) $(TYPE)
	./quick $(SIZE) $(TYPE)
	./bitonic $(SIZE) $(TYPE)

clean:
	rm -f merge quick bitonic *_flymake*
