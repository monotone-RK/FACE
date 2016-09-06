# in Mega Elements (data type: integer)
SIZE=256

# random, sorted, reverse, xorshift
TYPE=xorshift

all: 
	gcc -static -Ofast -Wall -W merge.c -o merge
	gcc -static -Ofast -Wall -W quick.c -o quick

run: 
	./merge $(SIZE) $(TYPE)
	./merge $(SIZE) $(TYPE)
	./merge $(SIZE) $(TYPE)
	./merge $(SIZE) $(TYPE)
	./merge $(SIZE) $(TYPE)
	./quick $(SIZE) $(TYPE)
	./quick $(SIZE) $(TYPE)
	./quick $(SIZE) $(TYPE)
	./quick $(SIZE) $(TYPE)
	./quick $(SIZE) $(TYPE)

clean:
	rm -f merge quick *_flymake*
