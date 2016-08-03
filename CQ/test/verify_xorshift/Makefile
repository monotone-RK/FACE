# in Kilo Elements (data type: integer)
SIZE=64

# random, sorted, reverse, xorshift
TYPE=xorshift

all: 
	gcc -static -O3 -Wall -W merge.c -o merge

run: 
	./merge $(SIZE) $(TYPE)

diff: 
	./merge $(SIZE) $(TYPE) > good.txt
	diff -s good.txt ../log.txt

clean:
	rm -f merge quick bitonic *_flymake* good.txt
