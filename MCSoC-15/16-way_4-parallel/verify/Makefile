# in Mega Elements (data type: integer)
SIZE=256

# random, sorted, reverse, xorshift
TYPE=xorshift

all: 
	gcc -static -O3 -Wall -W merge.c -o merge

run: 
	./merge $(SIZE) $(TYPE)

dump:
	./merge $(SIZE) $(TYPE) > good.txt

diff:
	python verify.py
	diff -s good.txt test.txt

clean:
	rm -f merge quick *_flymake* bubble insertion selection *.txt teraterm.log
