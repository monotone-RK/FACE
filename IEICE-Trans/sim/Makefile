############# simulation parameter #############

LAST_PHASE=7

# of ways per a tree (log_scale)
WAY_LOG=3

# of trees (log_scale)
P_LOG=3

# simulation type 
# - 0: evaluate 2x compression ratio
# - 1: evaluate 3x compression ratio
# - 2: display merged data in a phase
SIM=0

# data initialization type 
# - random, sorted, reverse, xorshift
TYPE=xorshift

all: 
	gcc -static -O3 -Wall -W main.c -o sim

run: 
	./sim $(LAST_PHASE) $(WAY_LOG) $(P_LOG) $(SIM) $(TYPE)

clean:
	rm -f sim *_flymake*
