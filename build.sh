set -e

gcc -O2 -DREPLACEMENT_POLICY=0 -o sim_lru    Simulator.c
gcc -O2 -DREPLACEMENT_POLICY=1 -o sim_random Simulator.c
./sim_lru    stress.tko
./sim_random stress.tko