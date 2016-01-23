CC := g++
SRC := $(wildcard *.c)
CFLAGS := -O1 -Wall -Wextra -std=c++11 -iquote ../RobotDriver/src/ -iquote ../DijkstraContest/SylvainLR/headers/
LDFLAGS := -L../RobotDriver/src
.PHONY = clean

main : $(SRC:%.c=%.o)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(SRC:%.c=%.o)
	rm -f main
