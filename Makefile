CC := g++
SRC := $(wildcard *.c)
CFLAGS := -O1 -Wall -Wextra -Werror -std=c++11 -iquote ../RobotDriver/src/ -iquote ../DijkstraContest/SylvainLR/headers/
LDFLAGS := -L ../DijkstraContest/SylvainLR/ -lpath
.PHONY = clean

main : $(SRC:%.c=%.o)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

clean:
	rm -f $(SRC:%.c=%.o)
	rm -f main
