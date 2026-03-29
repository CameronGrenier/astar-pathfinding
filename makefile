CC = clang
CFLAGS = -g -Wall -Wextra -fcolor-diagnostics -fansi-escape-codes -I./include
LDFLAGS = -lm -lpthread -lncurses

SRC = src/main.c src/map.c src/astar.c src/runner.c src/cli.c src/heap.c src/types.c src/menu.c
TARGET = astar

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)