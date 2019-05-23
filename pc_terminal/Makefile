
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal 
OBJFILES = pc_terminal.o pc_t20.o termIO.o rs232.o
TARGET = pc-terminal
LDFLAGS = -lpthread

all: $(TARGET) 

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

clean:
	rm -f $(OBJFILES) $(TARGET) 

run: all
	$(EXEC)

