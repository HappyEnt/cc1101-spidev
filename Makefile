CC=gcc
CFLAGS=
OBJ = cc1101.o

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

all: example-receiver example-transmitter

example-receiver: example-receiver.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

example-transmitter: example-transmitter.o $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)
