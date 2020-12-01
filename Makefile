CC=gcc
CFLAGS=
OBJ = example-receiver.o cc1101.o

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

example-receiver: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)
