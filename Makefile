CC = gcc
CFLAGS = -std=gnu99 -I.
LDFLAGS = -lm

libspaceturtle.so: libspaceturtle.o
	$(CC) $(CFLAGS) $(LDFLAGS) $< -shared -o $@

%.o: %.c libspaceturtle.h
	$(CC) $(CFLAGS) $(LDFLAGS) -c $< -o $@

test: libspaceturtle.o test.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@
	chmod 755 $@

clean:
	rm -f *.o *.so test
