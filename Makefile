CC := gcc
CFLAGS := -std=gnu99 -I. -fPIC
LDFLAGS := -lm
MAJOR := 0
MINOR := 1
VERSION := $(MAJOR).$(MINOR)
PREFIX := /usr

library: libspaceturtle.so.$(VERSION)

install: library
	cp libspaceturtle.so.$(VERSION) $(PREFIX)/lib/
	cp libspaceturtle.h $(PREFIX)/include/
	ldconfig -n $(PREFIX)/lib/
	rm -f $(PREFIX)/lib/libspaceturtle.so
	ln -s $(PREFIX)/lib/libspaceturtle.so.$(MAJOR) $(PREFIX)/lib/libspaceturtle.so

%.o: %.c libspaceturtle.h
	$(CC) $(CFLAGS) $(LDFLAGS) -c $< -o $@

test: libspaceturtle.o test.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@
	chmod 755 $@

libspaceturtle.so.$(VERSION): libspaceturtle.o
	$(CC) $(CFLAGS) $(LDFLAGS) $< -shared -Wl,-soname,libspaceturtle.so.$(MAJOR) -o $@

clean:
	rm -f *.o *.so.* *.so test

uninstall:
	rm -f $(PREFIX)/lib/libspaceturtle.so*
	rm -f $(PREFIX)/include/libspaceturtle.h
	ldconfig -n $(PREFIX)/lib/
