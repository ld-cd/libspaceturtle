NAME := spaceturtle
CC := gcc
CFLAGS := -std=gnu99 -I. -fPIC
LDFLAGS := -lm
MAJOR := 0
MINOR := 1
VERSION := $(MAJOR).$(MINOR)
PREFIX := /usr

library: lib$(NAME).so.$(VERSION)

install: library
	cp lib$(NAME).so.$(VERSION) $(PREFIX)/lib/
	cp lib$(NAME).h $(PREFIX)/include/
	ldconfig -n $(PREFIX)/lib/
	rm -f $(PREFIX)/lib/lib$(NAME).so
	ln -s $(PREFIX)/lib/lib$(NAME).so.$(MAJOR) $(PREFIX)/lib/lib$(NAME).so

%.o: %.c lib$(NAME).h
	$(CC) $(CFLAGS) $(LDFLAGS) -c $< -o $@

test: lib$(NAME).o test.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@
	chmod 755 $@

lib$(NAME).so.$(VERSION): lib$(NAME).o
	$(CC) $(CFLAGS) $(LDFLAGS) $< -shared -Wl,-soname,lib$(NAME).so.$(MAJOR) -o $@

clean:
	rm -f *.o *.so.* *.so test

uninstall:
	rm -f $(PREFIX)/lib/lib$(NAME).so*
	rm -f $(PREFIX)/include/lib$(NAME).h
	ldconfig -n $(PREFIX)/lib/
