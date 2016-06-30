CC		= gcc
CFLAGS	= `libusb-config --cflags` -O -Wall
LIBS	= `libusb-config --libs` -lm

PROGRAM = templogger

all: $(PROGRAM)

.c.o:
	$(CC) $(CFLAGS) -c $<

$(PROGRAM): $(PROGRAM).o
	$(CC) -o $(PROGRAM) $(PROGRAM).o $(LIBS)

clean:
	rm -f *.o $(PROGRAM)
