OBJS     = species.o ant.o environment.o matlab.o antsim.o
DEST     = antsim

CC       = g++
INCLUDES = 
CFLAGS   = -Wall -O2 -g
CLIBS    = -lm
all: Makefile $(OBJS)
	$(CC) -o $(DEST) $(OBJS)

%.o: Makefile %.c %.h
	$(CC) -c -o $(*F).o $(CFLAGS) $(INCLUDES) $(*F).c

%.o: Makefile %.s
	$(CC) -c -o $(*F).o $(CFLAGS) $(INCLUDES) $(*F).s

%.o: Makefile %.S
	$(CC) -c -o $(*F).o $(CFLAGS) $(INCLUDES) $(*F).S

clean:
	rm -rf $(OBJS) $(DEST) antsim.o
