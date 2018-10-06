#before build,you should set the enviroment.
# cd /usr/local/Cavium-Networks/OCTEON-SDK/
# source env-setup OCTEON_CN66XX_PASS1_2


#CC = mips64-octeon-linux-gnu-gcc
CC = gcc
CPPFLAGS +=-D_REENTRANT -Wall -Wunused -std=gnu99

OBJS += testrecv.o   testsend.o
LIBS +=

all : testrecv testsend

$(OBJS):%.o : %.c
	$(CC)  $(CPPFLAGS)  -o $@ -c $<

testrecv: testrecv.o
	$(CC)  $(LDDFLAGS) -o $@  $^ $(LIBS)

testsend:testsend.o
	$(CC)  $(LDDFLAGS) -o $@  $^ $(LIBS)

clean:
	rm -f  *.o   testrecv testsend
	
