#before build,you should set the enviroment.
# cd /usr/local/Cavium-Networks/OCTEON-SDK/
# source env-setup OCTEON_CN66XX_PASS1_2


#CC = mips64-octeon-linux-gnu-gcc
CC = gcc
CPPFLAGS +=-D_REENTRANT -Wall -Wunused -std=gnu99

OBJS += testrecv.o   testsend.o testsend_4k.o
LIBS +=

all : testrecv testsend testsend_4k testwrite

$(OBJS):%.o : %.c
	$(CC)  $(CPPFLAGS)  -o $@ -c $<

testrecv: testrecv.o
	$(CC)  $(LDDFLAGS) -o $@  $^ $(LIBS)

testsend:testsend.o
	$(CC)  $(LDDFLAGS) -o $@  $^ $(LIBS)

testwrite:testwrite.o
	$(CC)  $(LDDFLAGS) -o $@  $^ $(LIBS)

testsend_4k:testsend_4k.o
	$(CC)  $(LDDFLAGS) -o $@  $^ $(LIBS)
clean:
	rm -f  *.o   testrecv testsend testesn_4k testwrite
	
