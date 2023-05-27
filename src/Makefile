EXECS  = retrogamevh
CFLAGS = -Wall -Ofast -fomit-frame-pointer -funroll-loops -s \
 -I/opt/vc/include \
 -I/opt/vc/include/interface/vcos/pthreads \
 -I/opt/vc/include/interface/vmcs_host \
 -I/opt/vc/include/interface/vmcs_host/linux \
 -L/opt/vc/lib
LIBS   = -lbcm_host
CC     = gcc $(CFLAGS)

all: $(EXECS)

retrogamevh: retrogamevh.c keyTable.h
	$(CC) $< $(LIBS) -o $@
	strip $@

# KEYFILE = /usr/include/linux/input.h
KEYFILE = /usr/include/linux/input-event-codes.h
keyTable.h: keyTableGen.sh $(KEYFILE)
	sh $^ >$@

install:
	mv $(EXECS) /usr/local/bin

clean:
	rm -f $(EXECS) keyTable.h
