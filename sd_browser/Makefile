all: browser browser_prg.h

browser:
	cl65 -t c64 -Oi screen.c browser.c dir.c base.c tapecartif.c minidelay.s getbyte_fast.s loader.s -o browser.prg

browser_prg.h: browser.prg
	xxd -i browser.prg > browser_prg.h

clean:
	rm -rf *.o browser.prg browser_prg.h
