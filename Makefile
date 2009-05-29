CC=g++
CFLAGS= `Magick-config --cppflags` -w -O3 -funroll-loops -fno-rtti -fomit-frame-pointer -ffast-math -fno-stack-protector -ffunction-sections
LFLAGS= `Magick-config --ldflags --libs` -lX11 -lpthread

all: fingerprint ai motiondetection gui

fingerprint: examples/fingerprint.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
ai: examples/ai.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
motiondetection: examples/motiondetection.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
gui: examples/gui.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@

clean:
	rm -f fingerprint ai motiondetection gui
 
