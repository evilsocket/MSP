CC=g++
CFLAGS= `Magick-config --cppflags` -w -O3 -funroll-loops -fno-rtti -fomit-frame-pointer -ffast-math -fno-stack-protector -ffunction-sections
LFLAGS= `Magick-config --ldflags --libs`

fingerprint: examples/fingerprint.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
ai: examples/ai.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
motiondetection: examples/motiondetection.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
gui: examples/gui.cpp
	$(CC) $(CFLAGS) $? $(LFLAGS) -o $@
	
#debug:
#	$(CC) examples/motiondetection.cpp -o motiondetection -g3 $(LFLAGS)
#profile:
#	$(CC) main.cc -o msp -g3 -pg $(LFLAGS)
clean:
	rm fingerprint ai motiondetection gui
 
