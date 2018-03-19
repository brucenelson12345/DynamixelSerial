# build an executable for UsbAX

CC = g++
CFLAGS = -g -Wall -I../../include

HDIR = ../../include
SDIR = ../../src
ODIR = ../../src/obj

UAX = UsbAX
TARGET = moveDegSpeed

all: $(TARGET)

$(TARGET): $(TARGET).o $(UAX).o
	$(CC) $(CFLAGS) $< $(ODIR)/$(UAX).o -o $@
		
$(TARGET).o: $(TARGET).cpp
	$(CC) $(CFLAGS) -c $< -o $@
	
$(UAX).o: $(SDIR)/$(UAX).cpp $(HDIR)/$(UAX).h
	$(CC) $(CFLAGS) -c $< -o $(ODIR)/$@


target: $(TARGET)

$(TARGET): $(TARGET).o $(UAX).o
	$(CC) $(CFLAGS) $< $(ODIR)/$(UAX).o -o $@

clean:
	$(RM) -f core *.o $(TARGET)

cleanall:
		$(RM) -f core *.o $(ODIR)/*.o $(TARGET) $(SDIR)/*.cpp~ *.cpp~ $(HDIR)/*.h~ 
