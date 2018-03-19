# build an executable for JetsonAX12

CC = g++
CFLAGS = -g -Wall -I../../include

HDIR = ../../include
SDIR = ../../src
ODIR = ../../src/obj

LMX28 = JetsonAX12
LGPIO = jetsonGPIO

TARGET = readStats

all: $(TARGET)

$(TARGET): $(TARGET).o $(LMX28).o $(LGPIO).o
	$(CC) $(CFLAGS) $< $(ODIR)/$(LMX28).o $(ODIR)/$(LGPIO).o -o $@
		
$(TARGET).o: $(TARGET).cpp
	$(CC) $(CFLAGS) -c $< -o $@
	
$(LMX28).o: $(SDIR)/$(LMX28).cpp $(HDIR)/$(LMX28).h $(HDIR)/$(LGPIO).h
	$(CC) $(CFLAGS) -c $< -o $(ODIR)/$@

$(LGPIO).o: $(SDIR)/$(LGPIO).c	$(HDIR)/$(LGPIO).h
	$(CC) $(CFLAGS) -c $< -o $(ODIR)/$@


target: $(TARGET)

$(TARGET): $(TARGET).o $(LMX28).o $(LGPIO).o
	$(CC) $(CFLAGS) $< $(ODIR)/$(LMX28).o $(ODIR)/$(LGPIO).o -o $@

clean:
	$(RM) -f core *.o $(TARGET)

cleanall:
		$(RM) -f core *.o $(ODIR)/*.o $(TARGET) $(SDIR)/*.cpp~ *.cpp~ $(HDIR)/*.h~
