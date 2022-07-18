CC = gcc
CFLAGS = -Wall
LDFLAGS =
OBJFILES = main.o bcm2835.o lr1110_hal.o lr1110_gnss.o lr1110_regmem.o lr1110_wifi.o lr1110_system.o
TARGET = lrtest

all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

clean:
	rm -f $(OBJFILES) $(TARGET) *~
