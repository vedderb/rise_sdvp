TARGET = test
LIBS = -lm -L../ -lRControlStationComm
CC = gcc
CFLAGS = -g -Wall -std=gnu99 -I../
SOURCES = test.c
HEADERS = 
OBJECTS = $(SOURCES:.c=.o)

.PHONY: default all clean

default: $(TARGET)
all: default

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

run: $(TARGET)
	./$(TARGET)

