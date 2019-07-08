CFLAGS = -Wall

all: weather-sensors 

weather-sensors: weather-sensors.o
	$(CC) weather-sensors.o -o weather-sensors -lm -lutil -liio

clean:
	rm -f *.o weather

%.0:	%.c
	$(CC) -c $< -o $@

