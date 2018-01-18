INCLUDE = $(shell pkg-config --cflags opencv)
LIBS    = $(shell pkg-config --libs   opencv)

all:main.cpp 
	g++ -Wall -I $(INCLUDE) main.cpp -o main $(LIBS) -lpthread 
clean:
	rm main
