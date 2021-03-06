INCLUDE = $(shell pkg-config --cflags opencv)
LIBS    = $(shell pkg-config --libs   opencv)

all:main.cpp 
	nvcc  -ccbin g++  -I $(INCLUDE) main.cpp -o tracking  $(LIBS) -lpthread 
clean:
	rm tracking
