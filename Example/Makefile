
all: MS5611

HDRS = ../I2Cdev/I2Cdev.h ../MS561101BA.h
MS5611_OBJS = ../I2Cdev/I2Cdev.o ../MS561101BA.o main.o

CXXFLAGS =  -Wall -g -O2 

$(MS5611_OBJS) : $(HDRS)

MS5611: $(MS5611_OBJS) 
	$(CXX) -o $@ $^ -lm



clean:
	rm -f $(MS5611_OBJS) MS5611

