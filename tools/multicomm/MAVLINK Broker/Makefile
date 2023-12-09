CXX = g++
SRCS = mavlink_relay_server.cpp
OBJS = $(SRCS:.cpp = .o)
TARGET = mavlink_relay_server
INC = -I./c_library_v2

all : $(TARGET)
		$(CXX) -o $(TARGET) $(OBJS) $(INC)

$(TARGET) : 
		$(CXX) -c $(SRCS) $(INC) 

clean : 
		rm -f $(TARGET)
		rm -f *.o