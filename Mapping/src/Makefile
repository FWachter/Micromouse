CXX=g++
INC=$(CURDIR)/../inc
CPPFLAGS=-I$(INC) -std=c++14 -Wall
SRCS=Main.cpp Map.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: out

out: $(OBJS)
	-@$(CXX) $(CPPFLAGS) $(OBJS) -o out

%.o: %.cpp
	-@$(CXX) -c $(CPPFLAGS) $<

clean:
	-@rm -rf $(OBJS) out > /dev/null 2>&1
