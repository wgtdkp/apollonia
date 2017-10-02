OBJS_DIR = build/
TARGET = apollonia
CXX = clang++

SRCS = arbiter.cc body.cc collision.cc joint.cc main.cc world.cc

CXXFLAGS = -std=c++11 -g -Wall -Wfatal-errors
LDFLAGS = -lGLU -lglut -lGL
OBJS = $(addprefix $(OBJS_DIR), $(SRCS:.cc=.o))

default: all

all:
	@mkdir -p $(OBJS_DIR)
	@make $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(OBJS_DIR)$@ $^ $(LDFLAGS)

$(OBJS_DIR)%.o: %.cc
	$(CXX) $(CXXFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	-rm -rf $(OBJS_DIR)
