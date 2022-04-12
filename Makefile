CXX = g++
CC = gcc

EDCFLAGS:= -std=gnu11 -O2 $(CFLAGS)
EDCXXFLAGS:= -Wall -O2 -std=c++11 -I ./ -I drivers/ -I include/ -I network/ -I clkgen/include $(CXXFLAGS) -DDEFINE_WEAK
EDLDFLAGS:= -lpthread -lcfitsio -lncursest -lm $(LDFLAGS)

EDLDFLAGS+=$(shell pkg-config gstreamer-1.0 --libs)
EDLDFLAGS+=$(shell pkg-config gstreamer-video-1.0 --libs)
EDLDFLAGS+=$(shell pkg-config gobject-introspection-1.0 --libs)
EDLDFLAGS+=$(shell pkg-config tcam --libs)

EDCFLAGS+=$(shell pkg-config gstreamer-1.0 --cflags)
EDCFLAGS+=$(shell pkg-config gstreamer-video-1.0 --cflags)
EDCFLAGS+=$(shell pkg-config gobject-introspection-1.0 --cflags)
EDCFLAGS+=$(shell pkg-config tcam --cflags)

EDCXXFLAGS+=$(shell pkg-config gstreamer-1.0 --cflags)
EDCXXFLAGS+=$(shell pkg-config gstreamer-video-1.0 --cflags)
EDCXXFLAGS+=$(shell pkg-config gobject-introspection-1.0 --cflags)
EDCXXFLAGS+=$(shell pkg-config tcam --cflags)

COBJS = 
CPPOBJS = $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))

all: $(COBJS) $(CPPOBJS)
	$(CXX) -o atiktest.out $(COBJS) $(CPPOBJS) $(EDLDFLAGS)

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	rm -vf $(CPPOBJS)
	rm -vf *.out
	rm -vf *.jpg