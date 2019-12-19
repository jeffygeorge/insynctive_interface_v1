#
#	Makefile
#

BASE_PATH = /opt/poky/1.6.2/sysroots
EXEC_PATH = $(BASE_PATH)/x86_64-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi
LIB_PATH = $(BASE_PATH)/armv5te-poky-linux-gnueabi/usr/lib
INC_PATH = $(BASE_PATH)/armv5te-poky-linux-gnueabi/usr/include
DEBUG = -g
CXX = $(EXEC_PATH)/arm-poky-linux-gnueabi-gcc
CC = $(EXEC_PATH)/arm-poky-linux-gnueabi-gcc
LD = $(EXEC_PATH)/arm-poky-linux-gnueabi-gcc

CXXFLAGS = -O2 -pipe -g -feliminate-unused-debug-types $(DEBUG) -Wall -I $(INC_PATH)
CFLAGS = -O2 -pipe -std=gnu99 -feliminate-unused-debug-types $(DEBUG) -Wall -I $(INC_PATH)
LIBS = -lc -lpthread -lrt -ldl -lsqlite3

LDFLAGS = $(DEBUG) $(TARGET) -L $(LIB_PATH) -Bdynamic

OBJS := $(addsuffix .o,$(basename $(shell ls *.c)))

all : insynctive

insynctive : $(OBJS)
	 $(LD) -o $@ $(LDFLAGS) $^ $(LIBS)

clean:
	@$(RM) -f $(OBJS) insynctive *.d *~ #*#

.PHONY : all insynctive clean
