all: envirovision

clean:
	rm -f envirovision

INCLUDE_DIRS = -I /home/fa/CmakeWpilib/libraries/ntcore/ntcore/src/main/native/include/ -I /home/fa/CmakeWpilib/libraries/wpiutil/wpiutil/src/main/native/include/ -I/home/fa/CmakeWpilib/libraries/cscore/cscore/src/main/native/include
LIB_DIRS = -L /home/fa/CmakeWpilib/build/libraries/ntcore/ -L /home/fa/CmakeWpilib/build/libraries/wpiutil/ -L/home/fa/CmakeWpilib/build/libraries/cscore

envirovision: envirovision.cpp
	g++ -o envirovision envirovision.cpp $(INCLUDE_DIRS) -std=gnu++11 $(LIB_DIRS) -lcscore -lwpiutil -lntcore -lpthread -lv4l2 `pkg-config opencv --cflags --libs` -lwiringPi
