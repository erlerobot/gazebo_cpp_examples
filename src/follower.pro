SOURCES += \
    image_subscriber.cpp


INCLUDEPATH+= /opt/ros/indigo/include/ /usr/local/include

LIBS+= -L/opt/ros/indigo/lib  -L/usr/lib/x86_64-linux-gnu/ -L/usr/local/lib\
 -lcv_bridge -lrosconsole -lroscpp -limage_transport\
 -lopencv_core -lopencv_highgui\
 -laruco
