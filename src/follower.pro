SOURCES += \
    image_subscriber.cpp


INCLUDEPATH+= /opt/ros/indigo/include/ /usr/local/include /home/erle/simulation/ros_catkin_ws/devel/include/

LIBS+= -L/opt/ros/indigo/lib  -L/usr/lib/x86_64-linux-gnu/ -L/usr/local/lib\
 -L/home/erle/simulation/ros_catkin_ws/devel/lib/ \
 -lmavros \
 -lcv_bridge -lrosconsole -lroscpp -limage_transport -lroscpp_serialization\
 -lopencv_core -lopencv_highgui\
 -laruco

HEADERS +=
