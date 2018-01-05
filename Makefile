TARGET=project
CXX=g++
CFLAGS= -std=c++11 `pkg-config --libs --cflags opencv`

default: $(TARGET)

$(TARGET): main.o  undistort_images.o calibrate_camera.o epipoles.o undistort_images.h calibrate_camera.h epipoles.h
	$(CXX) -Wall main.o undistort_images.o calibrate_camera.o epipoles.o undistort_images.h calibrate_camera.h epipoles.h $(CFLAGS) -o project

main.o: calibrate_camera.o undistort_images.o epipoles.o  undistort_images.h calibrate_camera.h epipoles.h
	$(CXX) -c main.cpp $(CFLAGS)

undistort_images.o : calibrate_camera.o calibrate_camera.h undistort_images.h
	$(CXX) -c undistort_images.cpp $(CFLAGS)

calibrate_camera.o : calibrate_camera.h
	$(CXX) -c calibrate_camera.cpp $(CFLAGS)

epipoles.o : epipoles.h
	$(CXX) -c epipoles.cpp $(CFLAGS)

clean:
	rm -f  main.o calibrate_camera.o undistort_images.o epipoles.o project
