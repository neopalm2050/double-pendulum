
all: main.exe

main.exe: main.cpp Makefile
	g++ -g -Ifreeglut/include -IEigen3/include/eigen3 -Lfreeglut/lib/x64 main.cpp -lfreeglut -lopengl32 -lglu32 -o main.exe