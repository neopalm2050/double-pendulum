#include <GL/freeglut.h>
#include <Eigen/Dense>
#include <cmath>
#include <deque>

//5 ms per physics tick, or 200 ticks per second
#define msPerTick 1
#define msPerFrame 17
#define mass1 2.0
#define mass2 1.0
#define length1 0.8
#define length2 1.0
#define gravity 9.8
#define maxTrailLength 12500
#define taper_speed 0.9999
#define pi 3.141592653589793238462643383279

//the number of "units" tall half the canvas is
const double scale = (length1 + length2) * 1;

double currentTheta1Position = 0.6;
double currentTheta2Position = 0.5;
double currentTheta1Velocity = 0;
double currentTheta2Velocity = 0;

struct Point {
    double x;
    double y;
};

std::deque<Point> stateTrace;

void getAccels(double theta1, double theta2, double theta1dot, double theta2dot, double& theta1accel, double& theta2accel) {
	Eigen::Matrix2d A;
	Eigen::Vector2d b;
	A << (mass1+mass2) * length1*length1                       , mass2 * length1*length2 * cos(theta1 - theta2),
	      mass2        * length1*length2 * cos(theta1 - theta2), mass2 * length2*length2                       ;
	
	b << mass2*length1*length2 * theta2dot*theta2dot * sin(theta2 - theta1) - (mass1+mass2) * gravity * length1 * sin(theta1),
	     mass2*length1*length2 * theta1dot*theta1dot * sin(theta1 - theta2) -  mass2        * gravity * length2 * sin(theta2);
	
	Eigen::Vector2d accels = A.colPivHouseholderQr().solve(b);
	
	theta1accel = accels[0];
	theta2accel = accels[1];
	return;
}

Point stateToLocation(double theta1, double theta2, int windowWidth, int windowHeight) {
	double pivotX = 0.0;
	double pivotY = 0.0;
	
	double mass1X = pivotX + sin(theta1) * length1 / scale * ( (double) windowHeight)/windowWidth;
	double mass1Y = pivotY - cos(theta1) * length1 / scale;
	
	double mass2X = mass1X + sin(theta2) * length2 / scale * ( (double) windowHeight)/windowWidth;
	double mass2Y = mass1Y - cos(theta2) * length2 / scale;
	
	return {mass2X, mass2Y};
}

void physicsTick(int arg) {
	glutTimerFunc(msPerTick, physicsTick, 0);
	
	double currentTheta1Accel, currentTheta2Accel;
	getAccels(currentTheta1Position, currentTheta2Position, currentTheta1Velocity, currentTheta2Velocity, currentTheta1Accel, currentTheta2Accel);
	
	currentTheta1Position += currentTheta1Velocity * ( (double) msPerTick)/1000 + currentTheta1Accel * ( (double) msPerTick*msPerTick) / 2000000;
	currentTheta2Position += currentTheta2Velocity * ( (double) msPerTick)/1000 + currentTheta2Accel * ( (double) msPerTick*msPerTick) / 2000000;
	
	currentTheta1Velocity += currentTheta1Accel * ( (double) msPerTick)/1000;
	currentTheta2Velocity += currentTheta2Accel * ( (double) msPerTick)/1000;
	
	if (currentTheta1Position >= 2*pi) {
		currentTheta1Position -= 2*pi;
	}
	else if (currentTheta1Position <= -2*pi) {
		currentTheta1Position += 2*pi;
	}
	
	if (currentTheta2Position >= 2*pi) {
		currentTheta2Position -= 2*pi;
	}
	else if (currentTheta2Position <= -2*pi) {
		currentTheta2Position += 2*pi;
	}
	
	stateTrace.push_front({currentTheta1Position, currentTheta2Position});
	
	if (stateTrace.size() > maxTrailLength) {
		stateTrace.pop_back();
	}
}

void frameAdvance(int arg) {
	glutPostRedisplay();
    glutTimerFunc(msPerFrame, frameAdvance, 0);
}

void display() {
    int windowWidth = glutGet(GLUT_WINDOW_WIDTH);
    int windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
    //int timeSinceStart = glutGet(GLUT_ELAPSED_TIME);
    //int dt = timeSinceStart - oldTimeSinceStart;
    //oldTimeSinceStart = timeSinceStart;

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)
    
	
	Point prevTracePoint = stateToLocation(stateTrace.front().x, stateTrace.front().y, windowWidth, windowHeight);
    double brightness = 1;
	
	for (std::deque<Point>::iterator it = stateTrace.begin(); it != stateTrace.end(); ++it) {
		Point currTracePoint = stateToLocation((*it).x, (*it).y, windowWidth, windowHeight);
		
		glBegin(GL_LINES);
            glColor3d(0, brightness, 0);
            glVertex2d(prevTracePoint.x, prevTracePoint.y);
            glVertex2d(currTracePoint.x, currTracePoint.y);
        glEnd();
		brightness *= taper_speed;
		
		prevTracePoint = currTracePoint;
	}
	
	double theta1 = currentTheta1Position;
	double theta2 = currentTheta2Position;
	
	double pivotX = 0.0;
	double pivotY = 0.0;
	
	double mass1X = pivotX + sin(theta1) * length1 / scale * ( (double) windowHeight)/windowWidth;
	double mass1Y = pivotY - cos(theta1) * length1 / scale;
	
	double mass2X = mass1X + sin(theta2) * length2 / scale * ( (double) windowHeight)/windowWidth;
	double mass2Y = mass1Y - cos(theta2) * length2 / scale;
	
	glBegin(GL_QUADS);
		glColor3d(0.8, 0.8, 0.8);
		glVertex2d(pivotX - cos(theta1) * 0.01 * ( (double) windowHeight)/windowWidth, pivotY - sin(theta1) * 0.01);
		glVertex2d(pivotX + cos(theta1) * 0.01 * ( (double) windowHeight)/windowWidth, pivotY + sin(theta1) * 0.01);
		glVertex2d(mass1X + cos(theta1) * 0.01 * ( (double) windowHeight)/windowWidth, mass1Y + sin(theta1) * 0.01);
		glVertex2d(mass1X - cos(theta1) * 0.01 * ( (double) windowHeight)/windowWidth, mass1Y - sin(theta1) * 0.01);
	glEnd();
	
	glBegin(GL_QUADS);
		glColor3d(0.8, 0.8, 0.8);
		glVertex2d(mass1X - cos(theta2) * 0.01 * ( (double) windowHeight)/windowWidth, mass1Y - sin(theta2) * 0.01);
		glVertex2d(mass1X + cos(theta2) * 0.01 * ( (double) windowHeight)/windowWidth, mass1Y + sin(theta2) * 0.01);
		glVertex2d(mass2X + cos(theta2) * 0.01 * ( (double) windowHeight)/windowWidth, mass2Y + sin(theta2) * 0.01);
		glVertex2d(mass2X - cos(theta2) * 0.01 * ( (double) windowHeight)/windowWidth, mass2Y - sin(theta2) * 0.01);
	glEnd();
	
 
    glFlush();  // Render now
}


int main(int argc, char** argv) {
    //initialize program-specific stuff
    
    glutInit(&argc, argv);                 // Initialize GLUT
    glutCreateWindow("Double Pendulum"); // Create a window with the given title
    glutInitWindowSize(320, 320);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutDisplayFunc(display); // Register display callback handler for window re-paint
    glutTimerFunc(msPerTick, physicsTick, 0); // Schedule a physics tick. The next one then is scheduled by that physics tick.
	glutTimerFunc(msPerFrame, frameAdvance, 0); //Begin the display loop.
    glutMainLoop();           // Enter the event-processing loop
    return 0;
}