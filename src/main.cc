#include "body.h"
#include "collision.h"
#include "joint.h"
#include "world.h"
#include <GL/glut.h>
#include <GL/glu.h>
#include <chrono>

using namespace apollonia;
using namespace std::chrono;
static time_point<high_resolution_clock> last_clock = high_resolution_clock::now();
static World world({0.0f, 0.0f});

static void DrawText(int x, int y, const char* format, ...) {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glColor3f(0.9f, 0.9f, 0.9f);
  glRasterPos2i(x, y);
  
  char buffer[256];
  va_list args;
  va_start(args, format);
  int len = vsprintf(buffer, format, args);
  va_end(args);
  for (int i = 0; i < len; ++i) {
    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, buffer[i]);
  }

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

static void DrawAxis() {
  glColor3f(0.9f, 0.9f, 0.9f);
  glBegin(GL_LINE_LOOP);
  glVertex2f(-10, 0);
  glVertex2f(10, 0);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex2f(0, -10);
  glVertex2f(0, 10);
  glEnd();
}

static void DrawBody(const Body& body) {
	glColor3f(0.8f, 0.8f, 0.0f);

  glBegin(GL_LINE_LOOP);
  for (size_t i = 0; i < body.Count(); ++i) {
    auto point = body.LocalToWorld(body[i]);
	  glVertex2f(point.x, point.y);
  }
	glEnd();
}

static void ApolloniaRun() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0f, -5.0f, -25.0f);
  // Step:
  auto now = high_resolution_clock::now();
  auto dt = duration_cast<duration<double>>(now - last_clock).count();
  last_clock = now;
  
  int h = glutGet(GLUT_WINDOW_HEIGHT);
  DrawText(5, h - 20, "dt: %.2f ms", dt * 1000);
  DrawAxis();

  world.Step(dt);
  for (auto body : world.bodies()) {
    DrawBody(*body);
  }

  glutSwapBuffers();
}

static void Keyboard(unsigned char key, int x, int y) {

}

static void Mouse(int button, int state, int x, int y) {

}

static void Reshape(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, width/(float)height, 0.1, 100.0);
}

static void test() {
  auto body = World::NewBody(20, {0, 10}, 2, 2);
  body->angularVelocity = 1;
  world.Add(body);
}

int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(800, 800);
  glutCreateWindow("Apollonia");

  test();

  glutReshapeFunc(Reshape);
  glutDisplayFunc(ApolloniaRun);
  glutKeyboardFunc(Keyboard);
  glutMouseFunc(Mouse);
  glutIdleFunc(ApolloniaRun);
  glutMainLoop();

  return 0;
}
