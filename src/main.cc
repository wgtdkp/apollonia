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
static World world({0.0f, -10.0f});

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
  glTranslatef(0.0f, -7.0f, -25.0f);

  auto now = high_resolution_clock::now();
  auto dt = duration_cast<duration<double>>(now - last_clock).count();
  last_clock = now;
  
  int h = glutGet(GLUT_WINDOW_HEIGHT);
  DrawText(5, h - 20, "dt: %.2f ms", dt * 1000);

  dt = 0.016;
  world.Step(dt);
  for (auto body : world.bodies()) {
    DrawBody(*body);
  }

  glutSwapBuffers();
}

static void Test() {
  auto fencing = World::NewBody(kInf, {0, -10}, 100, 20);
  world.Add(fencing);
  
  Body* body;
  body = World::NewBody(200, {0, 8}, 2, 2);
  //body->velocity = {20, -10};
  body->rotation = Mat22(kPi / 4);
  world.Add(body);
}

Float Random(Float low, Float high) {
  return 1.0 * rand() / RAND_MAX * (high - low) + low;
}

// A vertical stack
static void TestStack() {
  auto fencing = World::NewBody(kInf, {0, -10}, 100, 20);
	fencing->friction = 0.2f;
	world.Add(fencing);

	for (int i = 0; i < 10; ++i) {
    Float x = Random(-0.1f, 0.1f);
    auto body = World::NewBody(1, {x, 0.51f + 1.05f * i}, 1, 1);
		body->friction = 0.2f;
		world.Add(body);
	}
}

static void TestPyramid() {
  auto fencing = World::NewBody(kInf, {0, -10}, 100, 20);
	fencing->friction = 0.2f;
  world.Add(fencing);
  
  Vec2 x(-6.0f, 0.75f);
	Vec2 y;

  int n = 10;
	for (int i = 0; i < n; ++i) {
		y = x;

		for (int j = i; j < n; ++j) {
      auto body = World::NewBody(10, y, 1, 1);
			body->friction = 0.2f;
			world.Add(body);
			y += Vec2(1.125f, 0.0f);
		}
		x += Vec2(0.5625f, 2.0f);
	}
}

static void TestJoint() {
  auto ground = World::NewBody(kInf, {0, -10}, 100, 20);
  world.Add(ground);
  
	auto box = World::NewBody(100, {9, 11}, 1, 1);
	world.Add(box);

  auto joint = World::NewRevoluteJoint(*ground, *box, {0, 11});
	world.Add(joint);
}

static void TestChain() {
  auto ground = World::NewBody(kInf, {0, -10}, 100, 20);
  world.Add(ground);

	const Float mass = 10.0f;
	const Float y = 12.0f;
  Body* last = ground;
	for (int i = 0; i < 15; ++i) {
    auto box = World::NewBody(mass, {0.5f+i, y}, 0.75, 0.25);
    world.Add(box);
    auto joint = World::NewRevoluteJoint(*last, *box, Vec2(i, y));
    world.Add(joint);
    last = box;
	}
}

static void Keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case '1':
    world.Clear();
    Test();
    break;
  case '2':
    world.Clear();
    TestStack();
    break;
  case '3':
    world.Clear();
    TestPyramid();
    break;
  case '4':
    world.Clear();
    TestJoint();
    break;
  case '5':
    world.Clear();
    TestChain();
    break;
  }
  
}

static void Mouse(int button, int state, int x, int y) {

}

static void Reshape(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, width/(float)height, 0.1, 100.0);
}

int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(800, 800);
  glutCreateWindow("Apollonia");

  TestPyramid();

  glutReshapeFunc(Reshape);
  glutDisplayFunc(ApolloniaRun);
  glutKeyboardFunc(Keyboard);
  glutMouseFunc(Mouse);
  glutIdleFunc(ApolloniaRun);
  glutMainLoop();

  return 0;
}
