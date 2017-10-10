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

static void DrawBody(const PolygonBody& body) {
	glColor3f(0.8f, 0.8f, 0.0f);
  glBegin(GL_LINE_LOOP);
  for (size_t i = 0; i < body.Count(); ++i) {
    auto point = body.LocalToWorld(body[i]);
	  glVertex2f(point.x, point.y);
  }
	glEnd();
}

static void DrawJoint(const RevoluteJoint& joint) {
  auto centroida = joint.a().LocalToWorld(joint.a().centroid());
  auto anchora = joint.WorldAnchorA();
  auto centroidb = joint.b().LocalToWorld(joint.b().centroid());
  auto anchorb = joint.WorldAnchorB();

  glColor3f(1, 1, 1);
  glBegin(GL_LINES);
  glVertex2f(centroida.x, centroida.y);
  glVertex2f(anchora.x, anchora.y);
  glVertex2f(centroidb.x, centroidb.y);
  glVertex2f(anchorb.x, anchorb.y);
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
    // TODO(wgtdkp):
    DrawBody(dynamic_cast<PolygonBody&>(*body));
  }

  for (auto joint : world.joints()) {
    DrawJoint(dynamic_cast<RevoluteJoint&>(*joint));
  }

  glutSwapBuffers();
}

static void TestPolygon() {
  auto ground = World::NewBox(kInf, 100, 20, {0, -10});
  world.Add(ground);

  world.Add(World::NewPolygonBody(200, {{-1, 0}, {1, 0}, {0, 1}}, {-1, 0}));
  world.Add(World::NewPolygonBody(200, {{-1, 0}, {1, 0}, {0, 1}}, {1, 0}));
  world.Add(World::NewBox(200, 3, 6, {0, 8}));
}

Float Random(Float low, Float high) {
  return 1.0 * rand() / RAND_MAX * (high - low) + low;
}

// A vertical stack
static void TestStack() {
  auto fencing = World::NewBox(kInf, 100, 20, {0, -10});
	fencing->set_friction(0.2);
	world.Add(fencing);

	for (int i = 0; i < 10; ++i) {
    Float x = Random(-0.1f, 0.1f);
    auto body = World::NewBox(1, 1, 1, {x, 0.51f + 1.05f * i});
		body->set_friction(0.2);
		world.Add(body);
	}
}

static void TestPyramid() {
  auto fencing = World::NewBox(kInf, 100, 20, {0, -10});
	fencing->set_friction(0.2);
  world.Add(fencing);
  
  Vec2 x(-6.0f, 0.75f);
	Vec2 y;

  int n = 10;
	for (int i = 0; i < n; ++i) {
		y = x;
		for (int j = i; j < n; ++j) {
      auto body = World::NewBox(10, 1, 1, y);
			body->set_friction(0.2);
			world.Add(body);
			y += Vec2(1.125f, 0.0f);
		}
		x += Vec2(0.5625f, 2.0f);
	}
}

static void TestJoint() {
  auto ground = World::NewBox(kInf, 100, 20, {0, -10});
  world.Add(ground);
  
	auto box1 = World::NewBox(100, 1, 1, {9, 11});
	world.Add(box1);
  auto joint1 = World::NewRevoluteJoint(*ground, *box1, {0, 11});
  world.Add(joint1);
  
  auto box2 = World::NewBox(100, 1, 1, {-1, 2});
  world.Add(box2);
  auto joint2 = World::NewRevoluteJoint(*ground, *box2, {-1, 11});
  world.Add(joint2);
}

static void TestChain() {
  auto ground = World::NewBox(kInf, 100, 20, {0, -10});
  world.Add(ground);

	const Float mass = 10.0f;
	const Float y = 12.0f;
  Body* last = ground;
	for (int i = 0; i < 15; ++i) {
    auto box = World::NewBox(mass, 0.75, 0.25, {0.5f+i, y});
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
    TestPolygon();
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

  TestJoint();

  glutReshapeFunc(Reshape);
  glutDisplayFunc(ApolloniaRun);
  glutKeyboardFunc(Keyboard);
  glutMouseFunc(Mouse);
  glutIdleFunc(ApolloniaRun);
  glutMainLoop();

  return 0;
}
