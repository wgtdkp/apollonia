#include "body.h"
#include "collision.h"
#include "joint.h"
#include "world.h"

#include <GLFW/glfw3.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace apollonia;

static World world({0, -9.8});
static constexpr int win_width = 800;
static constexpr int win_height = 800;
static GLFWwindow* window = nullptr;

static void DrawBody(const PolygonBody& body) {
  if (body.mass() == kInf) {
    glColor3f(1, 1, 1);
  } else {
    glColor3f(0.8, 0.8, 0);
  }
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

  glColor3f(0.6, 0.6, 0.6);
  glBegin(GL_LINES);
  if (joint.a().mass() != kInf) {
    glVertex2f(centroida.x, centroida.y);
    glVertex2f(anchora.x, anchora.y);
  }
  if (joint.b().mass() != kInf) {
    glVertex2f(centroidb.x, centroidb.y);
    glVertex2f(anchorb.x, anchorb.y);
  }
  glEnd();
}

static auto DiffTime() {
  using namespace std::chrono;
  using Seconds = std::chrono::duration<double>;
  static auto last_clock = high_resolution_clock::now();
  auto now = high_resolution_clock::now();
  auto dt = duration_cast<Seconds>(now - last_clock);
  last_clock = now;
  return dt;
}

/*
static void UpdateTitle(double dt) {
  int fps = round(1.0 / dt);
  std::stringstream ss;
  ss << "Apollonia - fps: " << fps;
  glfwSetWindowTitle(window, ss.str().c_str());
}
*/

static void Display() {
  glViewport(0, 0, win_width, win_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-12, 12, -12, 12, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glTranslatef(0.0f, -8.0f, 0.0f);

  glClear(GL_COLOR_BUFFER_BIT);
  world.Lock();
  for (auto body : world.bodies()) {
    // TODO(wgtdkp):
    DrawBody(dynamic_cast<PolygonBody&>(*body));
  }
  for (auto joint : world.joints()) {
    DrawJoint(dynamic_cast<RevoluteJoint&>(*joint));
  }
  world.Unlock();
  glfwSwapBuffers(window);
}

static PolygonBody* CreateFencing() {
  auto ground = World::NewBox(kInf, 20, 1, {0, -0.5});
  world.Add(ground);
  world.Add(World::NewBox(kInf, 20, 1, {0, 16.5}));
  world.Add(World::NewBox(kInf, 1, 18, {-9.5, 8}));
  world.Add(World::NewBox(kInf, 1, 18, {9.5, 8}));
  return ground;
}

static void TestPolygon() {
  world.Lock();
  CreateFencing();
  world.Add(World::NewPolygonBody(200, {{-1, 0}, {1, 0}, {0, 1}}, {-1, 0}));
  world.Add(World::NewPolygonBody(200, {{-1, 0}, {1, 0}, {0, 1}}, {1, 0}));
  world.Add(World::NewBox(200, 3, 6, {0, 8}));
  world.Unlock();
}

Float Random(Float low, Float high) {
  return 1.0 * rand() / RAND_MAX * (high - low) + low;
}

// A vertical stack
static void TestStack() {
  world.Lock();
  CreateFencing();
  for (int i = 0; i < 10; ++i) {
    Float x = Random(-0.1f, 0.1f);
    auto body = World::NewBox(1, 1, 1, {x, 0.51f + 1.05f * i});
    body->set_friction(0.2);
    world.Add(body);
  }
  world.Unlock();
}

static void TestPyramid() {
  world.Lock();
  CreateFencing();
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
    x += Vec2(0.5625f, 1.5f);
  }
  world.Unlock();
}

static void TestJoint() {
  world.Lock();
  auto ground = World::NewBox(kInf, 100, 20, {0, -10});
  world.Add(ground);

  auto box1 = World::NewBox(500, 1, 1, {13.5, 11});
  world.Add(box1);
  auto joint1 = World::NewRevoluteJoint(*ground, *box1, {4.5, 11});
  world.Add(joint1);

  for (size_t i = 0; i < 5; ++i) {
    auto box2 = World::NewBox(100, 1, 1, {3.5f-i, 2});
    world.Add(box2);
    auto joint2 = World::NewRevoluteJoint(*ground, *box2, {3.5f-i, 11});
    world.Add(joint2);
  }
  world.Unlock();
}

static void TestChain() {
  world.Lock();
  auto ground = World::NewBox(kInf, 100, 20, {0, -10});
  ground->set_friction(0.4);
  world.Add(ground);

  const Float mass = 10.0f;
  const Float y = 12.0f;
  Body* last = ground;
  for (int i = 0; i < 15; ++i) {
    auto box = World::NewBox(mass, 0.75, 0.25, {0.5f+i, y});
    box->set_friction(0.4);
    world.Add(box);
    auto joint = World::NewRevoluteJoint(*last, *box, Vec2(i, y));
    world.Add(joint);
    last = box;
  }
  world.Unlock();
}

static void Keyboard(GLFWwindow* window,
    int key, int scancode, int action, int mods) {
  world.Lock();
  world.Clear();
  world.Unlock();
  switch (key) {
  case '1': TestPolygon(); break;
  case '2': TestStack(); break;
  case '3': TestPyramid(); break;
  case '4': TestJoint(); break;
  case '5': TestChain(); break;
  }
}

/*
static void Reshape(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, 1.0*width/height, 0.1, 100.0);
}
*/

static std::atomic<bool> should_stop{false};
static void ApolloniaRun() {
  using namespace std::chrono_literals;
  TestPyramid();
  while (!should_stop) {
    // We give up some time for drawing
    std::this_thread::sleep_for(10ms);
    auto dt = DiffTime().count();

    world.Lock();
    world.Step(dt);
    world.Unlock();
  }
}

int main() {
  if (!glfwInit()) {
    return -1;
  }

  window = glfwCreateWindow(win_width, win_height, "apollonia", nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    return -2;
  }
  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
  glfwMakeContextCurrent(window);

  glfwSetKeyCallback(window, Keyboard);

  std::thread apollo_thread(ApolloniaRun);
  while (!glfwWindowShouldClose(window)) {
    Display();
    glfwPollEvents();
  }
  glfwTerminate();
  should_stop = true;
  apollo_thread.join();
  return 0;
}
