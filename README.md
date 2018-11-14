# Apollonia

A simplicity-first 2D physics engine.

## Dependency

- [OpenGL]

## Demo

[Stacking, Pyramid, Pendulum, Chain](./demo/demo.html)

## Build

Try on ubuntu, Mac OS and windows:

```bash
$ git clone https://github.com/wgtdkp/apollonia
$ cd apollonia && mkdir build && cd build
$ cmake ..
```

For windows, VC project files will be generated under `build/`. For Linux(ubuntu) and Mac OS, we can build apollonia by:

```bash
$ make
$ ./apollonia
```

## Reference

- [Box2D]
- [Chipmunk2D]

[OpenGL]:https://www.opengl.org/
[GLUT]:https://www.opengl.org/resources/libraries/glut/
[Box2D]:http://box2d.org/
[Chipmunk2D]:https://chipmunk-physics.net/
