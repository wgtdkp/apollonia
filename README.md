# Apollonia  <a href="https://996.icu"><img src="https://img.shields.io/badge/link-996.icu-red.svg"></a>

A simplicity-first 2D physics engine.

## Dependency

- [OpenGL]

## Demo

[Stacking, Pyramid, Pendulum, Chain](./demo/apollonia-demo.mp4)

## Build

Try on ubuntu, Mac OS and windows:

```bash
$ git clone --recurse-submodules https://github.com/wgtdkp/apollonia
$ cd apollonia && cmake -S . -B build
$ cmake --build
```

For windows, VC project files will be generated under `build/`. For Linux(ubuntu) and Mac OS, the `apollonia` binary will be generated under the `build/` directory.

## Reference

- [Box2D]
- [Chipmunk2D]

[OpenGL]:https://www.opengl.org/
[GLUT]:https://www.opengl.org/resources/libraries/glut/
[Box2D]:http://box2d.org/
[Chipmunk2D]:https://chipmunk-physics.net/
