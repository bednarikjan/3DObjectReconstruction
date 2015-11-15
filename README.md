# Reconstruciotn of 3D models from 2D images

The application enables the user to reconstruct simple block-shaped objects together with their position in the 3D world from 2D images of the scene. The user is required to draw 3 or 4 points specifying one base of the object while the volume and the position could then be easily derived as the user stretches the rendered object (to match the underlying image). 

## How it works?

Multiple images of the scene including a printed chessboard must be provided. It is then possible to calculate both the camera intrisic and extrinsic matrix. As the next step the world -> camera frame transformation matrix is computed. Since the chesboard serves the purpose of the marker here and defines an XY plane (Z = 0) it is possible to compute the homography between the world plane and the image. Using the inverse homography matrix the coordinates of the object vertices in the 3D scene are finally obtained.

## Install and run

```
$ cmake
$ make
$ ./objectReconstruction
```

![objects](http://bednarikjan.github.io/img/2015-08-08-3D_object_from_2D/table.png)
