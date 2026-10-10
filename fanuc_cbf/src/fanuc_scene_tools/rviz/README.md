# Orthographic view configs

Three RViz configs giving simultaneous XY / XZ / YZ orthographic views.

RViz2's only orthographic view controller is `TopDownOrtho`
(`FixedOrientationOrthoViewController`). Its `orientCamera()` builds the camera
orientation from a single scalar `angle_`, so it always looks straight down the
scene's -Z and the `Target Frame` property only follows position -- setting
`Target Frame: view_xz` leaves the view stubbornly XY. That was tried first and
does not work.

What does work is changing `Global Options -> Fixed Frame`. RViz transforms every
display into the fixed frame before rendering, so pointing the fixed frame
elsewhere rotates the whole scene under a camera that still looks down -Z:

| config        | Fixed Frame | projection            |
|---------------|-------------|-----------------------|
| ortho_xy.rviz | world       | world X right, Y up   |
| ortho_xz.rviz | view_xz     | world X right, Z up   |
| ortho_yz.rviz | view_yz     | world Y right, Z up   |

`view_xz` and `view_yz` are static frames published by the launch file at
rpy (90,0,0) and (90,0,90) about `world`.
