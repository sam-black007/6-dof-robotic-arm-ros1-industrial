# Model provenance

The manipulation scene models used by the demo worlds are taken from the
Amazon Picking Challenge (APC), originally obtained from
http://rll.berkeley.edu/amazon_picking_challenge/

- biscuits
- eraser
- soap
- soap2

These APC meshes are used by `pick_place.world` and `setup_1.world` as
graspable objects. All other scene assets (tables, dropbox, kinect sensor,
ur5 mount) are stock Gazebo models referenced through `GAZEBO_MODEL_PATH`,
which the launch file sets to this directory.