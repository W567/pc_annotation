# Point Cloud Annotation Toolkit (Open3D-based)

## Dependency

### Open3D

Build Open3D from source based on https://www.open3d.org/docs/release/compilation.html.
```
git clone https://github.com/isl-org/Open3D

# Only needed for Ubuntu
util/install_deps_ubuntu.sh

mkdir build
cd build
cmake ..
make -j
sudo make install
make install-pip-package # To install pip package
# install open3d with generated pip wheel in build/lib
```

## Build from Source

```
cd pc_annotation
mkdir build
cd build
cmake ..
make -j
sudo make install
```

## usage

```
python scripts/example.py
```

![Usage](figs/usage.png)

```
# Press 'h' with Open3D visualizer selected to show help menu
[Open3D INFO] ---------------- View control ----------------
[Open3D INFO]     F            : Enter free view.
[Open3D INFO]     X            : Enter orthogonal view along X axis, press again to flip.
[Open3D INFO]     Y            : Enter orthogonal view along Y axis, press again to flip.
[Open3D INFO]     Z            : Enter orthogonal view along Z axis, press again to flip.
[Open3D INFO]     C            : Toggle coordinate visualization.

[Open3D INFO] ---------------- Color control ----------------
[Open3D INFO]     0..5,9       : Set point cloud color option.
[Open3D INFO]                    0 - Default behavior, render point color.
[Open3D INFO]                    1 - Render point color.
[Open3D INFO]                    2 - x coordinate as color.
[Open3D INFO]                    3 - y coordinate as color.
[Open3D INFO]                    4 - z coordinate as color.
[Open3D INFO]                    5 - label as color.
[Open3D INFO]                    9 - normal as color.

[Open3D INFO] ---------------- Editing control ----------------
[Open3D INFO]     Ctrl + R                    : Clear selection.
[Open3D INFO]     Shift + mouse left button   : Pick a point and add to selection. If it is
[Open3D INFO]                                   already in the selection it will be removed.
[Open3D INFO]     Shift + mouse left drag     : Defines a rectangle, which will add all the 
[Open3D INFO]                                   points in it to the selection.
[Open3D INFO]     mouse right drag            : Moves selected points.
[Open3D INFO]     Delete / Backspace          : Removes all points in the rectangle from the
[Open3D INFO]                                   selection.

[Open3D INFO] ---------------- Annotation control ----------------
[Open3D INFO]     Space        :        Annotate with current tag.
[Open3D INFO]     N            :        Skip to next tag.
[Open3D INFO]     B            :        Back to previous tag.
[Open3D INFO]     S            :        Save tags.
[Open3D INFO]     R            :        Read tags.
```
