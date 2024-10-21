# Assignment 1 - LIDAR

For Ubuntu versions greater than 18.04, you need to add the following import in the `cluster_extraction.cpp` file:

```c++
#include <boost/filesystem.hpp>
```

### Dependences
To properly compile the projects, you need to install the PCL library from here, or you can use the following commands:

```bash
  sudo apt install pcl-tools
  sudo apt install libpcl-dev
```

You also need to install make and cmake:

```bash
  sudo apt install make
  sudo apt install cmake
```

## Build
To compile and run an exercise, navigate to the corresponding folder. If there is a dataset (e.g., _dataset_1.rar_), extract it first. Then, in _cluster_extraction.cpp_, edit the line with the absolute path of the dataset.

Now, you can create the build directory:

```bash
  mkdir build
  cd build
```

Next, create the necessary files for compilation and compile:

```bash
  cmake ..
  make
```

Finally, launch the program:
```bash
  ./cluster_extraction <param>
```

Where param could be:
- 1 to enable the plane rendering
- 0 otherwise