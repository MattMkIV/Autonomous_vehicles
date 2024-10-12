# Assignments of Autonomous Vehicles

This repository contains all the exercises assigned in the course "Platforms and Algorithms for Autonomous Vehicles."
## Requirements

### Operating System
The exercises were tested on different Ubuntu distributions:
- Ubuntu 18.04 (server ARM version)
- Ubuntu 24.04 (desktop ARM version)

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

## Compilation
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
  ./cluster_extraction ../dataset_1
```