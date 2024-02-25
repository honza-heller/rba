RBA: Robotic Bundle Adjuster
=============================

RBA is a command line tool for performing camera calibration, hand-eye calibration, hand-eye-robot-world calibration and robotic calibration. The initial hand-eye calibration is performed using classical linear estimator. The final fine-tuning of the calibration is formulated as a bundle adjustment problem and the hand-eye (and robot-world) calibration is further optimized using world 3D points visible in the camera.

Compilation
-----------

The `rba` command line tool can be compiled using a provided `cmake` script. Currently, Linux-type operating systems are supported. For further convenience, a docker file is available to provide a Ubuntu 22.04 build environment. 

Assuming the `${RBA_ROOT}` BASH variable points to the root directory of the RBA source tree, the following commands can be used to compile the `rba` tool:

```
$ cd ${RBA_ROOT}/docker/ubuntu-22.04
$ docker build -t rbabe-22.04 .
$ docker run --rm -it --user $(id -u):$(id -g) -v ${RBA_ROOT}:/rba/ rbabe-22.04 /bin/bash

# cd /rba/
# mkdir build
# cd build
# cmake ..
# make
```

The `rba` executable will appear as `/src/build/src/rba`.

```
# /rba/build/src/rba
This is RBA, v0.4.0

Command line options only:
  -h [ --help ]                         produce help message
  --version                             print version string
  -c [ --config ] arg                   name of a configuration file
  --verbose [=arg(=1)]                  Print verbose runtime information
...
```

Execution
---------

The `rba` is a dynamically linked executable, so the dependencies need to be available on the target system. Alternatively, the dependencies could be copied out from the docker environment. The following will copy the dependencies into the `${RBA_ROOT}/build/src` directory from the docker shell:

```
# cd /rba/build/src/
# ldd ./rba | sed '/=>/!d' | sed 's/^.*=> \([^ ]*\).*/\1/' | xargs -n 1 sh -c 'cp $0 .'
```

On the host system, the `rba` executable can then be eecuted as

```
$ LD_LIBRARY_PATH=${RBA_ROOT}/build/src/ ${RBA_ROOT}/build/src/rba
This is RBA, v0.4.0

Command line options only:
  -h [ --help ]                         produce help message
  --version                             print version string
  -c [ --config ] arg                   name of a configuration file
  --verbose [=arg(=1)]                  Print verbose runtime information
...
```

Matlab 
------

The Matlab interface to the `rba` tool is called `mrba` and can be found in the `${RBA_ROOT}/mrba` directory. The `${RBA_ROOT}/mrba/examples` contains examples of camera and robot calibrations. These can be easily executed from the matlab environment:

```
>> rba_root = getenv('RBA_ROOT');
>> addpath([rba_root filesep 'mrba']);
>> addpath([rba_root filesep 'mrba/examples'])

>> ccalib = camcalib_exp1;
>> ccalib.dist
ans =
    0.1618   -0.4137    0.0008   -0.0003         0         0         0         0
>> ccalib.K
ans =
   1.0e+03 *
    2.5858         0    1.6245
         0    2.5858    0.9783
         0         0    0.0010

>> hec = heccalib_exp2;
>> hec{1}.X
ans =
    0.9996   -0.0252    0.0096 -106.0805
    0.0256    0.9986   -0.0464  166.7559
   -0.0084    0.0466    0.9989   30.8570
         0         0         0    1.0000