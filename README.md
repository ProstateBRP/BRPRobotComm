# Stand-alone BRP Robot Communication Module

## CMAKE

Change directories of `EIGEN_DIR`, `IGTL_DIR`, `BOOST_DIR`, `SURGICAL_ROBOT_SOURCE_DIR`, `PROSTATE_COMM_SOURCE_DIR` in `testing/src/CMakeLists.txt`.

`EIGEN_DIR`: Eigen library path.

`IGTL_DIR`: OpenIGTLink path. (Optional)

Change Line 3 in `testing/src/Communication/OpenIGTLink/CMakeLists.txt` to your OpenIGTLink build path.

Please use OpenIGTLink_Protocol_Version3
`https://github.com/openigtlink/OpenIGTLink/releases/tag/v3.0v`

`BOOST_DIR`: BOOST library path.

`SURGICAL_ROBOT_SOURCE_DIR`: `PROSTATE_COMM_SOURCE_DIR/testing`

`PROSTATE_COMM_SOURCE_DIR`: Path of this repo.



## Build 

~~~
cd testing
mkdir build
cd build
cmake ../src
make
~~~

## Run

`./CommModuleTest ProstateRobot`

