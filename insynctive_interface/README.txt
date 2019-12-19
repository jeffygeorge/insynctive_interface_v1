To make all insynctive source:
$make all

To clean
$make clean

For itest
$cd itest
$arm-linux-gnueabi-g++ -o itest itest.cpp
For imx28 with timer support: arm-linux-gnueabi-g++ -o itest itest.cpp -lrt