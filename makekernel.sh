export CROSS_COMPILE='/home/vlw/gcc-linaro_4.8.4-2014.114/bin/arm-eabi-'
export ARCH=arm

make O=./out lineageos_a3_defconfig -j5 
make O=./out -j5
