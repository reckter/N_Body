#!/bin/sh


echo ">>>$1<<<"


echo "\nbuilding without anything"
cmake -DBUILD_QT=OFF -DUSE_OPENMP=OFF $1 > /dev/null &&
make clean > /dev/null &&
make > /dev/null &&

echo "\nnbody:"
./bin/praktikum4 nbody.txt nbody_out.txt
echo "\nrandom:"
./bin/praktikum4 random.txt random_out.txt
echo "\ncollision:"
./bin/praktikum4 collision.txt collision_out.txt


echo "\nbuilding with OpenMP"
cmake -DBUILD_QT=OFF -DUSE_OPENMP=ON $1> /dev/null &&
make clean > /dev/null &&
make > /dev/null &&

echo "\nnbody:"
./bin/praktikum4 nbody.txt nbody_MP_out.txt
echo "\nrandom:"
./bin/praktikum4 random.txt random_MP_out.txt
echo "\ncollision:"
./bin/praktikum4 collision.txt collision_MP_out.txt
