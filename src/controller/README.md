Proto File is MIT Licensed and sourced from Fuzzy Logic gitlab here : https://gitlab.com/fuzzylogicrobotics-examples/fuzzy-rtos-example-code/-/tree/main/UDP?ref_type=heads

This programm was made by Clement Coudoin during his internship with the MFJA and LAAS-CNRS in Toulouse. It is used to communicate with and control a Staubli TX2-60L robotic arm with a Fuzzy Logics's RTOS : https://fuzzylogicrobotics.gitlab.io/fuzzy_docs/docs/4.12.0/fuzzy-rtos/

A second folder is dedicated to feedback force and moments data.(This part will be precised later)

To try the programm :
1. Clone the repository
git clone https://github.com/CCoudoin/devel.git
cd devel/controller

2. Build the project with Cmake
- mkdir build
- cd build
- cmake ..
- cmake --build .

3. Run the programm
- ./main

Dependencies :
- Protobuf
- Cmake >= 3.31
- C++ >= 20
- Linux System
