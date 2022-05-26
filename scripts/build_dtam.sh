cd code/build
cmake .. -GNinja
cmake --build . --parallel 8
# make -j8
cd ..
