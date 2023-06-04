rm -r build
mkdir build
pushd build
    cmake ..
    cmake --build .
popd