pip install pybind11
pip install "pybind11[global]"
export PATH=/home/user/.local/bin:$PATH
rm -rf build
mkdir build 
cd build
cmake ..
make 
cd ..