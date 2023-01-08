pip install pybind11
pip install "pybind11[global]"
export PATH=/home/user/.local/bin:$PATH
mkdir build 
cd build
cmake ..
make 
cd ..

