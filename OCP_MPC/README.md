## For ACADOS
Installation: https://docs.acados.org/installation/index.html

Link to problems with installation: https://discourse.acados.org/t/acados-installation-in-pycharm/103

## Bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib
export ACADOS_SOURCE_DIR=$HOME/acados


## installation
cd ~/acados
git submodule update --init --recursive
rm -rf build && mkdir build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=$HOME/acados_installed \
  -DBUILD_PYTHON_INTERFACE=ON        \
  -DBLASFEO_TARGET=GENERIC           \
  -DACADOS_WITH_QPOASES=ON           \
  -DACADOS_EXAMPLES=ON               \
  -DHPIPM_TARGET=GENERIC             \
  -DACADOS_WITH_BLAS_API=CBLAS        \
  -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
make install

## run a C example, e.g.:
./examples/c/sim_wt_model_nx6

## go back to ~/acados and find interfaces folder, then go into acados template like this:
pwd: ~/acados/interfaces/acados_template 
run pip install .
