#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
# git clone https://gitee.com/xiacanming/osqp-0.5.0.git
# wget https://github.com/osqp/osqp/archive/refs/tags/v0.5.0.tar.gz
# tar xzf v0.5.0.tar.gz
git clone --recursive --branch v0.5.0 --single-branch https://github.com/osqp/osqp.git


pushd osqp
mkdir build && cd build
cmake ../
make
make install
popd

# git clone https://gitee.com/xiacanming/osqp-eigen-0.4.1.git
# wget https://github.com/robotology/osqp-eigen/archive/refs/tags/v0.4.1.tar.gz
# tar xzf v0.4.1.tar.gz
git clone --recursive --branch v0.4.1 --single-branch https://github.com/robotology/osqp-eigen

pushd osqp-eigen
mkdir build && cd build
cmake ../
make
make install
popd

#Clean
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -fr osqp
rm -fr osqp-eigen
