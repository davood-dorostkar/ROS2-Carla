#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# sudo apt-get install -y cppad gfortran 
sudo apt-get update && apt-get install -y gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev cppad
# wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip -O Ipopt-3.12.8.zip
# unzip Ipopt-3.12.8.zip
# git clone --recursive --branch releases/3.12.8 --single-branch https://github.com/coin-or/Ipopt.git
# wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.tgz
# tar xzf Ipopt-3.12.8.tgz && mv Ipopt-3.12.8 Ipopt

# Step by step   
# cd Ipopt/ThirdParty
# cd Blas && ./get.Blas    
# cd ../Lapack && ./get.Lapack    
# cd ../Mumps && ./get.Mumps    
# cd ../Metis && ./get.Metis    
# cd ../ASL && ./get.ASL  

git clone https://github.com/coin-or-tools/ThirdParty-ASL.git
cd ThirdParty-ASL
./get.ASL
./configure
make -j6
make install

cd ..
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
./get.Mumps
./configure
make -j6
make install

cd ..
git clone https://github.com/coin-or-tools/ThirdParty-Blas.git
cd ThirdParty-Blas
./get.Blas
./configure
make -j6
make install
cp -a lib/* /usr/lib/. 

cd ..
git clone https://github.com/coin-or-tools/ThirdParty-Lapack.git
cd ThirdParty-Lapack
./get.Lapack
./configure
make -j6
make install
cp -a lib/* /usr/lib/. 

cd ..
git clone https://github.com/coin-or-tools/ThirdParty-Metis.git
cd ThirdParty-Metis
./get.Metis
./configure
make -j6
make install
# cp -a lib/* /usr/lib/. 


cd ..
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt
mkdir build  
cd build  
../configure  
make -j6  
make install  

# cp -a include/* /usr/include/.  
# cp -a lib/* /usr/lib/. 

# Clean up.
cd ../..
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -rf Ipopt ThirdParty*
