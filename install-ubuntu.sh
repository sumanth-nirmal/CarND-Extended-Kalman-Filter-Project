#!/bin/bash

#check if its ubuntu 14.04
RELEASE="$(lsb_release -s -r)"
echo "${RELEASE}"

if [ "$RELEASE" == "14.04" ]; then
  wget http://mirrors.kernel.org/ubuntu/pool/universe/libu/libuv1/libuv1-dev_1.8.0-1_amd64.deb
  sudo dpkg --install libuv1-dev_1.8.0-1_amd64.deb
  rm -r libuv1-dev_1.8.0-1_amd64.deb

  wget http://mirrors.kernel.org/ubuntu/pool/universe/libu/libuv1/libuv1_1.8.0-1_amd64.deb
  sudo dpkg --install libuv1_1.8.0-1_amd64.deb
  rm -r libuv1_1.8.0-1_amd64.deb
else
  sudo apt-get install libuv1-dev libssl-dev
fi

git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
