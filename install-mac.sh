#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
OPENSSL_VERSION=`/usr/local/opt/openssl/bin/openssl version -v | cut -d' ' -f2`
OPENSSL_ROOT_DIR=$(brew --cellar openssl)/$OPENSSL_VERSION
OPENSSL_LIBRARIES=$OPENSSL_ROOT_DIR/lib
cmake -DOPENSSL_ROOT_DIR=$OPENSSL_ROOT_DIR -DOPENSSL_LIBRARIES=$OPENSSL_LIBRARIES ..
make
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
