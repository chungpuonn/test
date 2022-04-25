# Installation

1. Unmet dependency when installing packagese
   * Possilbe reason: the repositories of the source of the packages is not properly set, or boost has a different name with the one inside the repository source.
   * Possible solution

   ~~~bash
   apt install libboost-dev
   ~~~

2. /dev/urandom is missing
   * Possible reason:/dev/urandom cannot be accessed by non-root user
   * possible solution

   ~~~bash
   echo 'chmod 666 /dev/urandom' > \etc\rc.local
   ~~~

   * or

   ~~~bash
   sudo chown -R USERNAME /dev/urandom
   ~~~

3. Compiling python

   ~~~bash

      tar -xf pythonxxx.tar.xz
      ./configure --prefix=/usr   --enable-loadable-sqlite-extensions   --enable-shared ..  --with-lto   --enable-optimizations   --with-system-expat   --with-system-ffi   --enable-ipv6

   ~~~

4. Change Ownership of folder

    ~~~bash
    sudo chown -R USERNAME /PATH/TO/FOLDER
    ~~~

5. Unzip compressed file

    ~~~bash
    tar xjf file.tar.bz2
    tar -zxvf file.tar.gz
    ~~~

6. CMake Configuration for DUAL ABI

   ~~~cmake
   set(CMAKE_CXX_FLAGS,"${CMAKE_CXX_FLAGS} -D_CLIBCXX_USE_CXX11_ABI=0")
   ~~~

7. Install dependencies for ros packages

   ~~~
   rosdep install --from-paths src --ignore-src -r -y
   ~~~

8. HwHiAiUser has no permission to do xxx

   ~~~bash
     sudo echo 'HwHiAiUser ALL=(ALL) ALL' > /etc/sudoers
   ~~~

9. E: gnupg, gnupg2 and gnupg1 do not seem to be installed, but one of them is required for this operation

   ~~~bash
      sudo apt install gnupg2
   ~~~

10. Change vscode terminal setting 

   ~~~json
   "terminal.integrated.fontFamily": "courier, PowerlineSymbols"
   ~~~

11. Download hokuyo model for gazebo 

    ~~~bash
    cd ~/.gazebo/models
    wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/hokuyo/
    ~~~
11. How to use Pepperal fuchs laser scanner
   ~~~bash
   gssdp-discover -i <interface> --timeout=3 --target=urn:pepperl-fuchs-com:device:R2300:1
   ~~~
   where <interface> is the network interface connected to pepperal fuchs