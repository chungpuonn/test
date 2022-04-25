# Install ROS, input is ros version
installROSKinetic() 
{
    #using japan server
    sudo sh -c 'echo "deb http://packages.ros.org.jsk.imi.i.u-tokyo.ac.jp/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://ros.jsk.imi.i.u-tokyo.ac.jp/jsk.key -O - | sudo apt-key add -

    sudo apt -qq update
    
    sudo apt --assume-yes install ros-kinetic-ros-base
   # echo "source /opt/ros/kinetic/setup.bash" >>~/.bashrc
   # source ~/.bashrc
   # sudo apt --assume-yes install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
   # sudo rosdep init && rosdep update
}

installROSMelodic()  #working tested
{
    #using japan server
    sudo sh -c 'echo "deb http://packages.ros.org.jsk.imi.i.u-tokyo.ac.jp/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    #wget http://packages.ros.org.jsk.imi.i.u-tokyo.ac.jp/jsk.key -O - | sudo apt-key add -
    wget http://ros.jsk.imi.i.u-tokyo.ac.jp/jsk.key -O - | sudo apt-key add -
    sudo apt -qq update
    
    sudo apt --assume-yes install ros-melodic-ros-base
    sudo apt --assume-yesinstall ros-melodic-amcl
    sudo apt --assume-yesinstall ros-melodic-robot-pose-ekf ros-melodic-move-base ros-melodic-map-server

    echo "source /opt/ros/melodic/setup.bash" >>~/.bashrc
    source ~/.bashrc
    sudo apt --assume-yes install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo rosdep init && rosdep update
}

installUtilities()
{
    echo "Installing dependencies"
    sudo apt-get --assume-yes install openssh-server && sudo systemctl enable ssh && sudo systemctl start ssh
    sudo apt-get --assume-yes install exfat-fuse exfat-utils git xclip setserial libgoogle-glog* libxmlrpc-c++* fonts-powerline powerline
    # patch fonts
    mv PowerlineSymbols.otf /usr/share/fonts/
    #give access to servial 
    sudo gpasswd --add ${USER} dialout
    # installed vim and vim packages
    sudo apt-get --assume-yes remove vim vim-runtime gvim
    sudo apt install  --assume-yes libncurses5-dev libgnome2-dev libgnomeui-dev \
    libgtk2.0-dev libatk1.0-dev libbonoboui2-dev \
    libcairo2-dev libx11-dev libxpm-dev libxt-dev python-dev \
    python3-dev ruby-dev lua5.2 liblua5.2-dev libperl-dev 
    git clone https://github.com/vim/vim.git
    cd vim
    ./configure --with-features=huge --enable-multibyte\
	    --enable-rubyinterp=yes --enable-python3interp=yes\
	    --with-python3-config-dir=$(python3-config --configdir)\
	    --enable-perlinterp=yes --enable-luainterp=yes\
	    --enable-cscope --prefix=/usr/local
    make VIMRUNTIMEDIR=/usr/local/share/vim/vim82
    sudo make install
    cd ..
    sudo rm -r vim/
    
    #install vundle for vim 
    git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim
    cp .vimrc ~/
    
    vim +PluginInstall +qa
    
    #install others followed
    sudo apt --assume-yes install $@

}

changeNetWorkSetting()
{

    sudo echo "auto eth0" > /etc/network/interfaces
    sudo echo "iface eth0 inet static" > /etc/network/interfaces
    sudo echo "address 192.168.10.123" > /etc/network/interfaces
    sudo echo "netmask 255.255.255.0" > /etc/network/interfaces
    sudo echo "gateway 192.168.10.1" > /etc/network/interfaces

}

moveSettingFiles()
{
    sudo cp 40-usb-serial.rules /etc/udev/rules.d/
    cp .vimrc ~/.vimrc
    cp .bashrc ~/.bashrc

    sudo udevadm trigger #load the rules
}

installVIM()  #working
{
    cd ~
    git clone https://github.com/vim/vim.git
    cd vim
    sudo apt-get install ncurses-dev
    make clean distclean
    ./configure --with-features=huge --enable-multibyte\
	    --enable-rubyinterp=yes --enable-python3interp=yes\
        --with-python3-command=python3\
	    --with-python3-config-dir=$(python3-config --configdir)\
	    --enable-perlinterp=yes --enable-luainterp=yes\
	    --enable-cscope
    make VIMRUNTIMEDIR=/usr/local/share/vim/vim82
    #sudo make uninstall
    sudo make install
    cd ..
   # sudo rm -r vim/
}

installCartographer() #working tested 
{
  cd ~  
  mkdir -p ~/depend_ws && cd ~/depend_ws
  sudo apt update && sudo apt --assume-yes install python-wstool python-rosdep ninja-build stow &&
  wstool init src
  wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall

  wstool update -t ~/depend_ws/src
  rosdep update --include-eol-distros
  rosdep install --from-paths ~/depend_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y
  if ["$ROS_DISTRO" == "kinetic" ] ; then
    bash ~/depend_ws/src/cartographer/scripts/install_proto3.sh
    fi
  bash src/cartographer/scripts/install_abseil.sh
  sudo apt remove ros-${ROS_DISTRO}-abseil-cpp

  catkin_make_isolated --install --use-ninja

  echo "source ~/depend_ws/devel_isolated/setup.bash --extend" >> ~/.bashrc
  cd ~

}
# installROS && installUtilities
