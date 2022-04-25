# install and use zsh 
installZsh()
{
    sudo apt update
    sudo apt --assume-yes install zsh
    chsh -s $(which zsh) # this line set zsh as default shell
    # install oh-my-zsh
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"

}

setShell2Bash()
{
    chsh -s $(which bash)
}