# -*- mode: ruby -*-
# vi: set ft=ruby :

$adol_c = <<SCRIPT
  sudo apt-get update

  # dev tools
  sudo apt-get install -y \
    build-essential \
    automake autoconf

  # gitlab ci
  sudo apt-get install -y -qq \
    libcolpack-dev \
    libboost-all-dev

  # adol-c
  sudo apt-get install -y \
    texlive-full

  cd /vagrant/optimisationRefactoring/differentiatedCAD/adol-c
  ./update_versions.sh
  ./configure --prefix=$(cd .. && pwd)/adolc_base
  automake
  make
  make install 
SCRIPT

$occt = <<SCRIPT
  sudo apt-get update

  # OCCT
  sudo apt-get install -y \
    cmake cmake-curses-gui \
    tcl tcl-dev \
    tk tk-dev \
    libfreetype6 \
    libfreeimage3 libfreeimage-dev \
    libadolc2 libadolc-dev

  mkdir -p /vagrant/optimisationRefactoring/differentiatedCAD/build
  mkdir -p /vagrant/optimisationRefactoring/differentiatedCAD/install

  cd /vagrant/optimisationRefactoring/differentiatedCAD/build
  cmake -G 'Unix Makefiles' ../occt-min-topo-src/
  make -j$(nproc)
  make install
SCRIPT

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu/xenial64"
  config.vm.provider "virtualbox" do |vb|
    vb.memory = "2048"
    vb.cpus = "2"
  end

  config.vm.provision "adol-c", type: "shell", inline: $adol_c
  config.vm.provision "occt", type: "shell", inline: $occt
end
