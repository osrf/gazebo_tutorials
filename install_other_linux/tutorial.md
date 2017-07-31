# Install Gazebo on Linux distributions (non Ubuntu)

Gazebo is available to install on other Linux distributions different than Ubuntu 
([Ubuntu install instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
are hosted in a different tutorial). Linux distributions providing gazebo packages: 
Debian, Fedora, Arch and Gentoo.

## Debian

[Gazebo in Debian](https://packages.debian.org/source/sid/gazebo) is available
as an official package in Debian Sid (the Gazebo team is the official
maintainer in Debian) which usually hosts the latest gazebo release.

1. Install Gazebo5

        sudo apt-get install gazebo5
        # For developers that works on top of Gazebo, one extra package
        sudo apt-get install libgazebo5-dev

## Fedora

[Gazebo in Fedora](https://apps.fedoraproject.org/packages/gazebo) is available
as an official package (the maintainer is Rich Mattes). Depending on the Fedora
version, the version of Gazebo available is different:

 * Rawhide is usually hosting one of the latest releases
 * Fedora 23: gazebo-5.0.1
 * Fedora 22: gazebo-4.0.2
 * Fedora 21: gazebo-3.1.0

1. Install Gazebo
        sudo yum install gazebo
        # For developers that works on top of Gazebo, one extra package
        sudo yum install gazebo-devel

## Arch

[Gazebo in Arch](https://aur.archlinux.org/packages/gazebo/) is currently in the AUR:
Arch User Repository (Benjamin Chrétien is the maintainer). This means that it
is not in the official package repositories and users need to compile it from
source. The easiest way to install it is to use an AUR helper, such as yaourt
or packer:

1. Install Gazebo

        yaourt -S gazebo
        # or 
        sudo packer -S gazebo

## Gentooo

[Gazebo in Gentoo](https://packages.gentoo.org/package/sci-electronics/gazebo)
is available as an official package (the maintainer is Alexis Ballier). It is
currently masked as ~amd64 so please read about how to [mix software
branches](https://wiki.gentoo.org/wiki/Handbook:AMD64/Portage/Branches) if you
are using stable.

Some *use flags* are available to customize the package, use equery (from
gentoolkit) to know more about the optional support:

      emerge --ask app-portage/gentoolkit
      equery uses gazebo -a

1. Install Gazebo on stable branch
       
        echo "sci-electronics/gazebo" >> /etc/portage/package.accept_keywords
        emerge gazebo

1. Install Gazebo on testing branch

        emerge gazebo

## Other linux distributions?

If you know of any other Linux distribution supporting Gazebo installation,
feel free to [create an issue](https://bitbucket.org/osrf/gazebo_tutorials/issues)
to expand this tutorial.
