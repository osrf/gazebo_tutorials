# Install Gazebo on Linux distributions (non Ubuntu)

Gazebo is available to install on other Linux distributions different than Ubuntu
([Ubuntu install instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
are hosted in a different tutorial). Linux distributions providing gazebo packages:
Debian, Fedora, Arch and Gentoo.

## Debian

[Gazebo in Debian](https://packages.debian.org/source/sid/gazebo) is available
as an official package since Stretch. Debian Sid (the Gazebo team is the official
maintainer in Debian) usually hosts the latest gazebo release. Depending on the
Debian version the version of Gazebo available is different:

 * Debian Sid is usually hosting one of the latest releases
 * Debian Buster: gazebo-9.6.0
 * Debian Bullseye: gazebo-11.1.0

1. Install Gazebo11 (on Bullseye)

        sudo apt-get install gazebo11
        # For developers that works on top of Gazebo, one extra package
        sudo apt-get install libgazebo11-dev

## Fedora

[Gazebo in Fedora](https://apps.fedoraproject.org/packages/gazebo) is available
as an official package.

 * Rawhide: gazebo-10
 * Fedora 35: gazebo-10.x
 * Fedora 36: gazebo-10.x

1. Install Gazebo
        sudo dnf install gazebo gazebo-ode
        # For developers that works on top of Gazebo, one extra package
        sudo dnf install gazebo-devel

## Arch

[Gazebo in Arch](https://aur.archlinux.org/packages/gazebo/) is currently in the AUR:
Arch User Repository (billypilgrim is the maintainer). This means that it
is not in the official package repositories and users need to compile it from
source. The easiest way to install it is to use an AUR helper, such as yaourt
or packer:

1. Install Gazebo11

        yaourt -S gazebo
        # or
        sudo packer -S gazebo

## Gentoo

[Gazebo11 in Gentoo](https://packages.gentoo.org/package/sci-electronics/gazebo)
is available as an official package (the maintainer is Alexis Ballier). It is
currently masked as ~amd64 so please read about how to [mix software
branches](https://wiki.gentoo.org/wiki/Handbook:AMD64/Portage/Branches) if you
are using stable.

Some *use flags* are available to customize the package, use equery (from
gentoolkit) to know more about the optional support:

      emerge --ask app-portage/gentoolkit
      equery uses gazebo -a

1. Install Gazebo11 on stable branch

        echo "sci-electronics/gazebo" >> /etc/portage/package.accept_keywords
        emerge gazebo

1. Install Gazebo on testing branch

        emerge gazebo

## Other linux distributions?

There is a large list of Gazebo packages available in many different
distributions, checking the great
[repology](https://repology.org/project/gazebo/versions) project can help.
