# Unstable packages: prerelease and nightly (Ubuntu)

## Experimental gazebo packages

***Note:*** prereleases and nightlies are unsupported packages, they should
only be used for development proposes.

Gazebo nightly and prereleases repositories are extensions to the stable
repo which will add unstable builds to be installed by using `apt-get`
in the same way as stable releases. These repositories rely on the
presence of gazebo-stable repository in the system.

## Install and uninstall packages

### Gazebo prerelease repo

Gazebo prerelease versions are those released to test an upcoming release.
The prerelease packages repository is designed to work together with the
stable repository, both need to be installed.

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo10 # (might not be released)
```

### Gazebo nightly repo

Gazebo nightlies are packages released every night which can be used for
different purposes like testing the last feature added to gazebo code. The
nighly packages repository is designed to work together with the stable
repository, both need to be installed.

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo10 # (might not be released)
```

### Remove prereleases and nightly installed packages

Removing the packages installed from the nightly or prerelease repositories
can be accomplished by running:

```
# Remove packages installed from nightly/prerelease repositories
sudo apt-get install aptitude
sudo apt-get remove $(aptitude search -F '%p' '~S ~i ?origin("OSRF nightly*")')
sudo apt-get remove $(aptitude search -F '%p' '~S ~i ?origin("OSRF prerelease*")')
```

For disabling the repositories completely they need to be removed
from the apt sources.list and the package cache updated:

```
# Uninstall repositories
sudo rm -f /etc/apt/sources.list.d/gazebo-{nightly,prerelease}.list
sudo apt-get update
```

## Versioning in nightly and prerelease

### Version schemes

Prerelease versioning scheme: `{upcoming_version}~pre{prerelease_version}`

 * `upcoming_version:` upstream version target for current prerelease series
 * `prerelease_version`: prerelease version number in the series

Nightly use the following versioning scheme: `{current_released_version}+hg{date}+${nightly_revision}r{hash}-{nightly_revision}`

 * `current_released_version:` will be the latest version released available in
   the changelog file of the corresponding -release repo. If the nightly is
   used for an upcoming release (for example, gazebo10) then R-1.99.99-1
   (gazebo10_9.99.99-1) form will be used until prereleases or final release.

 * `date`: timestamp YYYY-MM-DD

 * `hash`: mercurial hash corresponding to code HEAD used in the nightly build.
    Used for information proposes.

 * `nightly_revision`:  revision number to apply to the nightly. It is also
   used to generate a new nightly using the same same date timestamp.

### Versions when mixing repositories

Which version has priority when using prerelease and stable repositories?

 * packageA version: `1.0.0-1` (stable)
 * packageA prerelease: `1.0.1~pre1-1` (prerelease)
 * packageA prerelease: `1.0.1~pre2-1` (prerelease)
 * packageA version: `1.0.1-1` (stable)

 * Order: `1.0.1-1` > `1.0.1~pre2-1` > `1.0.1~pre1-1` > `1.0.0`

Which version has priority when using nightly and stable repositories?

 * packageA version: `0.99.99+hg20150101r2212b5136299-1` (nightly)
 * packageA version: `1.0.0-1` (stable)
 * packageA version: `1.0.0-1+hg20150303r6912b5136236-1` (nightly)
 * packageA version: `1.0.1-1` (stable)

 * Order: `1.0.1-1` > `1.0.0-1+hg20150303r6912b5136236-1` > `1.0.0-1` > `0.99.99+hg20150101r2212b5136299-1`

Which version has priority when using nightly, prerelease and stable repositories?

 * packageA version: `0.99.99+hg20150101r2212b5136299-1` (nightly)
 * packageA prerelease: `1.0.0~pre1` (prerelease)
 * packageA version: `1.0.0~pre1+hg20150101r2212b5136299-1` (nightly)
 * packageA prerelease: `1.0.0~pre2-1` (prerelease)
 * packageA version: `1.0.0-1` (stable)

 * Order: `1.0.0-1` > `1.0.0~pre2-1` > `1.0.0~pre1+hg20150101r2212b5136299-1` > `1.0.0~pre1-1` > `0.99.99+hg20150101r2212b5136299-1`
