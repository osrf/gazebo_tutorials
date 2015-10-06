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
To install the prerelease, first use the
instructions above to install the stable repository and after it add the
prerelease repository:

```
# Be sure to install the stable repo first !!
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list'
sudo apt-get update
sudo apt-get install gazebo7 # (might not be released)
```

### Gazebo nightly repo

Gazebo nightlies are packages released every night which can be for different
proposes like testing the last feature added to gazebo code. To install the
nightlies, first use the instructions above to install the stable repository
and after it add the nightly repository:

```
# Be sure to install the stable repo first !!
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list'
sudo apt-get update
sudo apt-get install gazebo7
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

Nightly use the following versioning scheme: `{current_released_version}+hg{date}r{hash}-{nightly_revision}`

 * `current_released_version:` will be the latest version released available in
   the changelog file of the corresponding -release repo. If the nightly is
   used for an upcoming release (lets say gazebo7) then R-1.99.99-1
   (gazebo7_6.99.99-1) form will be used until prereleases or final release.

 * `date`: timestamp YYYY-MM-DD

 * `hash`: mercurial hash corresponding to code HEAD used in the nightly build

 * `nightly_revision`:  revision number to apply to the nightly

### Versions when mixing repositories

How does work a prerelease + stable repositories setup?

 * packageA version: `1.0.0-1` (stable)
 * packageA prerelease: `1.0.1-1~pre1` (prerelease)
 * packageA prerelease: `1.0.1-1~pre2` (prerelease)
 * packageA version: `1.0.1-1` (stable)

 * Order: `1.0.1-1` > `1.0.1-1~pre2` > `1.0.1-1~pre1` > `1.0.0`

How does work a nightly + stable repositories setup?

 * packageA version: `0.99.99+hg20150101r2212b5136299-1` (nightly)
 * packageA version: `1.0.0-1` (stable)
 * packageA version: `1.0.0-1+hg20150303r6912b5136236-1` (nightly)
 * packageA version: `1.0.1-1` (stable)

 * Order: `1.0.1-1` > `1.0.0-1+hg20150303r6912b5136236-1` > `1.0.0-1` > `0.99.99+hg20150101r2212b5136299-1`

How does work a nightly + prerelease + stable repositories setup?

 * packageA version: `0.99.99+hg20150101r2212b5136299-1` (nightly)
 * packageA prerelease: `1.0.0-1~pre1` (prerelease)
 * packageA version: `1.0.0-1~pre1+hg20150101r2212b5136299-1` (nightly)
 * packageA prerelease: `1.0.0-1~pre2` (prerelease)
 * packageA version: `1.0.0-1` (stable)

 * Order: `1.0.0-1` > `1.0.0-1~pre2` > `1.0.0-1~pre1+hg20150101r2212b5136299-1` > `1.0.0-1~pre1` > `0.99.99+hg20150101r2212b5136299-1`
