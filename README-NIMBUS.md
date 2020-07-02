# **Binaries**
Pre-built binaries for various boards are hosted inside the `software/Pixhawk_builds` directory inside NimbusLab's Box.

# Cloning
When cloning this repository, fetch all submodules (thanks Pedro):
`git clone --recurse-submodules <repository>`

# *Building on Debian*
Getting ardupilot to build on Debian is a little more involved, and the people
behind the show seem to like Ubuntu better.
If using the usual installation script on a Debian machine, read on.

```
# want to check if this is a Debian install?
if lsb_release -a 2> /dev/null | grep -c "Debian"; then
  echo "yay";
else
  echo "nay"
fi
```
Tell dpkg that you will work with a new architechture:
```
# set architecture for cross-compiling
sudo dpkg add-architecture armhf

# update repositories to reflect new architecture tools
sudo apt-get update
```
then install a bunch of packages:
```
## "pkg-config-arm-linux-gnueabihf" referenced in the install script is NOT a
## debian package (up until 9.5 Stretch). Be decent and install base debian 
## packages first (it'll also pull in a bunch of other stuff - choose wisely, or
## get more disk space)
sudo apt-get install crossbuild-essential-armhf

## Installing binutils is not critical (maybe?), but it helps with the crossbuild stuff
## NOTE: this will pull in build tools for Fortran, Go, ObjC ... bye bye disk space :)
sudo apt-get install binutils-arm-none-eabi
sudo apt-get install gcc-arm-none-eabi --no-install-recommends
```
NOW you can run the install prereq script. Removed a couple of things from there:
`pkg-config-arm-linux-gnueabihf` (like I said, NOT A DEBIAN PACKAGE)
`python-opencv` (don't need more of that, ROS install brings it in)


