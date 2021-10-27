# configuration-manager
[![Build and Test in Ubuntu](https://github.com/Auterion/configuration-manager/actions/workflows/build_test_ubuntu.yml/badge.svg?branch=master)](https://github.com/Auterion/configuration-manager/actions/workflows/build_test_ubuntu.yml?query=branch%3Amaster) [![Build/deb packaging for Skynode and other archs](https://github.com/Auterion/configuration-manager/workflows/Build/deb%20packaging%20for%20Skynode%20and%20other%20archs/badge.svg?branch=master)](https://github.com/Auterion/configuration-manager/actions/workflows/build_pkg_multi_arch.yaml?query=branch%3Amaster)

## Introduction

configuration-manager is a software that receives a mavlink stream destined to a software component and forward the request to this component through another communication channel like DBUS or socket. The current software interfaces available are:
- autopilot-manager

## Dependencies

configuration-manager depends on the following packages:

```
apt install \
  libdbus-1-dev \
  libglib2.0-dev
```

You also need to install mavsdk:  https://mavsdk.mavlink.io/develop/en/getting_started/installation.html

## Build

You can build configuration-manager with the following commands:

```
mkdir -p build
cd build
cmake ..
make
```

You can package configuration-manager as a debian package with the following commands:

```
dpkg-buildpackage -us -uc -nc -b
```

## Installation

In case you built configuration-manager, you can run the following command to install it:

```
dpkg -i configuration-manager*.deb
```

## Run

You can run configuration-manager with the following command:

```
configuration-manager
```
