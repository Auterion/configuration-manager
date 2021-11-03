<img align="right" height="20" src="https://auterion.com/wp-content/uploads/2020/05/auterion_logo_default_sunrise.svg">

# Configuration Manager
[![Build and Test in Ubuntu](https://github.com/Auterion/configuration-manager/actions/workflows/build_test_ubuntu.yml/badge.svg?branch=master)](https://github.com/Auterion/configuration-manager/actions/workflows/build_test_ubuntu.yml?query=branch%3Amaster)

## Introduction

The Configuration Manager is a software service that uses MAVSDK and MAVLink in order to setup parameters in Software components that do not communicate through MAVLink. This is achieved by MAVLink Parameter Transaction microservice (WIP and to be documented in the spec), and the transaction happens through another communication protocol channel, like DBUS or UDS. Currently the service uses DBUS and provides a way to configure the parameters of the [Autopilot Manager service](https://github.com/Auterion/autopilot_manager), which means this service is a dependency of the Autopilot Manager service for correct parameter setup.

Note that a ground station-type of MAVLink endpoint should be capable of setting and initializing the transaction for the different parameters to be configured on the end software component. Auterion Mission Control already provides this implementation, while the Parameter Transaction is not standerdized as a microservice in the spec, so to be then used in QGroundcontrol and in MAVSDK. Please contact Auterion if you want to know more about Auterion Mission Control.

### The Parameter Transaction MAVLink microservice

This microservice is still work-in-progress and will soon be documented on the MAVLink spec so the parameter transaction initialization can be initialized. Though the respective messages are already available in the `development` dialect:

* [PARAM_TRANSACTION_TRANSPORT enum](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/development.xml#L48-L68)
* [MAV_CMD_PARAM_TRANSACTION command](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/development.xml#L75-L80)
* [PARAM_ACK_TRANSACTION](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/development.xml#L126-L134)

## Dependencies

The _configuration-manager_ depends on the following packages:

```
apt install \
  libdbus-1-dev \
  libglib2.0-dev
```

One also needs to install MAVSDK. Follow this guide for how to: https://mavsdk.mavlink.io/develop/en/getting_started/installation.html

## Build

One can build _configuration-manager_ with the following commands:

```
mkdir -p build
cd build
cmake ..
make
```

## Packaging

One can package _configuration-manager_ as a debian package with the following commands:

```
dpkg-buildpackage -us -uc -nc -b
```

**Note before packaging:** the DBUS configuration should include the username of the system where the _configuration-manager_ is running so the user has the appropriate policies to access DBUS through it. Make sure to add the policy as following to the `com.auterion.configuration_manager.conf` file before packaging, so it then gets deployed with the right policies to `/usr/share/dbus-1/system.d/`. Example bellow:

```xml
<!DOCTYPE busconfig PUBLIC
 "-//freedesktop//DTD D-BUS Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy user="root">
    <allow own="com.auterion.configuration_manager"/>
    <allow send_destination="com.auterion.configuration_manager"
        send_interface="com.auterion.configuration_manager.interface"/>
  </policy>
  <!-- Add your user here -->
  <!-- example user -->
  <policy user="user1"> <!-- replace "user1" with your system username -->
    <allow own="com.auterion.configuration_manager"/>
    <allow send_destination="com.auterion.configuration_manager"
        send_interface="com.auterion.configuration_manager.interface"/>
  </policy>
  <!--**************-->
</busconfig>
```

## Installation

In case one built the _configuration-manager_, you can run the following command to install it system-wide:

```
dpkg -i configuration-manager*.deb
```

## Run

Run the _configuration-manager_ with the following command:

```
configuration-manager
```
