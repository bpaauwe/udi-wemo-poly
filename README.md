# Polyglot v3 Node Server for Wemo Switches

[![license](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/rl1131/udi-wemo-poly/blob/master/LICENSE)

This [Polyglot v3](https://github.com/UniversalDevicesInc/pg3) node server provides an interface between the ISY home automation controller from Universal Devices Inc. and Wemo WiFi switches.

### Installation instructions
1. Got to the Polyglot Store and click "Install"

When the node server starts, it will scan your network to discover Wemo
switch devices and add nodes for each one that if finds.

### Configuration

#### Short Poll
 *  How often to poll the switches to verify status is up-to-date. Default
 is 60 seconds.

#### Long Poll
 * How often to re-scan the network for new devices. Default is 180 seconds.


### Requirements and Attribution

This node server is dependent Greg Dowling's [pywemo](https://github.com/pavoni/pywemo) Python module.
