# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.1.1] - 2019-07-30

### Added

* SOUNDEFFECT for language, symbol change, microphone, AI button presses
* Button press debounce on mode, numeric, language change

### Removed

* `while(!Serial)` loop as HID device didn't start as long as Serial monitor wasn't started
* Implemented mic functionality for when device is sleeping

### Changed

* Baud rate from 115200 to 9600 (`Serial.begin()` is dummy method)

## [1.1.0] - 2019-07-22

### Added

* Symbol layer and modifier layer as shown in Swipo application keyboard layout
* Getter functions for language, symbol, modifier layers and pad sectors
* Implement keycode formatting

### Changed

- Change Serial Port baud rate from 9600 to 115200

### Removed

* Commented useless test code

* Unneeded Serial.print

  

## [1.0.2] - 2019-05-13

### Changed

* Implement  partial Swipo application keyboard layout using "Swipo_ButtomInterface_Spec_V2.pdf" as a reference

  

## [1.0.1] - 2019-05-03

### Added

* Output pointer over symbol to Serial Port Monitor

### Removed
* Commented useless test code

