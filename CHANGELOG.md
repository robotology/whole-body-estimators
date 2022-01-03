# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

## [0.6.1] - 2021-01-03

### Fixed
- Fixed compilation against YARP 3.6 (https://github.com/robotology/whole-body-estimators/pull/135).

## [0.6.0] - 2021-12-03
### Added
- Add a parameter to set the periodicity of the WholeBodyDynamics thread (See [!130](https://github.com/robotology/whole-body-estimators/pull/130)).

### Fixed
- Fixed joint acceleration filtering (See [!124](https://github.com/robotology/whole-body-estimators/pull/124)).
- Fixed `wholeBodyDynamics` device when loaded by the `gazebo_yarp_robotinterface` Gazebo plugin [!126](https://github.com/robotology/whole-body-estimators/issues/126).
- Switch to generate YARP idl files at every build to fix compatibility with YARP master [!131](https://github.com/robotology/whole-body-estimators/pull/131).

### Changed
- Always enable compilation of devices (See [!115](https://github.com/robotology/whole-body-estimators/pull/115))
- Migrate from `RateThread` to `PeriodicThread` in WholeBodyDynamics device (See [!130](https://github.com/robotology/whole-body-estimators/pull/130)).

## [0.5.1] - 2021-07-12
### Added
- Added Conda-based Continuous Integration job (See [!117](https://github.com/robotology/whole-body-estimators/pull/117) and [!118](https://github.com/robotology/whole-body-estimators/pull/118)).

### Fixed
- Fixed compilation with iDynTree 4 (See [!116](https://github.com/robotology/whole-body-estimators/pull/116)).

## [0.5.0] - 2021-05-14
### Added
- Added a `yarp rpc` command `calibStandingWithJetsiRonCubMk1` that performs `calibStanding` for `iRonCub-Mk1` model/robot when the jets are ON and on `idle` thrust. (See [!113](https://github.com/robotology/whole-body-estimators/pull/113)).
- Added an optional config parameter list of fixed joints `additionalConsideredFixedJoints` being omitted by the URDF model parser but are needed to be considered in the joint list. (See [!109](https://github.com/robotology/whole-body-estimators/pull/109)).
- Added some debug prints to detect where the wholeBodyDynamics device hangs at startup. (See [!106](https://github.com/robotology/whole-body-estimators/pull/106)).
- Added possibility to publish net external wrenches. (See [!111](https://github.com/robotology/whole-body-estimators/pull/111)).

### Fixed
- Fixed the configuration files to run wholeBodyDynamics with iCubGazeboV3. (See [!107](https://github.com/robotology/whole-body-estimators/pull/107)).
- Avoid to use getOutputCount before broadcasting data. (See [!108](https://github.com/robotology/whole-body-estimators/pull/108)).

## [0.4.0] - 2021-02-11
### Added
- Add `HW_USE_MAS_IMU` feature in `wholeBodyDynamics` device to enable the use YARP's Multiple Analog Sensor interface based IMU measurements. (See [!99](https://github.com/robotology/whole-body-estimators/pull/99))

### Fixed
- Fixed bug in opening the `wholeBodyDynamics` device while using the `assume_fixed` parameter in the configuration instead of `imuFrameName`. (See [!93](https://github.com/robotology/whole-body-estimators/pull/93)).
- Fixed parsing `checkTemperatureEvery_seconds` parameter in `wholeBodyDynamics` device. (See [!93](https://github.com/robotology/whole-body-estimators/pull/93)).
- Fix `yarp::os::ConstString` deprecations (See [!94](https://github.com/robotology/whole-body-estimators/pull/94))

## [0.3.0] - 2020-11-09
### Added
- Added 'startWithZeroFTSensorOffsets' option when passed starts the WBD device with zero offsets for FT sensors, bypassing the offset calibration. (See [!72](https://github.com/robotology/whole-body-estimators/pull/72)).
- Added support for compensating temperature in Force/Torque sensors readings and for specify user offline offset. (See [!45](https://github.com/robotology/whole-body-estimators/pull/45))
- Added `useSkinForContacts` option flag to enable/disable the usage of tactile skin sensor information for updating contact points and relevant information. (See [!88](https://github.com/robotology/whole-body-estimators/pull/88))
- Added `wholeBodyDynamics` configuration files for `iCubGazeboV3`. (See [!90](https://github.com/robotology/whole-body-estimators/pull/90))

## [0.2.1] - 2020-04-08
### Fixed
- Fixed launch-wholebodydynamics-`*`.xml configuration files in order to properly open `wholebodydynamics` device without silently skipping it. (See [!56](https://github.com/robotology/whole-body-estimators/pull/56)).
- Fixed a few YARP related and iDynTree related deprecations in `wholebodydynamics`, `genericSensorClient` and `baseEstimatorV1`. (See [!58](https://github.com/robotology/whole-body-estimators/pull/58)).
- Fixed a bug in `wholebodydynamics` related to hard-coded cut-off frequency parameters used for low-pass filtering measurements. (See [!63](https://github.com/robotology/whole-body-estimators/pull/63)).

### Added
- Added the possibility in `wholeBodyDynamics` device to set the assumptions about the external wrench type (full wrenches, pure forces or pure forces with known direction) from the configuration file. (See[!48](https://github.com/robotology/whole-body-estimators/pull/48)).
- Github Workflows to check compilation on `ubuntu-latest`, `macOS-latest` and `windows-latest`. (See [!47](https://github.com/robotology/whole-body-estimators/pull/47)).
- Disabled fail-fast strategy for CI/C++ builds to not stop checks completely when one of the build fails (See [!52](https://github.com/robotology/whole-body-estimators/pull/52)).
- Added caching of source-based dependencies for CI/C++ builds to speed up the build workflows (See [!54](https://github.com/robotology/whole-body-estimators/pull/54)).


## [0.2.0] - 2020-03-11

- Adding the following YARP devices:
* `genericSensorClient`
* `virtualAnalogClient`
* `virtualAnalogRemapper`
* `wholeBodyDynamics`

## [0.1.0] - 2020-03-11

Initial release of whole-body-estimators, containing the following YARP devices:
* `baseEstimatorV1`
