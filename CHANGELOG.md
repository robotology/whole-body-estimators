# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.1] - 2020-04-08
### Fixed
- Fixed launch-wholebodydynamics-`*`.xml configuration files in order to properly open `wholebodydynamics` device without silently skipping it. (See [!56](https://github.com/robotology/whole-body-estimators/pull/56)).
- Fixed a few YARP related and iDynTree related deprecations in `wholebodydynamics`, `genericSensorClient` and `baseEstimatorV1`. (See [!58](https://github.com/robotology/whole-body-estimators/pull/58)).
- Fixed a bug in `wholebodydynamics` related to hard-coded cut-off frequency parameters used for low-pass filtering measurements. (See [!63](https://github.com/robotology/whole-body-estimators/pull/63)).

### Added
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
