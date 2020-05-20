# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.1] - 2020-04-08

### Added
- Added the possibility in `wholeBodyDynamics` device to set the assumptions about the external wrench type (full wrenches, pure forces or pure forces with known direction) from the configuration file. (See[!48](https://github.com/robotology/whole-body-estimators/pull/48)).

## [0.2.0] - 2020-03-11

- Adding the following YARP devices: 
* `genericSensorClient`
* `virtualAnalogClient`
* `virtualAnalogRemapper`
* `wholeBodyDynamics`

## [0.1.0] - 2020-03-11

Initial release of whole-body-estimators, containing the following YARP devices: 
* `baseEstimatorV1`
