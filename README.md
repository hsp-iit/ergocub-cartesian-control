ergocub-cartesian-control
=====================

This repository contains the code for single arm controllers for the ergoCub and R1 (model R1SN003, the CRIS one) robot.

Documentation
-------------

Full documentation available at [hsp.github.io/ergocub-cartesian-control](hsp.github.io/ergocub-cartesian-control)


Installation
-------------

### Docker
```
docker build -t hsp/ergocub-cartesian-control:latest .
```

### Source

#### Dependencies

- `robotology-superbuild` latest build
- You can also install them with the provided conda environment `environment.yml`

#### Instructions

We recommend installing the repository in the superbuild install directory, but of course you can choose based on your needs

```
git clone https://github.com/hsp-iit/ergocub-cartesian-control.git
cd ergocub-cartesian-control
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=<your-path-to-robotology-superbuild>/build/install
make install
```

CI Status
---------

[![Build Status](https://github.com/hsp-iit/ergocub-cartesian-control/workflows/CI%20Workflow/badge.svg)](https://github.com/hsp-iit/ergocub-cartesian-control/actions?query=workflow%3A%22CI+Workflow%22)

License
---------

[![License](https://img.shields.io/badge/license-BSD--3--Clause%20%2B%20others-19c2d8.svg)](LICENSE)

Maintainers
--------------
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/fbrand-new.png" width="40">](https://github.com/fbrand-new) | [@fbrand-new](https://github.com/fbrand-new) |
| [<img src="https://github.com/PasMarra.png" width="40">](https://github.com/PasMarra) | [@PasMarra](https://github.com/PasMarra) |
| [<img src="https://github.com/fincatomarta.png" width="40">](https://github.com/fincatomarta) | [@fincatomarta](https://github.com/fincatomarta) |
| [<img src="https://github.com/randaz81.png" width="40">](https://github.com/randaz81) | [@randaz81](https://github.com/randaz81) |
