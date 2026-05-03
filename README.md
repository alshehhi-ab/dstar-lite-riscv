# D* Lite for FPGAs

![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=flat&logo=c%2B%2B&logoColor=white)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![RISC-V](https://img.shields.io/badge/Arch-RISC--V-EF3D30?style=flat&logo=riscv&logoColor=white)
![Status](https://img.shields.io/badge/Status-Active_Research-success)

This repository contains a high-performance implementation of the **D**Lite* path-planning algorithm, specifically optimised for **RV32I/IMC** RISC-V soft-cores running on **FPGA** hardware.


## Overview

Dynamic path planning is a fundamental yet computationally expensive task in autonomous robotics. This project addresses these bottlenecks through a hardware-software co-design approach, implementing D* Lite in C++ for resource-constrained environments. By utilising fixed-point arithmetic and custom ISA extensions, this implementation achieves the low-latency response required for real-time navigation.

## Repository Structure & Branches

This project is divided into branches based on the level of hardware integration:

* **`main`**: The core C++ implementation of D* Lite using purely fixed-point integer arithmetic. This branch is target-agnostic for RISC-V IMC cores.
* **`feature/hw-accel`**: Contains the code optimised for **RV32IMC extensions**. This branch includes the specialised assembly/intrinsics required to interface with custom FPGA hardware logic for priority queue acceleration.
* **`research/profiling`**: Dedicated to performance analysis, containing scripts and benchmarks used to evaluate the speedup of the hardware-accelerated version versus the software-only baseline.

## Theoretical Background

The **D** Lite* algorithm, developed by **Sven Koenig** and **Maxim Likhachev**, is an incremental heuristic search method. Unlike static planners, D* Lite efficiently repairs previous search results when obstacles are detected, avoiding the need to plan from scratch.

### Key Features
* **Incremental Replanning**: Handles dynamic world changes efficiently.
* **Fixed-Point Arithmetic**: Pure Fixed-Point implementation of the D* Lite alorithm, reducing the need for floating-point hardware support.
* **Configurable Heuristics**: Supports Manhattan, Octile, and Chebyshev distances.


## Hardware Architecture

The implementation targets a RISC-V environment with the following specs:
* **Core**: RV32IMC (Integer, Multiply, Compressed).
* **Custom ISA Extensions**: Moves bottleneck operations like priority calculations into dedicated hardware logic (see the `hw-accel` branch).
* **Target Platform**: RV32IMC on FPGA

## Target Platform

This project uses a custom FPGA-based RISC-V platform built around the **lowRISC Ibex** core configured for an RV32IMC-style environment.


- Ibex repository: https://github.com/lowRISC/ibex
- Ibex documentation: https://ibex-core.readthedocs.io/
- Ibex demo system: https://github.com/lowRISC/ibex-demo-system

## Getting Started

### Prerequisites
* **RISC-V Toolchain**: `riscv64-unknown-elf-gcc` or similar cross-compiler.
* **Hardware/Simulator**: An FPGA development board or a cycle-accurate simulator (e.g., Verilator) supporting the RV32IMC extension.

### Build Instructions
```bash
# Clone the repository
git clone https://github.com/alshehhi-ab/dstar-lite-fpga.git
cd dstar-lite-fpga

# Build the project
make
```

To clean the build:
```bash
make clean
```


## Academic References

If you use this work in your research, please cite the original D* Lite paper:

> Koenig, S., & Likhachev, M. (2002). **D* Lite**. *Proceedings of the Eighteenth National Conference on Artificial Intelligence*, 476–483.

```bibtex
@inproceedings{koenig2002dstar,
  author = {Koenig, Sven and Likhachev, Maxim},
  title = {D* Lite},
  booktitle = {Proceedings of the Eighteenth National Conference on Artificial Intelligence},
  year = {2002},
  pages = {476--483},
  publisher = {AAAI Press},
  address = {Edmonton, Alberta, Canada}
}
```

### Additional Papers:
```bibtex
@INPROCEEDINGS{koenig2002improved,
  author={Koenig, S. and Likhachev, M.},
  booktitle={Proceedings 2002 IEEE International Conference on Robotics and Automation (Cat. No.02CH37292)}, 
  title={Improved fast replanning for robot navigation in unknown terrain}, 
  year={2002},
  volume={1},
  number={},
  pages={968-975 vol.1},
  keywords={Navigation;Robot kinematics;Mobile robots;Motion planning;Costs;Land vehicles;Software prototyping;Prototypes;Educational institutions;Search methods},
  doi={10.1109/ROBOT.2002.1013481}}
```

### Repository BibTeX
```bibtex
@software{alshehhi_dstar_fpga_2026,
  author = {Alshehhi, A.},
  title = {D* Lite for FPGAs: Hardware-Accelerated Path Planning},
  year = {2026},
  url = {https://github.com/alshehhi-ab/dstar-lite-fpga},
}
```
 

## Authors
- [@alshehhi-ab](https://www.github.com/alshehhi-ab)


## License
This project is licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0).
