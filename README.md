# yaw_moment_diagrams
This code calculates the yaw moment and beta diagrams for a bicycle model on a 3d track.

## Explained:
This program demonstrates via a bicycle model how optimal control can be used to calculate yaw moment diagrams.
This method has significant advantages over traditional point-by-point calculation techniques.

## Features:
* A simple tire combined slip tire model
* Several additional outputs including stability, and yaw-damping derivatives.

## How to use:
* The code is run from BicycleMMMMain.m
* BicycleMMMDAE.m has most of the relevant dynamic equations.
* BicycleMMMContinuous.m predominantly hosts BicycleMMMDAE.m
* BicycleMMMEndpoint.m sets the objective function.
* DFyAlf provides the instantaneous cornering stiffness of a tire with a slip input.
* [GPOPS-II](https://www.gpops2.com/) must be installed to solve the optimal control problem.
  * Please refer to the user manual available on their website for information about how the optimal control problem is configured.

## Information
* Extensive information and references can be found in the paper published

## Acknowledgements
* [Professor David JN Limebeer](https://scholar.google.com/citations?user=FaVnBlIAAAAJ&hl=en&oi=sra)
* [Professor Matteo Massaro](https://scholar.google.it/citations?user=4-tQq6cAAAAJ&hl=en)
