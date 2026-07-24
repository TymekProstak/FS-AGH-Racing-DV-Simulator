# Model equations

The mathematical model reference is maintained as a standalone LaTeX
document:

- [`model_equations.pdf`](model_equations.pdf)
- [`model_equations.tex`](model_equations.tex)

It contains the implemented equations for:

- planar four-wheel vehicle dynamics;
- `ONE_WHEEL` and `FOUR_WHEEL` torque routing;
- MF6.1 tyres, combined slip and force relaxation;
- suspension and transient load transfer;
- steering and drivetrain dynamics;
- camera, lidar and sensor fusion;
- dropout, false positives and execution latency;
- IMU and state-estimator noise;
- independent discrete device timing.

Build the PDF from this directory with:

```bash
tectonic model_equations.tex
```
