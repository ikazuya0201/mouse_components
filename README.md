# mouse_components
`mouse_components` contains several crates for micromouse.

## mousecore
A core crate for micromouse.
This crate contains several modules to control micromouse.
The design of this crate is very **bad**.
I don't recommend to try to understand the code of this crate.
I'm planning to create a new crate which aims to simplify the contents of this crate.

## mousesim
A simple simulator of micromouse.
This crate provides a simulator and implementations of sensors and actuators
whose interfaces are defined in `mousecore` crate.

## sensors
Implementations of sensors and actuators whose interfaces
are defined in `mousecore` crate.
These implementations depend on `embedded_hal` crate.
