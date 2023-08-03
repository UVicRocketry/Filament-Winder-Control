#Filament Winder Documentation
- [HARDWARE](#hardware)
  - [MACHINING AXIS](#machining-axis)
- [SOFTWARE(in planning)](#softwarein-planning)

NOTE: This document is in draft and not finalised.

# HARDWARE #
//TODO add high level block diagram

## MACHINING AXIS ##
xxxx Stepper Motor + xxx driver //TODO update to actual model

The machine has 4 axis, axis is the same as a lathe as shown in the following figure:
![](https://mellowpine.com/wp-content/uploads/2022/07/Lathe-axis-Guide.jpg)

Avaliable axis:
- X: for xxx part //TODO add detailed but concise desciption of how the axis is used
- Z:
- C:
- B:

Axis X & Z have 2 NO (Normally Open) limit switches for homing and hard stop, internal pull up is used, the input pin will be pulled down when triggered.


# SOFTWARE(in planning) #
- Stepper motor control: AccelStepper library

Code Structure
