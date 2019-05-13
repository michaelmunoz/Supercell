# Supercell
## TeleOp written by FTC Team 779 Victorian Voltage for the 2017-2018 Relic Recovery Game
The focus of this TeleOp, Supercell, is to allow drivers to easily multitask and minimize excess hand/finger movement while driving using the Logitech Gamepad controllers.

This code was designed and written by the drivers of 7797 for the most convenient and enjoyable driver experience.

## Design Overview
Supercell accomadates up to 2 drivers. The two drivers' responsibilites are split in such a way that a *complete cypher* can be constructed in **under 50 seconds**, even without a *second driver*.

Responsibilites and controls of each driver are as follows.

**Driver 1 :**
* Drive Base 
  * Mechanum wheel-base
  * L-Joystick - Omni-directional movement
  * R-Joystick - Turning
* Intake Mechanism 
  * Motors located at the front of the bot used for sucking *glyphs* into robot
  * X - Intake 
  * Y - Reverse Intake
  * Y - Quickly spits out and then switches back to sucking in *glyph*
* Ramp 
  * Used for dumping and scoring *glyphs*
  * L/R Bumpers - Toggle position of scoring ramp

**Driver 2 :**
* Relic Arm 
  * Linear slide used for *relic* scoring in end game
  * L-joustick - Extension and retraction of relic arm
* Relic Servos
  * Servos attached to end of Relic Arm used to grab and release *relic*
  * L-Bumper - Toggle relic arm position
  * R-Bumper - Toggle relic claw position
* Jewel Collector 
  * Motor used to collect a pesky jewel that slipped into the *Cryptobox*
  * L/R Trigger - Spin jewel collector

## Software Overview
Supercell makes use of polar mathematics to allow for calculations needed for our robots mechanum drive train. This allows drivers to effortlessly move in any direction using only 1 of the 2 joysticks.

Turning is also achieved by 1 of the 2 joysticks. In addition, turning is split into two turn rates thanks to *Dual-Zone* analog stick calculations.

Concurrent instructions were implemented so that the drivers can schedule commands and override ongoing instructions for a short period of time. These paralled commands were used to solve the issue of a *glyph* getting stuck in our intake. Without these concurrent instructions, drivers would have two toggle two seperate buttons to unjam the intake. With them, the driver only has to hit 1 button once.

The co-pilot's controls are also very intuitive with extension and retraction linear slide mapped to a single anolog stick. The co-pilot also control servos and other ammenities with the use of toggle buttons.

## When did Supercell see use
This final iteration of Supercell was used at the 2018 UIL FTC Robotics State Championship.

For the majority of our team's season we slowly improved and adapted Supercell to the changes in robot hardware and driver skill as the season progressed.

## The Robot
Check out our [robot reveal here](https://www.youtube.com/watch?v=vCFBw3pLAIE)!
