# UAV_Badger
Second generation quadcopter code

After the first generation quadcopter crashed I started a clean sheet design. The new frame, callsign "Badger", reduces part count, increases strength, and uses threaded heat set inserts to bolt components together. In addition, the Arduino Mega can now be bolted to the frame and is completely protected in the event of a crash. One of the primary focii on this design is flight dynamics. My research indicates that a CG at, or slightly above the CP plane is optimal. In an effort to clean up the wiring and reduce weight, a power distribution board replaces the "octopus" wire splitter, and a protoshield eliminates the rat's nest from the Arduino Mega.

The software will be completely reworked for "Badger". Instead of using a bluetooth link to a phone, "Badger" uses bluetooth to connect to an Arduino Uno that is connected serially to a computer. This will provide improved control input as the software increases in complexity and capability. The PID developed in the previous generation will be perfected so hovering for long periods is trivial. I will also work to develop flight capabilities beyond hovering. Moving in a single direction is the obvious starting point, and sensors will be added to future iterations to allow for colission avoidance.

This repository is for all the code, practice and flight ready, that goes into this project.
