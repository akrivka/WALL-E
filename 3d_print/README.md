# Wall-E 3D print files

This is 3D print project based on:

* https://www.thingiverse.com/thing:3703555/remixes (Thingiverse post)
* https://wired.chillibasket.com/3d-printed-wall-e/ (dude's own blogpost with more details).

Our version is 5/3x scaled, which brings up the model to about 70% "life-size". All the electronics will be custom based on boards and components from the class EE 110 at Caltech.

We will be using the MG996R servos instead of SG90 (~2x bigger, much stronger), so need to rework the attachments. We'll also probably using a different DC motor.

It would be nice to implement the following remixes:

1. Track/drive improvements recommended in the blogpost (@Neil / @Jack which to choose?)
2. [Motorised eyebrows](https://www.thingiverse.com/thing:5420196)
3. Servo for door hinge? (need to find if someone has done)

Small parts will be printed on Bambu X1 Carbon printers and large parts on a CraftBot Flow XL printer. All parts will be sand-blasted and spary painted before assembly.

## Folder structure 

The idea of the subfolders is:

- `0.1_source_files`: Files directly downloaded from the internet and other then unzipping and minor folder renaming/mergin, they are unmodified
- `0.2_converted`: Parts converted to SolidWorks .sldprt
- `0.3_scaled`: Parts scaled by 5/3, nothing else is done to them
- `1_sldprt`: Current SolidWorks parts with any changes/additions. A kind of source of truth.
- `2_stl`: .stl export for 3D printing
- `3_batches`: 3D printer specific files (.3mf for Bambu, ...) with groups of parts (importted from 2_stl/) for batch printing, indexed in chronologically order

Furthermore, I am also tracking all parts and components in `list_of_parts.xlsx`. 

