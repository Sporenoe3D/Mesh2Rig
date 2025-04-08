# Mesh2Rig
Blender add-on for converting mesh to rig

Will help transfer cloth simulation or something like that to rig. This is a test extension that should help transfer the animation of the grid to the rig. I made it in 6 hours without any programming knowledge using Ai, so do not judge strictly.

Setup:

Prepare an object with mesh animation (e.g., cloth simulation) and a duplicate of it, matching its shape on the first frame but without animation.
Stage 1 - Rig Creation:

Select the non-animated object.
Generate a rig on it.
(Optional) In Edit Mode, remove excess bones or add new ones as needed.
(I use a bone length of 0.003 m)
Stage 2 - Rig Binding:

Select the created rig and the non-animated object.
Perform the second stage.
Stage 3 - Rig Finalization:

Select the rig.
Execute the third stage.
Stage 4 - Target Assignment:

Set the non-animated object as the source.
Set the animated object (e.g., with cloth simulation) as the target.
Stage 5 - Animation Transfer:

Define the first and last frames of the animation
Apply the fifth stage.
Stage 6 - Final Setup:

Select the original non-animated object and the rig.
Complete the process.
