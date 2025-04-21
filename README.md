# Mesh2Rig
Blender add-on for converting mesh to rig

Will help transfer and bake cloth simulation or something like that to rig. This can be useful when transferring animation into a game engine or other software.

I highly recommend watching the official video tutorial: https://youtu.be/k9EHTYyUhok

Setup:

Prepare an object with mesh animation (e.g., cloth simulation).

Step 1: Automatic Rig generation

Set the maximum number of bones.
Set the minimum distance multiplier between bones.
Set the bone scale controller (adjust if model surfaces are close).

Step 1.5: Merge Bones

Allows you to merge selected bones in Edit mode if needed. 
Set Factor: Distance multiplier for searching nearby bones.
Set Min count: Lower limit for merging bones.

Steps 2: Making scales for rig

Steps 3: Automatic Setup

Use the "Add rotation constraints" checkbox for custom rigs or for low bone counts on high-poly models. Recommended to leave off for automated rigs with lots of bones. 

Step 4: Animation Baking

Set animation Start frame.
Set animation End frame.
Bake the animation.

Step 5: Finalize

Note: Don't forget to remove your initial animation (cloth simulation, deformation modifiers, etc.)

https://extensions.blender.org/add-ons/mesh2rig/
