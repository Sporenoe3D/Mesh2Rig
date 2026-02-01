# Mesh2Rig – Save Any Animation as a Rig 🦴

**Mesh2Rig** is a game-ready tool designed to bake Cell Fracture/Rigid Bodies, Cloth, Softbody, Shape Keys, and any complex mesh deformations into a standard skeletal rig with just a few clicks.

## 🔥 New in Update 2.5
*   🚀 **Automatic Batch Processing:** Process multiple objects simultaneously.
*   🔗 **Smart Merge:** Combine everything into one model + one rig (perfect for destruction scenes!).
*   ⚡ **Optimized "Simple Mode":** Faster processing for rigid objects.

> **I’d really appreciate your feedback in the reviews!** It helps me improve the tool and boosts visibility. Thanks!

### 📺 Video Tutorials
*   **Official Tutorial:** https://youtu.be/k9EHTYyUhok
*   **2.5 Update Overview:** https://youtu.be/kuwmC5m1SF8

---

## ✨ Key Features

*   **Automated Bone Generation:** Uses KDTree proximity to intelligently place bones inside the mesh volume, or a **Simple Mode** (1-bone per object) for rigid debris.
*   **Surface Deform Workflow:** Automatically sets up Surface Deform modifiers for high-quality deformation without the need for manual weight painting.
*   **One-Click Baking:** Convert complex mesh animation into standard skeletal animation (Actions) instantly.
*   **Batch Processing:** Process entire collections (e.g., Cell Fracture results) in one click. Automatically rigs, binds, and bakes hundreds of objects sequence-by-sequence.
*   **Smart Merging:** Merges multiple generated rigs and meshes into a single Master Rig and Mesh, preserving all animations.

**Perfect for:** Game development optimization, baking cloth/soft body simulations to bones, and managing complex destruction scenes.

---

## 📦 Batch Processing Setup (Cell Fracture / Debris)

*Ideal for processing hundreds of fractured pieces at once.*

1.  **Prepare Collection:** Move all your simulation objects (shards/debris) into a specific Collection in Blender.
2.  **Select Collection:** In the Mesh2Rig panel, select that collection in the "Batch Target" field.
3.  **Choose Mode:**
    *   Check **Simple Mode** if the objects are rigid (don't deform internally).
    *   Uncheck it for soft bodies (cloth/jelly).
4.  **Process:** Click **Batch Process Collection**.
5.  **Merge (Optional):** Once processed, click **Merge Generated Rigs** to combine them all into one optimized asset.

---

## ⚙️ Manual Processing / Single Object Workflow

*Prepare an object with mesh animation (e.g., cloth simulation).*

### Step 1: Automatic Rig Generation
*   **Bone Count:** Set the maximum number of bones to generate.
*   **Proximity:** Set the minimum distance multiplier between bones.
*   **Bone Length:** Set the bone scale controller (adjust if model surfaces are very close).

### Step 1.5: Merge Bones (Optional)
*Allows you to merge selected bones in Edit mode if needed.*
*   **Factor:** Distance multiplier for searching nearby bones.
*   **Min Count:** Lower limit for merging bones.

### Step 2: Bind Mesh to Rig
Parents the mesh to the generated rig with automatic weights.

### Step 3: Automatic Setup (Constraints)
Sets up the logic for the rig to follow the mesh.
*   **Note:** Use the **"Add rotation constraints"** checkbox for custom rigs or for low bone counts on high-poly models. It is recommended to leave this **OFF** for automated rigs with high bone counts to avoid rotation artifacts.

### Step 4: Animation Baking
*   Set **Start Frame** and **End Frame**.
*   Click **Bake Rig Action** to transfer the movement to the bones.

### Step 5: Finalize
Removes temporary modifiers (Surface Deform) and applies the final Armature modifier.

> **⚠️ Note:** Don't forget to remove your initial animation sources (cloth simulation, cache modifiers, shape keys) after finalizing to clean up the scene.
