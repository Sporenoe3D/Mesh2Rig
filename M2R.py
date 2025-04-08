bl_info = {
    "name": "Mesh2Rig",
    "author": "Sporenoe3D & Gemini Pro", # Updated AI mention
    "version": (1, 8, 9), # v1.8.0: Refactored bpy.ops, PropertyGroup, state handling
    "location": "View3D > Sidebar (N-Panel) > Rig Tool", # Category name updated for consistency
    "description": "Creates rig from mesh density, merges bones, adds constraints, applies pose, removes Armature Mod, adds Surface Deform, bakes action, finalizes deform. Note: Developed with AI assistance.",
    "warning": "",
    "doc_url": "",
    "category": "Rigging",
}

import bpy
import bmesh
import mathutils
from mathutils import Vector
from mathutils.kdtree import KDTree
import random
import time # For simple profiling if needed

# --- Helper Functions ---

def get_first_selected_object_by_type(obj_type):
    """Returns the first selected object of the specified type."""
    # Check active object first
    active_obj = bpy.context.active_object
    if active_obj and active_obj.type == obj_type:
        # Ensure it's actually selected in the view layer
        if active_obj.name in bpy.context.view_layer.objects and bpy.context.view_layer.objects[active_obj.name].select_get():
             return active_obj

    # Check all selected objects
    selected_objs = [o for o in bpy.context.selected_objects if o.type == obj_type]
    if selected_objs:
        return selected_objs[0]
    return None

def get_selected_mesh_object():
    """Returns the first selected MESH object."""
    return get_first_selected_object_by_type('MESH')

def get_selected_armature_object():
    """Returns the first selected ARMATURE object."""
    return get_first_selected_object_by_type('ARMATURE')


# --- Core Rigging Logic ---

def select_bone_vertices(mesh_obj, target_bone_count, proximity_threshold):
    """
    Selects vertices for bone head placement using proximity (KDTree).
    (No bpy.ops needed here)
    """
    mesh = mesh_obj.data
    if not mesh.vertices:
        print("Mesh has no vertices.")
        return []

    world_matrix = mesh_obj.matrix_world
    vert_coords_world = [world_matrix @ v.co for v in mesh.vertices]
    vert_indices = list(range(len(mesh.vertices))) # Store original indices

    if not vert_coords_world:
        print("Could not get vertex coordinates.")
        return []

    potential_indices = []
    processed_indices = set() # Indices of vertices that are already processed or too close

    # --- KDTree Logic (Proximity Check) ---
    print("Using KDTree proximity check for initial bone placement.")
    kdt = KDTree(len(vert_coords_world))
    for i, co in enumerate(vert_coords_world): kdt.insert(co, i)
    kdt.balance()
    effective_threshold = max(proximity_threshold, 0.0001)

    # Iterate through vertices
    indices_to_check = vert_indices
    for i in indices_to_check:
        if i in processed_indices: continue
        potential_indices.append(i)
        processed_indices.add(i)
        # Find neighbors within the threshold and add them to processed set
        nearby_results = kdt.find_range(vert_coords_world[i], effective_threshold)
        nearby_indices = {idx for co, idx, dist_sq in nearby_results if idx != i} # Exclude self
        processed_indices.update(nearby_indices)

    print(f"Found {len(potential_indices)} potential bone locations after proximity filtering.")

    # --- Selecting the Target Number of Bones ---
    if not potential_indices:
        print("No vertices left after proximity filtering.")
        return []

    if len(potential_indices) > target_bone_count:
        print(f"Sampling {target_bone_count} bones randomly from {len(potential_indices)} potential locations.")
        selected_indices = random.sample(potential_indices, target_bone_count)
    else:
        print(f"Using all {len(potential_indices)} potential locations (<= target {target_bone_count}).")
        selected_indices = potential_indices

    final_vertex_indices = selected_indices
    print(f"Final selected vertex indices for bone heads: {len(final_vertex_indices)}")
    return final_vertex_indices


def create_rig_from_vertices(context, mesh_obj, bone_head_indices, fixed_bone_length=0.005):
    """
    Creates an armature with bones at the specified vertex indices using bpy.data.
    Minimizes bpy.ops usage.
    """
    if not bone_head_indices:
        print("No vertex indices provided for bone creation.")
        return None, {} # Return None and an empty dictionary

    mesh = mesh_obj.data
    world_matrix = mesh_obj.matrix_world

    if not mesh or not mesh.vertices:
        print(f"Error: Mesh '{mesh_obj.name}' seems invalid or has no vertices.")
        return None, {}

    # --- Data Creation ---
    armature_data = bpy.data.armatures.new(mesh_obj.name + "_Rig_Data")
    armature_obj = bpy.data.objects.new(mesh_obj.name + "_Rig", armature_data)
    context.collection.objects.link(armature_obj) # Link to current collection

    created_bone_map = {} # Dictionary: Bone Name -> Original Vertex Index
    missing_verts = 0
    max_vert_index = len(mesh.vertices) - 1

    # --- Bone Creation in Edit Mode (Requires switching context briefly) ---
    original_active = context.view_layer.objects.active
    original_mode = original_active.mode if original_active else 'OBJECT'

    # Ensure armature is active and we are in EDIT mode to add bones
    context.view_layer.objects.active = armature_obj
    # bpy.ops.object.mode_set needs the object to be active
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)

    try:
        # Ensure normals are calculated
        try:
            mesh.calc_normals_split()
            print("Mesh normals calculated.")
        except Exception as ne:
             print(f"Error calculating mesh normals: {ne}. Proceeding with potential issues.")

        edit_bones = armature_data.edit_bones
        existing_bone_names = set() # Track names within this creation batch

        for i, vert_index in enumerate(bone_head_indices):
            if vert_index > max_vert_index or vert_index < 0:
                print(f"Warning: Vertex index {vert_index} out of bounds (max is {max_vert_index}). Skipping.")
                missing_verts += 1
                continue

            vert = mesh.vertices[vert_index]
            head_pos = world_matrix @ vert.co

            # Calculate normal safely
            normal_world = Vector((0,0,1)) # Default fallback
            try:
                 normal_local = vert.normal
                 normal_world = (world_matrix.to_3x3().inverted_safe().transposed() @ normal_local).normalized()
                 if normal_world.length < 0.1: normal_world = Vector((0,0,1))
            except Exception as e:
                print(f"Warning: Could not get/transform normal for vertex {vert_index}: {e}. Using fallback.")

            tail_pos = head_pos + normal_world * fixed_bone_length
            bone_name_base = f"Vtx_{vert_index}"
            bone_name = bone_name_base
            count = 1
            while bone_name in edit_bones or bone_name in existing_bone_names: # Check both existing and newly added
                 bone_name = f"{bone_name_base}_{count}"
                 count += 1
                 if count > len(bone_head_indices) + 10:
                      print(f"Error: Could not find unique name for bone based on Vtx_{vert_index}. Skipping.")
                      bone_name = None; break
            if bone_name is None:
                 missing_verts += 1; continue

            try:
                edit_bone = edit_bones.new(bone_name)
                edit_bone.head = head_pos
                edit_bone.tail = tail_pos
                created_bone_map[bone_name] = vert_index
                existing_bone_names.add(bone_name) # Add to our set for this run
            except Exception as e:
                print(f"Error creating bone '{bone_name}' for vertex {vert_index}: {e}")
                missing_verts += 1

        if missing_verts > 0: print(f"Skipped {missing_verts} bones (invalid vertex indices or errors).")

        if not created_bone_map:
            print("No bones were created.")
            # Need to exit edit mode before removing
            bpy.ops.object.mode_set(mode='OBJECT')
            # Clean up the created empty armature object using bpy.data
            armature_name = armature_obj.name
            armature_data_name = armature_data.name
            bpy.data.objects.remove(armature_obj, do_unlink=True)
            # Check if armature data is unused before removing
            if armature_data_name in bpy.data.armatures and bpy.data.armatures[armature_data_name].users == 0:
                 bpy.data.armatures.remove(bpy.data.armatures[armature_data_name], do_unlink=True)
            # Restore original active object
            if original_active and original_active.name in context.view_layer.objects:
                 context.view_layer.objects.active = original_active
                 bpy.ops.object.mode_set(mode=original_mode) # Restore original mode
            return None, {}

        # Exit edit mode after creating all bones
        bpy.ops.object.mode_set(mode='OBJECT')
        print(f"Created {len(created_bone_map)} initial bones in armature '{armature_obj.name}'.")

        # Restore original active object and mode
        if original_active and original_active.name in context.view_layer.objects:
             context.view_layer.objects.active = original_active
             # Only set mode if it's different, avoids unnecessary op call
             if original_active.mode != original_mode:
                  bpy.ops.object.mode_set(mode=original_mode)
        elif context.object: # If original active is gone, just ensure we are in object mode
             if context.object.mode != 'OBJECT':
                  bpy.ops.object.mode_set(mode='OBJECT')


        return armature_obj, created_bone_map

    except Exception as e:
        print(f"Critical error during armature creation: {e}")
        # Ensure we are out of edit mode before potential cleanup
        if context.object and context.object.mode == 'EDIT':
            bpy.ops.object.mode_set(mode='OBJECT')

        # Clean up if armature object was partially created using bpy.data
        if armature_obj and armature_obj.name in bpy.data.objects:
            armature_name = armature_obj.name
            armature_data_name = armature_data.name
            bpy.data.objects.remove(armature_obj, do_unlink=True)
            # Check if armature data is unused before removing
            if armature_data_name in bpy.data.armatures and bpy.data.armatures[armature_data_name].users == 0:
                 bpy.data.armatures.remove(bpy.data.armatures[armature_data_name], do_unlink=True)

        # Restore original active object and mode if possible
        if original_active and original_active.name in context.view_layer.objects:
             context.view_layer.objects.active = original_active
             if original_active.mode != original_mode:
                  bpy.ops.object.mode_set(mode=original_mode)
        elif context.object and context.object.mode != 'OBJECT':
             bpy.ops.object.mode_set(mode='OBJECT')

        return None, {}


def parent_mesh_to_rig(context, mesh_obj, armature_obj):
    """
    Parents the mesh to the armature with automatic weights.
    Requires bpy.ops.object.parent_set for automatic weights.
    Minimizes unnecessary state changes.
    """
    if not mesh_obj or mesh_obj.name not in context.view_layer.objects:
        print("Mesh object not found for parenting.")
        return False
    if not armature_obj or armature_obj.name not in context.view_layer.objects:
         print("Armature object not found for parenting.")
         return False
    if not armature_obj.data.bones:
        print(f"Armature '{armature_obj.name}' has no bones. Skipping parenting.")
        return False
    if not mesh_obj.data.vertices:
        print(f"Mesh '{mesh_obj.name}' has no vertices. Skipping parenting.")
        return False

    # Store original active object (we MUST change it for parent_set)
    original_active = context.view_layer.objects.active
    original_mesh_mode = mesh_obj.mode
    original_armature_mode = armature_obj.mode

    # Ensure both are in Object mode for parenting operation
    if mesh_obj.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    if armature_obj.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT', toggle=False)


    # --- Prepare selection and active object for parent_set ---
    # Deselect all first using direct access
    for obj in context.view_layer.objects: obj.select_set(False)
    # Select mesh and armature
    mesh_obj.select_set(True)
    armature_obj.select_set(True)
    # Set armature as the active object (required by parent_set)
    context.view_layer.objects.active = armature_obj

    print(f"Parenting '{mesh_obj.name}' to '{armature_obj.name}' with automatic weights...")
    parenting_successful = False
    try:
        # --- CRITICAL bpy.ops call ---
        bpy.ops.object.parent_set(type='ARMATURE_AUTO')
        print("Parenting successful.")
        parenting_successful = True
    except RuntimeError as e:
        print(f"Error during automatic weighting: {e}")
        # Clean up potential partial parenting/modifier
        if mesh_obj.parent == armature_obj: mesh_obj.parent = None
        mod = mesh_obj.modifiers.get("Armature") # Default name
        if mod and mod.object == armature_obj: mesh_obj.modifiers.remove(mod)
        parenting_successful = False
    except Exception as e:
        print(f"Unexpected error during parenting: {e}")
        parenting_successful = False
    finally:
        # --- Restore minimal state ---
        # Deselect all again to leave a clean state
        # for obj in context.view_layer.objects: obj.select_set(False)
        # Restore original active object IF IT STILL EXISTS
        if original_active and original_active.name in context.view_layer.objects:
            context.view_layer.objects.active = original_active
            # Try restoring modes if objects still exist
            if mesh_obj.name in context.view_layer.objects and mesh_obj.mode != original_mesh_mode:
                 try: bpy.ops.object.mode_set(mode=original_mesh_mode)
                 except: pass # Ignore if mode set fails (e.g., object deleted)
            if armature_obj.name in context.view_layer.objects and armature_obj.mode != original_armature_mode:
                 try: bpy.ops.object.mode_set(mode=original_armature_mode)
                 except: pass
        # If original active is gone, maybe make the armature active? Or leave as is.
        # Let's leave it as is, the parent_set likely left the armature active anyway.

    return parenting_successful


# --- Property Group ---
class MeshRigProperties(bpy.types.PropertyGroup):
    bone_count: bpy.props.IntProperty(
        name="Bone Count",
        description="Target number of bones to generate",
        default=100, min=1
    )
    proximity_threshold: bpy.props.FloatProperty(
        name="Proximity Threshold",
        description="Minimum distance between generated bone heads",
        default=0.03, min=0.0, subtype='DISTANCE', unit='LENGTH', precision=4
    )
    fixed_bone_length: bpy.props.FloatProperty(
        name="Bone Length (Creation)",
        description="Fixed length for bones during initial creation",
        default=0.005, min=0.0001, max=1.0, subtype='DISTANCE', unit='LENGTH', precision=4
    )
    merge_threshold_factor: bpy.props.FloatProperty(
        name="Merge Radius Factor",
        description="Multiplier for Proximity Threshold to determine merge radius (Factor * Threshold)",
        default=2.5, min=1.01, precision=2
    )
    merge_min_count: bpy.props.IntProperty(
        name="Min Bones to Merge",
        description="Minimum number of selected bones within merge radius to be merged",
        default=3, min=2
    )
    parenting_mesh_target: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Mesh",
        description="The mesh object to be parented",
        poll=lambda self, obj: obj.type == 'MESH'
    )
    parenting_armature_target: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Armature",
        description="The armature object to parent the mesh to",
        poll=lambda self, obj: obj.type == 'ARMATURE'
    )
    constraint_target: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Target Mesh (VG)",
        description="Mesh containing vertex groups matching bone names for constraints",
        poll=lambda self, obj: obj.type == 'MESH'
    )
    surfdeform_source_mesh: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Source",
        description="Mesh object that will receive the SurfaceDeform modifier",
        poll=lambda self, obj: obj.type == 'MESH'
    )
    surfdeform_target_mesh: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Target (Bind)",
        description="Mesh object the SurfaceDeform modifier will bind to",
        poll=lambda self, obj: obj.type == 'MESH'
    )
    bake_armature_target: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Armature",
        description="The armature whose pose action will be baked",
        poll=lambda self, obj: obj.type == 'ARMATURE'
    )
    bake_start_frame: bpy.props.IntProperty(
        name="Start Frame",
        description="Frame to start baking from",
        default=1 # Sensible default, user can sync with scene
    )
    bake_end_frame: bpy.props.IntProperty(
        name="End Frame",
        description="Frame to end baking at",
        default=250 # Sensible default
    )
    final_source_mesh: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Source Mesh",
        description="Mesh to remove SD from and add final Armature mod to",
        poll=lambda self, obj: obj.type == 'MESH'
    )
    final_target_armature: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Target Armature",
        description="Armature for the final Armature modifier",
        poll=lambda self, obj: obj.type == 'ARMATURE'
    )

# --- Rig Creation Operator ---
class RIG_OT_CreateMeshRig(bpy.types.Operator):
    """Creates an armature for the selected mesh based on vertex proximity"""
    bl_idname = "object.create_mesh_rig"
    bl_label = "Create Rig"
    bl_options = {'REGISTER', 'UNDO'}

    # Note: Bone length is now read directly from the PropertyGroup in execute

    @classmethod
    def poll(cls, context):
        return get_selected_mesh_object() is not None

    def execute(self, context):
        start_time = time.time()
        props = context.scene.mesh_rig_tool_props
        mesh_obj = get_selected_mesh_object() # Get currently selected mesh

        if not mesh_obj:
            self.report({'ERROR'}, "No MESH object selected.")
            return {'CANCELLED'}

        target_bone_count = props.bone_count
        proximity_threshold = props.proximity_threshold
        fixed_bone_length = props.fixed_bone_length # Use the property from the group

        # --- Checks ---
        if target_bone_count <= 0: self.report({'ERROR'}, "Bone Count must be > 0."); return {'CANCELLED'}
        if proximity_threshold < 0.0: self.report({'ERROR'}, "Proximity Threshold must be >= 0."); return {'CANCELLED'}
        if fixed_bone_length <= 0: self.report({'ERROR'}, "Bone Length must be > 0."); return {'CANCELLED'}

        self.report({'INFO'}, f"Starting rig creation for '{mesh_obj.name}'...")
        self.report({'INFO'}, f"Parameters: Bones={target_bone_count}, Threshold={proximity_threshold:.4f}, Length={fixed_bone_length:.4f}")

        # 1. Select vertices (Helper function, no ops)
        selected_indices = select_bone_vertices(mesh_obj, target_bone_count, proximity_threshold)
        if not selected_indices:
            self.report({'WARNING'}, "Could not select vertices for bones.")
            return {'CANCELLED'}

        # 2. Create armature (Helper function, minimized ops)
        # Pass context for mode switching if needed inside the helper
        armature_obj, bone_vertex_map = create_rig_from_vertices(context, mesh_obj, selected_indices, fixed_bone_length=fixed_bone_length)
        if not armature_obj:
            self.report({'ERROR'}, "Failed to create armature.")
            # No need to restore active object here, helper should handle its internal state
            return {'CANCELLED'}

        final_bone_count = len(armature_obj.data.bones) if armature_obj.data else 0
        end_time = time.time()
        report_msg = f"Rig '{armature_obj.name}' created ({final_bone_count} bones). Time: {end_time - start_time:.2f}s. Mesh NOT parented."
        self.report({'INFO'}, report_msg)

        # --- Set targets for the next steps in the PropertyGroup ---
        props.parenting_mesh_target = mesh_obj
        props.parenting_armature_target = armature_obj
        props.constraint_target = mesh_obj # Default constraint target to original mesh
        props.surfdeform_source_mesh = mesh_obj # Default SurfDeform source to original mesh
        props.surfdeform_target_mesh = mesh_obj # Target for SurfDeform often same as source initially
        props.bake_armature_target = armature_obj # Default Bake target to new armature
        props.final_source_mesh = mesh_obj # Default Finalize source to original mesh
        props.final_target_armature = armature_obj # Default Finalize target to new armature

        # Select the created armature and make it active (User Experience)
        # Deselect others first for clarity
        for obj in context.selected_objects: obj.select_set(False)
        if armature_obj.name in context.view_layer.objects:
            armature_obj.select_set(True)
            context.view_layer.objects.active = armature_obj
        else:
             self.report({'WARNING'}, f"Created armature '{armature_obj.name}' not found after creation.")

        return {'FINISHED'}

# --- Operator to merge SELECTED bones in Edit Mode ---
class RIG_OT_MergeSelectedBones(bpy.types.Operator):
    """Merges selected bones in Edit Mode if they are close enough"""
    bl_idname = "armature.merge_selected_bones_tool"
    bl_label = "Merge Selected Bones"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        # Check mode and object type without relying on selection state alone
        obj = context.object
        if not (obj and obj.type == 'ARMATURE' and obj.mode == 'EDIT'): return False
        # Check properties and selected *editable* bones
        props = context.scene.mesh_rig_tool_props
        min_count = props.merge_min_count
        return context.selected_editable_bones and len(context.selected_editable_bones) >= min_count

    def execute(self, context):
        start_time = time.time()
        props = context.scene.mesh_rig_tool_props
        armature_obj = context.object # Assumes poll() ensures this is the correct armature in Edit mode
        armature_data = armature_obj.data

        proximity_threshold = props.proximity_threshold
        merge_factor = props.merge_threshold_factor
        min_merge_count = props.merge_min_count

        if merge_factor <= 1.0: self.report({'WARNING'}, "Merge Factor must be > 1.0."); return {'CANCELLED'}
        if min_merge_count < 2: self.report({'ERROR'}, "Min Bones to Merge must be >= 2."); return {'CANCELLED'}

        merge_radius = proximity_threshold * merge_factor
        self.report({'INFO'}, f"Starting merge of selected bones. Radius: {merge_radius:.4f}, Min Count: {min_merge_count}")

        # Use context.selected_editable_bones directly
        selected_bones = context.selected_editable_bones[:] # Make a copy
        if len(selected_bones) < min_merge_count:
            self.report({'INFO'}, f"Not enough bones selected ({len(selected_bones)}/{min_merge_count}).")
            return {'CANCELLED'}

        # --- Use bpy.data within Edit Mode ---
        bone_data = {}
        bone_heads = []
        bone_names_ordered = []

        for bone in selected_bones:
            # Check if bone still exists (might have been removed in a previous merge iteration if run again)
            if bone.name not in armature_data.edit_bones: continue
            head = bone.head.copy()
            tail = bone.tail.copy()
            length = bone.length
            direction = bone.vector.normalized() if length > 1e-6 else Vector((0,0,1))
            bone_data[bone.name] = {'head': head, 'tail': tail, 'length': length, 'dir': direction, 'bone': bone}
            bone_heads.append(head)
            bone_names_ordered.append(bone.name)

        if not bone_heads: self.report({'WARNING'}, "Could not find heads of selected bones."); return {'CANCELLED'}

        kdt = KDTree(len(bone_heads))
        for i, head_pos in enumerate(bone_heads): kdt.insert(head_pos, i)
        kdt.balance()

        processed_bone_indices = set()
        bones_to_remove_names = set()
        bones_to_add_data = []
        merged_bone_names = set() # Track newly created names to avoid duplicates in same run
        merge_counter = 0

        for i, bone_name in enumerate(bone_names_ordered):
            if i in processed_bone_indices or bone_name not in bone_data: continue

            current_head = bone_heads[i]
            cluster_results = kdt.find_range(current_head, merge_radius)
            # Check against processed_bone_indices *and* ensure the bone name still exists in our current data
            actual_cluster_indices = {idx for pos, idx, dist_sq in cluster_results
                                      if idx not in processed_bone_indices and bone_names_ordered[idx] in bone_data}

            if len(actual_cluster_indices) >= min_merge_count:
                merge_counter += 1
                actual_cluster_names = [bone_names_ordered[idx] for idx in actual_cluster_indices]
                self.report({'INFO'}, f"  Cluster {merge_counter} ({len(actual_cluster_names)}): {', '.join(actual_cluster_names)}")

                avg_head, avg_dir = Vector((0,0,0)), Vector((0,0,0))
                total_length, num_in_cluster = 0.0, len(actual_cluster_indices)
                cluster_edit_bone_names = set()

                for idx in actual_cluster_indices:
                    name = bone_names_ordered[idx]; data = bone_data[name]
                    avg_head += data['head']; avg_dir += data['dir']; total_length += data['length']
                    cluster_edit_bone_names.add(name)

                avg_head /= num_in_cluster
                avg_length = total_length / num_in_cluster
                if avg_dir.length > 1e-6: avg_dir.normalize()
                else: avg_dir = Vector((0,0,1))
                final_bone_length = max(avg_length, 0.001)
                avg_tail = avg_head + avg_dir * final_bone_length

                new_bone_name_base = f"Merged_{merge_counter:03d}"; new_bone_name = new_bone_name_base; k = 1
                # Check against all current edit bones AND names we plan to add in this run
                existing_and_pending_names = {b.name for b in armature_data.edit_bones} | merged_bone_names
                while new_bone_name in existing_and_pending_names:
                    new_bone_name = f"{new_bone_name_base}_{k}"; k += 1

                bones_to_add_data.append({'name': new_bone_name, 'head': avg_head, 'tail': avg_tail})
                merged_bone_names.add(new_bone_name) # Track for duplicate check within this run
                bones_to_remove_names.update(cluster_edit_bone_names)
                processed_bone_indices.update(actual_cluster_indices)
                # Remove processed bones from bone_data to prevent re-processing
                for name_to_remove in cluster_edit_bone_names:
                     if name_to_remove in bone_data: del bone_data[name_to_remove]
                self.report({'INFO'}, f"    -> Merging into '{new_bone_name}' at {avg_head.to_tuple(3)}")
            else:
                # Mark as processed even if not merged
                processed_bone_indices.add(i)

        # --- Perform removals and additions using edit_bones ---
        removed_count = 0
        if bones_to_remove_names:
            self.report({'INFO'}, f"Removing {len(bones_to_remove_names)} bones...")
            edit_bones = armature_data.edit_bones
            for name in list(bones_to_remove_names): # Iterate over copy
                 bone_to_remove = edit_bones.get(name)
                 if bone_to_remove:
                     try: edit_bones.remove(bone_to_remove); removed_count += 1
                     except Exception as e: self.report({'WARNING'}, f"  Error removing bone '{name}': {e}")
            self.report({'INFO'}, f"Removed {removed_count} bones.")

        added_count = 0; newly_added_bone_names = []
        if bones_to_add_data:
            self.report({'INFO'}, f"Adding {len(bones_to_add_data)} merged bones...")
            edit_bones = armature_data.edit_bones
            for data in bones_to_add_data:
                 # Final check for name uniqueness before adding
                 final_name = data['name']; k=1
                 while final_name in edit_bones: final_name = f"{data['name']}_dup{k}"; k+=1
                 if final_name != data['name']: self.report({'INFO'}, f"  Name changed to '{final_name}'.")
                 try:
                     new_bone = edit_bones.new(final_name)
                     new_bone.head, new_bone.tail = data['head'], data['tail']
                     newly_added_bone_names.append(final_name); added_count += 1
                 except Exception as e: self.report({'ERROR'}, f"  Error adding bone '{final_name}': {e}")
            self.report({'INFO'}, f"Added {added_count} merged bones.")

        # --- Update selection state in Edit Mode ---
        # Deselect all original bones first
        for bone in edit_bones: bone.select = False
        # Select the newly added bones
        for name in newly_added_bone_names:
            new_bone = edit_bones.get(name)
            if new_bone: new_bone.select, new_bone.select_head, new_bone.select_tail = True, True, True

        # No need to manage active object or global selection, we stay in Edit Mode
        end_time = time.time()
        self.report({'INFO'}, f"Merge complete. Removed: {removed_count}, Added: {added_count}. Time: {end_time - start_time:.2f}s.")
        return {'FINISHED'}


# --- Mesh Parenting Operator ---
class RIG_OT_ParentMeshToRigTool(bpy.types.Operator):
    """Parents the target mesh to the target armature with automatic weights"""
    bl_idname = "object.parent_mesh_to_rig_tool"
    bl_label = "Parent Mesh to Armature"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        mesh_target = props.parenting_mesh_target
        armature_target = props.parenting_armature_target
        # Check if objects exist in the current view layer context
        return (mesh_target and armature_target and
                mesh_target.name in context.view_layer.objects and
                armature_target.name in context.view_layer.objects)
                # Type checks are implicitly handled by PointerProperty poll

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        mesh_obj = props.parenting_mesh_target
        armature_obj = props.parenting_armature_target

        # Re-verify existence in case they were deleted after poll
        if not mesh_obj or mesh_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Mesh not found."); return {'CANCELLED'}
        if not armature_obj or armature_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Armature not found."); return {'CANCELLED'}

        self.report({'INFO'}, f"Parenting '{mesh_obj.name}' to '{armature_obj.name}'...")
        # Use the helper function which handles context switching for the operator
        parenting_successful = parent_mesh_to_rig(context, mesh_obj, armature_obj)

        if parenting_successful:
            self.report({'INFO'}, "Mesh parenting to armature complete.")
            # Update SurfDeform target if it was pointing to the source mesh
            # (assuming source was the mesh being parented)
            if props.surfdeform_target_mesh == mesh_obj:
                 # This logic might need adjustment depending on the exact workflow goal
                 # Should the target *always* become the parented mesh? Or only if it was the source?
                 # Let's assume it should update if it was the *same* as the source being parented.
                 if props.surfdeform_source_mesh == mesh_obj:
                      props.surfdeform_target_mesh = mesh_obj # Update target pointer
                      print(f"Updated SurfDeform Target to '{mesh_obj.name}' after parenting.")

        else:
            self.report({'ERROR'}, "Error during mesh parenting.")
            return {'CANCELLED'}

        return {'FINISHED'}


# --- Operator: Constraints -> Apply Pose -> Remove Mod ---
class RIG_OT_AddBoneConstraints(bpy.types.Operator):
    """Adds constraints, applies current pose as Rest Pose, removes Armature modifier from the initially parented mesh"""
    bl_idname = "object.add_bone_constraints"
    bl_label = "Constr -> Apply Pose -> Del Mod"
    bl_options = {'REGISTER', 'UNDO'}

    # Using a fixed constraint type for simplicity now
    # constraint_type: bpy.props.StringProperty(default='COPY_LOCATION')

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        # Use the specific armature intended for baking/finalizing, which should be the one created/merged
        armature_obj = props.bake_armature_target # Or final_target_armature? Let's use bake target for now.
        target_mesh_vg = props.constraint_target
        return (armature_obj and armature_obj.name in context.view_layer.objects and armature_obj.pose and
                target_mesh_vg and target_mesh_vg.name in context.view_layer.objects)

    def execute(self, context):
        start_time = time.time()
        props = context.scene.mesh_rig_tool_props
        # Get objects from properties
        armature_obj = props.bake_armature_target # The rig being driven
        target_mesh_vg = props.constraint_target # Mesh with VGs matching bone names
        mesh_parented = props.parenting_mesh_target # Mesh that *was* parented (to remove mod from)

        # Validation
        if not armature_obj or armature_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Armature (bake target) not set/found."); return {'CANCELLED'}
        if not target_mesh_vg or target_mesh_vg.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Mesh (VG) not set/found."); return {'CANCELLED'}
        # mesh_parented can be None if Step 2 was skipped, handle gracefully

        if not armature_obj.pose: self.report({'ERROR'}, f"Armature '{armature_obj.name}' has no Pose."); return {'CANCELLED'}
        if not target_mesh_vg.vertex_groups: self.report({'WARNING'}, f"Mesh '{target_mesh_vg.name}' (VG Target) has no vertex groups.")
        if not armature_obj.data.bones: self.report({'WARNING'}, f"Armature '{armature_obj.name}' has no bones."); return {'CANCELLED'}

        self.report({'INFO'}, f"Step 3: Constraints -> Pose -> Remove Mod...")
        self.report({'INFO'}, f"  Armature: '{armature_obj.name}', VG Target: '{target_mesh_vg.name}', Mod Mesh: '{mesh_parented.name if mesh_parented else 'None'}'")

        constraint_type = 'COPY_LOCATION' # Hardcoded for now
        constraint_count, missing_vg_count, apply_pose_success, removed_modifier_flag = 0, 0, False, False

        original_active = context.view_layer.objects.active
        original_armature_mode = armature_obj.mode

        try:
            # --- Set Armature to Pose Mode ---
            # We need to make it active to switch mode
            context.view_layer.objects.active = armature_obj
            if original_armature_mode != 'POSE':
                bpy.ops.object.mode_set(mode='POSE')

            # --- 1. Add/Update Constraints (using bpy.data) ---
            self.report({'INFO'}, "Adding/updating constraints...")
            pose_bones = armature_obj.pose.bones
            for pbone in pose_bones:
                bone_name = pbone.name
                target_vg_name = bone_name # Assume VG name matches bone name
                vertex_group = target_mesh_vg.vertex_groups.get(target_vg_name)

                if vertex_group:
                    # Remove existing constraints added by this tool
                    for c in list(pbone.constraints): # Iterate over copy
                         if c.type == constraint_type and c.name.startswith("RigTool_"):
                             try: pbone.constraints.remove(c)
                             except Exception as rem_e: print(f"Warn: Could not remove existing constraint on '{bone_name}': {rem_e}")

                    # Add new constraint
                    try:
                        constraint = pbone.constraints.new(type=constraint_type)
                        constraint.name = f"RigTool_{constraint_type}"
                        constraint.target = target_mesh_vg
                        constraint.subtarget = vertex_group.name # Use VG name as subtarget
                        # Configure constraint specifics (adjust if using different types)
                        if constraint_type == 'COPY_LOCATION':
                            constraint.use_x, constraint.use_y, constraint.use_z = True, True, True
                        # Add other constraint type settings here if needed
                        constraint_count += 1
                    except Exception as e: print(f"Error adding constraint to bone '{bone_name}': {e}")
                else:
                    missing_vg_count += 1
            self.report({'INFO'}, f"Added/updated {constraint_count} constraints (skipped {missing_vg_count} missing VGs).")

            # --- 2. Apply Pose as Rest Pose (Requires bpy.ops) ---
            self.report({'INFO'}, f"Applying pose as Rest Pose for '{armature_obj.name}'...")
            try:
                # Select all pose bones for the operator
                bpy.ops.pose.select_all(action='SELECT')
                # --- CRITICAL bpy.ops call ---
                bpy.ops.pose.armature_apply(selected=False) # Apply to all selected bones
                self.report({'INFO'}, "Apply Rest Pose successful."); apply_pose_success = True
            except RuntimeError as e: self.report({'ERROR'}, f"Error applying Rest Pose: {e}"); raise # Re-raise to stop
            except Exception as e: self.report({'ERROR'}, f"Unexpected error during Rest Pose: {e}"); raise # Re-raise

            # --- 3. Remove Armature Modifier (using bpy.data, no mode switch needed) ---
            if mesh_parented and mesh_parented.name in context.view_layer.objects:
                self.report({'INFO'}, f"Attempting to remove Armature mod from '{mesh_parented.name}' (linked to '{armature_obj.name}')...")
                modifier_to_remove = None
                for mod in mesh_parented.modifiers:
                    if mod.type == 'ARMATURE' and mod.object == armature_obj:
                        modifier_to_remove = mod
                        break
                if modifier_to_remove:
                    try:
                        mod_name = modifier_to_remove.name
                        mesh_parented.modifiers.remove(modifier_to_remove)
                        self.report({'INFO'}, f"Removed Armature mod ('{mod_name}') from '{mesh_parented.name}'.")
                        removed_modifier_flag = True
                    except Exception as mod_rem_e:
                        self.report({'WARNING'}, f"Could not remove Armature mod ('{modifier_to_remove.name}') from '{mesh_parented.name}': {mod_rem_e}")
                else:
                    self.report({'INFO'}, f"No Armature mod targeting '{armature_obj.name}' found on '{mesh_parented.name}'.")
            elif mesh_parented:
                 self.report({'WARNING'}, f"Mesh '{mesh_parented.name}' for modifier removal not found in view layer.")
            else:
                self.report({'INFO'}, "Mesh for parenting/modifier removal not specified (Step 2 likely skipped).")

        except Exception as e:
            self.report({'ERROR'}, f"Error during Step 3: {e}")
            # Fall through to finally for state restoration
        finally:
            # --- Restore minimal state ---
            # Restore original mode of the armature if it still exists and was changed
            if armature_obj.name in context.view_layer.objects and armature_obj.mode != original_armature_mode:
                try: bpy.ops.object.mode_set(mode=original_armature_mode)
                except: pass # Ignore if fails

            # Restore original active object if it exists
            if original_active and original_active.name in context.view_layer.objects:
                context.view_layer.objects.active = original_active
            # If original active is gone, the armature is likely active now, which is fine.

        end_time = time.time()
        report_msg = f"Step 3 Done. C:{constraint_count}/{missing_vg_count}(add/skip). P:{'Ok' if apply_pose_success else 'Err'}. M:{'Ok' if removed_modifier_flag else 'No/NA'}. T:{end_time-start_time:.1f}s."
        if not apply_pose_success: self.report({'ERROR'}, report_msg)
        elif mesh_parented and not removed_modifier_flag: self.report({'WARNING'}, report_msg)
        else: self.report({'INFO'}, report_msg)

        return {'FINISHED'} if apply_pose_success else {'CANCELLED'}


# --- Surface Deform Add Operator ---
class RIG_OT_AddSurfaceDeform(bpy.types.Operator):
    """Adds a SurfaceDeform modifier and performs Bind"""
    bl_idname = "object.add_surface_deform_tool"
    bl_label = "Add SurfaceDeform & Bind"
    bl_options = {'REGISTER', 'UNDO'}

    modifier_name: bpy.props.StringProperty(default="RigTool_SurfaceDeform") # Renamed for clarity

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        source_mesh = props.surfdeform_source_mesh
        target_mesh = props.surfdeform_target_mesh
        return (source_mesh and source_mesh.name in context.view_layer.objects and
                target_mesh and target_mesh.name in context.view_layer.objects)

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        source_mesh = props.surfdeform_source_mesh
        target_mesh = props.surfdeform_target_mesh

        if not source_mesh or source_mesh.name not in context.view_layer.objects: self.report({'ERROR'}, "Source mesh not found."); return {'CANCELLED'}
        if not target_mesh or target_mesh.name not in context.view_layer.objects: self.report({'ERROR'}, "Target mesh not found."); return {'CANCELLED'}
        if source_mesh == target_mesh: self.report({'WARNING'}, "Source and Target meshes are the same.")

        self.report({'INFO'}, f"Adding/Updating SurfaceDeform on '{source_mesh.name}', target '{target_mesh.name}'...")

        original_active = context.view_layer.objects.active
        original_source_mode = source_mesh.mode
        mod, bind_success = None, False

        try:
            # Modifier operations generally don't require Object mode, do them directly
            mod = source_mesh.modifiers.get(self.modifier_name)
            if mod:
                self.report({'INFO'}, f"Modifier '{self.modifier_name}' exists. Updating target.")
                if mod.type != 'SURFACE_DEFORM':
                    self.report({'ERROR'}, f"Existing modifier '{self.modifier_name}' is not type SurfaceDeform."); return {'CANCELLED'}
                # Check if target changed AND it was bound - needs unbind/rebind
                if mod.is_bound and mod.target != target_mesh:
                    self.report({'INFO'}, "Target changed and modifier was bound. Attempting unbind/rebind.")
                    # --- Unbind requires bpy.ops and context ---
                    context.view_layer.objects.active = source_mesh
                    # Ensure object mode for the operator? Docs are unclear, but safer.
                    if source_mesh.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
                    try:
                        # --- CRITICAL bpy.ops call (toggle) ---
                        bpy.ops.object.surfacedeform_bind(modifier=mod.name)
                        mod.target = target_mesh # Set new target *after* unbind
                    except RuntimeError as unbind_e:
                        print(f"Info: Could not unbind cleanly: {unbind_e}. Setting target anyway.")
                        mod.target = target_mesh # Try setting target even if unbind failed
                else:
                     mod.target = target_mesh # Just update target if not bound or target is same
            else:
                mod = source_mesh.modifiers.new(name=self.modifier_name, type='SURFACE_DEFORM')
                mod.target = target_mesh
                self.report({'INFO'}, f"Modifier '{self.modifier_name}' added.")

            # --- Apply Bind (Requires bpy.ops) ---
            if not mod.is_bound:
                self.report({'INFO'}, f"Applying Bind for '{mod.name}'...")
                # --- Set context for bind operator ---
                context.view_layer.objects.active = source_mesh
                if source_mesh.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')

                # --- CRITICAL bpy.ops call ---
                bpy.ops.object.surfacedeform_bind(modifier=mod.name)

                # Check status after bind
                # We might need a context update? Sometimes modifiers status isn't immediate.
                context.view_layer.update() # Force dependency graph update
                mod = source_mesh.modifiers.get(self.modifier_name) # Re-get modifier from object data
                if mod and mod.is_bound:
                    self.report({'INFO'}, "Bind applied successfully."); bind_success = True
                else:
                    self.report({'ERROR'}, f"Failed to apply Bind for '{mod.name if mod else 'N/A'}'. Check intersection/topology.")
            else:
                self.report({'INFO'}, "Modifier already bound to the correct target."); bind_success = True

        except RuntimeError as e: self.report({'ERROR'}, f"SurfaceDeform/Bind error: {e}"); bind_success = False
        except Exception as e: self.report({'ERROR'}, f"Unexpected error: {e}"); bind_success = False
        finally:
            # --- Restore minimal state ---
            # Restore original mode of the source mesh if it exists and was changed
            if source_mesh.name in context.view_layer.objects and source_mesh.mode != original_source_mode:
                 try: bpy.ops.object.mode_set(mode=original_source_mode)
                 except: pass
            # Restore original active object if it exists
            if original_active and original_active.name in context.view_layer.objects:
                context.view_layer.objects.active = original_active
            # Otherwise, source_mesh is likely active, which is acceptable.

        return {'FINISHED'} if bind_success else {'CANCELLED'}

# --- Animation Baking Operator ---
class RIG_OT_BakeRigAction(bpy.types.Operator):
    """Bakes animation for the target armature with fixed parameters"""
    bl_idname = "object.bake_rig_action_tool"
    bl_label = "Bake Rig Action"
    bl_options = {'REGISTER', 'UNDO'}

    # Fixed parameters for the bake operation
    fixed_visual_keying = True
    fixed_clear_constraints = True
    fixed_use_current_action = True
    fixed_bake_types = {'POSE'}

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        armature_target = props.bake_armature_target
        return (armature_target and armature_target.name in context.view_layer.objects and
                armature_target.pose) # Check if armature has a pose structure

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        armature_obj = props.bake_armature_target
        start_frame = props.bake_start_frame
        end_frame = props.bake_end_frame

        if not armature_obj or armature_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target armature not found."); return {'CANCELLED'}
        if not armature_obj.pose or not armature_obj.pose.bones: self.report({'ERROR'}, f"Armature '{armature_obj.name}' has no Pose Bones."); return {'CANCELLED'}
        if end_frame < start_frame: self.report({'ERROR'}, "End frame is before start frame."); return {'CANCELLED'}

        self.report({'INFO'}, f"Baking animation for '{armature_obj.name}' from {start_frame} to {end_frame}...")
        self.report({'INFO'}, f"  Options: VisualKeying={self.fixed_visual_keying}, ClearCons={self.fixed_clear_constraints}, UseCurrentAction={self.fixed_use_current_action}, BakeTypes={self.fixed_bake_types}")

        original_active = context.view_layer.objects.active
        original_mode = armature_obj.mode
        original_frame = context.scene.frame_current

        try:
            # --- Prepare context for nla.bake ---
            # Deselect all first (bake operator often works on selected objects/bones)
            for obj in context.selected_objects: obj.select_set(False)
            # Select and activate the target armature
            armature_obj.select_set(True)
            context.view_layer.objects.active = armature_obj
            # Enter Pose mode
            if armature_obj.mode != 'POSE':
                bpy.ops.object.mode_set(mode='POSE')

            # Ensure all pose bones are selected for the bake (as only_selected=True seems intended)
            bpy.ops.pose.select_all(action='SELECT')

            # --- CRITICAL bpy.ops call ---
            bpy.ops.nla.bake(
                frame_start=start_frame,
                frame_end=end_frame,
                step=1,
                only_selected=True, # Bake only selected pose bones (all in this case)
                visual_keying=self.fixed_visual_keying,
                clear_constraints=self.fixed_clear_constraints,
                clear_parents=False,
                use_current_action=self.fixed_use_current_action,
                # overwrite_current_action=False, # Parameter deprecated/removed?
                bake_types=self.fixed_bake_types
            )
            self.report({'INFO'}, "Baking completed successfully.")

        except RuntimeError as e: self.report({'ERROR'}, f"Error during baking: {e}"); return {'CANCELLED'}
        except Exception as e: self.report({'ERROR'}, f"Unexpected error during baking: {e}"); return {'CANCELLED'}
        finally:
            # --- Restore minimal state ---
            context.scene.frame_set(original_frame)
            # Restore original mode if object still exists
            if armature_obj.name in context.view_layer.objects and armature_obj.mode != original_mode:
                 try: bpy.ops.object.mode_set(mode=original_mode)
                 except: pass
            # Restore original active object if it exists
            if original_active and original_active.name in context.view_layer.objects:
                 context.view_layer.objects.active = original_active
            # Deselect the armature? Optional, leave it selected/active for now.
            # if armature_obj.name in context.view_layer.objects:
            #      armature_obj.select_set(False)


        return {'FINISHED'}


# --- Step 6 Operator: Finalize Deformation ---
class RIG_OT_FinalizeDeform(bpy.types.Operator):
    """Removes SurfaceDeform and adds Armature modifier to the Source Mesh"""
    bl_idname = "object.finalize_deform_tool"
    bl_label = "Remove SD, Add Armature Mod"
    bl_options = {'REGISTER', 'UNDO'}

    surfdeform_modifier_name: bpy.props.StringProperty(
        name="SurfaceDeform Name to Remove",
        default="RigTool_SurfaceDeform" # Match name used in Step 4
    )
    armature_modifier_name: bpy.props.StringProperty(
        name="New Armature Modifier Name",
        default="Final_Armature_Deform"
    )

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        source_mesh = props.final_source_mesh
        target_armature = props.final_target_armature
        return (source_mesh and source_mesh.name in context.view_layer.objects and
                target_armature and target_armature.name in context.view_layer.objects)

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        source_mesh = props.final_source_mesh
        target_armature = props.final_target_armature

        if not source_mesh or source_mesh.name not in context.view_layer.objects: self.report({'ERROR'}, "Source mesh not found."); return {'CANCELLED'}
        if not target_armature or target_armature.name not in context.view_layer.objects: self.report({'ERROR'}, "Target armature not found."); return {'CANCELLED'}

        self.report({'INFO'}, f"Step 6: Finalizing for '{source_mesh.name}' with armature '{target_armature.name}'...")

        removed_sd = False
        added_armature = False
        # No need to store active/mode, operations use bpy.data

        try:
            # --- 1. Remove Surface Deform Modifier (bpy.data) ---
            sd_mod = source_mesh.modifiers.get(self.surfdeform_modifier_name)
            if sd_mod and sd_mod.type == 'SURFACE_DEFORM':
                self.report({'INFO'}, f"Removing SurfaceDeform modifier '{sd_mod.name}'...")
                try:
                    source_mesh.modifiers.remove(sd_mod)
                    removed_sd = True
                    self.report({'INFO'}, "SurfaceDeform removed.")
                except Exception as e:
                    self.report({'WARNING'}, f"Could not remove SurfaceDeform '{sd_mod.name}': {e}")
            elif sd_mod:
                 self.report({'WARNING'}, f"Found modifier '{self.surfdeform_modifier_name}', but it's not type SurfaceDeform.")
            else:
                self.report({'INFO'}, f"SurfaceDeform modifier '{self.surfdeform_modifier_name}' not found on '{source_mesh.name}'.")

            # --- 2. Remove potentially existing Armature mod pointing to the SAME target (bpy.data) ---
            existing_arm_mod = None
            for mod in source_mesh.modifiers:
                 if mod.type == 'ARMATURE' and mod.object == target_armature:
                      existing_arm_mod = mod
                      break
            if existing_arm_mod:
                 self.report({'INFO'}, f"Found existing Armature modifier '{existing_arm_mod.name}' targeting '{target_armature.name}'. Removing before adding new...")
                 try: source_mesh.modifiers.remove(existing_arm_mod)
                 except Exception as e: self.report({'WARNING'}, f"Could not remove existing Armature mod '{existing_arm_mod.name}': {e}")

            # --- 3. Add Armature Modifier (bpy.data) ---
            self.report({'INFO'}, f"Adding Armature modifier '{self.armature_modifier_name}'...")
            try:
                arm_mod = source_mesh.modifiers.new(name=self.armature_modifier_name, type='ARMATURE')
                arm_mod.object = target_armature
                # Check if VGs exist before enabling, prevents console error spam if they don't
                if source_mesh.vertex_groups:
                     arm_mod.use_vertex_groups = True
                     self.report({'INFO'}, f"Armature modifier '{arm_mod.name}' added, target '{target_armature.name}', using vertex groups.")
                else:
                     arm_mod.use_vertex_groups = False
                     self.report({'WARNING'}, f"Armature modifier '{arm_mod.name}' added, target '{target_armature.name}', but NO vertex groups found on '{source_mesh.name}'. Manual weighting needed?")

                added_armature = True

            except Exception as e:
                self.report({'ERROR'}, f"Failed to add Armature modifier: {e}")

        except Exception as e:
            self.report({'ERROR'}, f"Unexpected error during finalization: {e}")
            # Fall through to finally block (which is now empty as no state was saved)
        finally:
            # No state restoration needed as we only used bpy.data
            pass

        # Report overall status
        final_status = f"Step 6 Complete. SD Removed: {'Yes' if removed_sd else 'No/Not Found'}. Armature Added: {'Yes' if added_armature else 'Error'}."
        if added_armature:
             self.report({'INFO'}, final_status)
             return {'FINISHED'}
        else:
             self.report({'ERROR'}, final_status)
             return {'CANCELLED'}


# --- UI Panel ---
class RIG_PT_MeshRigPanel(bpy.types.Panel):
    bl_label = "Mesh2Rig" # Shorter Label
    bl_idname = "RIG_PT_MeshRigPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Rig Tool' # Consistent Category Name

    def draw(self, context):
        layout = self.layout
        # Access properties via the PropertyGroup
        props = context.scene.mesh_rig_tool_props

        # --- Section 1: Create Rig ---
        box_create = layout.box(); col = box_create.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="1. Create Rig", icon='ARMATURE_DATA')
        settings = col.column(align=True)
        settings.prop(props, "bone_count")
        settings.prop(props, "proximity_threshold")
        settings.prop(props, "fixed_bone_length") # Now directly controls creation length
        col.separator()
        # Operator reads settings from props, no need to pass them explicitly
        col.operator(RIG_OT_CreateMeshRig.bl_idname, text="Create Rig from Selected", icon='PLUS')

        info = col.row()
        mesh_sel = get_selected_mesh_object() # Still useful to show user what's selected
        if mesh_sel: info.label(text=f"Source: {mesh_sel.name}", icon='MESH_DATA')
        else: info.label(text="Select a MESH object", icon='ERROR')
        layout.separator()

        # --- Section 1.5: Merge Bones ---
        box_merge = layout.box(); col = box_merge.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="1.5 Merge Bones (Edit Mode)", icon='MOD_DECIM')
        settings = col.column(align=True)
        settings.prop(props, "merge_threshold_factor", text="Factor")
        settings.prop(props, "merge_min_count", text="Min Count")
        col.separator()
        col.operator(RIG_OT_MergeSelectedBones.bl_idname, text="Merge Selected", icon='AUTOMERGE_ON')
        info = col.row()
        is_arm_edit = (context.object and context.object.type == 'ARMATURE' and context.object.mode == 'EDIT')
        sel_bones_count = len(context.selected_editable_bones) if is_arm_edit else 0
        min_count = props.merge_min_count
        if not is_arm_edit: info.label(text="Requires Armature Edit Mode", icon='INFO')
        elif sel_bones_count < min_count: info.label(text=f"Select >= {min_count} bones ({sel_bones_count} sel.)", icon='INFO')
        else: info.label(text=f"Selected: {sel_bones_count}", icon='CHECKMARK')
        layout.separator()

        # --- Section 2: Parent Mesh ---
        box_parent = layout.box(); col = box_parent.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="2. Parent Mesh to Rig", icon='LINKED')
        settings = col.column(align=True)
        settings.prop(props, "parenting_mesh_target") # Text defaults to property name "Mesh"
        settings.prop(props, "parenting_armature_target") # Text defaults to "Armature"
        col.separator()
        col.operator(RIG_OT_ParentMeshToRigTool.bl_idname, text="Parent Targets", icon='LINKED')
        # Info rows show the targets set in the properties
        mesh_p = props.parenting_mesh_target
        arm_p = props.parenting_armature_target
        col.label(text=f"Mesh: {mesh_p.name if mesh_p else 'None'}", icon='MESH_DATA' if mesh_p else 'QUESTION')
        col.label(text=f"Arm: {arm_p.name if arm_p else 'None'}", icon='ARMATURE_DATA' if arm_p else 'QUESTION')
        layout.separator()

        # --- Section 3: Constraints -> Pose -> Remove Mod ---
        box_con = layout.box(); col = box_con.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="3. Constr -> Pose -> Del Mod", icon='CONSTRAINT_BONE')
        settings = col.column(align=True)
        settings.prop(props, "constraint_target") # Text defaults to "Target Mesh (VG)"
        col.separator()
        col.operator(RIG_OT_AddBoneConstraints.bl_idname, text="Execute Step 3", icon='HOOK')
        # Info displays the relevant property targets for this step
        arm_bake = props.bake_armature_target # Armature used in this step
        mesh_vg = props.constraint_target
        mesh_mod = props.parenting_mesh_target # Mesh for mod removal info
        col.label(text=f"Armature: {arm_bake.name if arm_bake else 'Set Bake Target!'}", icon='ARMATURE_DATA' if arm_bake else 'ERROR')
        col.label(text=f"VG Target: {mesh_vg.name if mesh_vg else 'None'}", icon='MESH_DATA' if mesh_vg else 'QUESTION')
        col.label(text=f"Del Mod from: {mesh_mod.name if mesh_mod else 'None/Skipped'}", icon='MODIFIER_OFF' if mesh_mod else 'INFO')
        layout.separator()

        # --- Section 4: Surface Deform ---
        box_sd = layout.box(); col = box_sd.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="4. Surface Deform", icon='MOD_MESHDEFORM')
        settings = col.column(align=True)
        settings.prop(props, "surfdeform_source_mesh") # Text defaults to "Source"
        settings.prop(props, "surfdeform_target_mesh") # Text defaults to "Target (Bind)"
        col.separator()
        col.operator(RIG_OT_AddSurfaceDeform.bl_idname, text="Add/Update & Bind", icon='MOD_MESHDEFORM')
        mesh_sd_s = props.surfdeform_source_mesh
        mesh_sd_t = props.surfdeform_target_mesh
        col.label(text=f"Source: {mesh_sd_s.name if mesh_sd_s else 'None'}", icon='MESH_DATA' if mesh_sd_s else 'QUESTION')
        col.label(text=f"Target: {mesh_sd_t.name if mesh_sd_t else 'None'}", icon='MESH_DATA' if mesh_sd_t else 'QUESTION')
        layout.separator()

        # --- Section 5: Bake Action ---
        box_bake = layout.box(); col = box_bake.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="5. Bake Action", icon='ACTION')
        settings = col.column(align=True)
        settings.prop(props, "bake_armature_target") # Text defaults to "Armature"
        row_frames = settings.row(align=True)
        row_frames.prop(props, "bake_start_frame", text="Start") # Use property name
        row_frames.prop(props, "bake_end_frame", text="End")   # Use property name
        col.separator()
        col.operator(RIG_OT_BakeRigAction.bl_idname, text="Bake Target Armature", icon='RENDER_ANIMATION')
        arm_bake = props.bake_armature_target
        col.label(text=f"Arm: {arm_bake.name if arm_bake else 'None'}", icon='ARMATURE_DATA' if arm_bake else 'QUESTION')
        layout.separator()

        # --- Section 6: Finalize Deform ---
        box_fin = layout.box(); col = box_fin.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="6. Finalize Deform", icon='MOD_ARMATURE')
        settings = col.column(align=True)
        settings.prop(props, "final_source_mesh") # Text defaults to "Source Mesh"
        settings.prop(props, "final_target_armature") # Text defaults to "Target Armature"
        col.separator()
        op = col.operator(RIG_OT_FinalizeDeform.bl_idname, text="Remove SD, Add Armature", icon='CHECKMARK')
        # Optionally display operator properties if needed (but defaults are likely fine)
        # op_row = settings.row()
        # op_row.prop(op, "surfdeform_modifier_name", text="SD Name")
        # op_row.prop(op, "armature_modifier_name", text="Arm Mod Name")

        mesh_fin_s = props.final_source_mesh
        arm_fin_t = props.final_target_armature
        col.label(text=f"Source: {mesh_fin_s.name if mesh_fin_s else 'None'}", icon='MESH_DATA' if mesh_fin_s else 'QUESTION')
        col.label(text=f"Target: {arm_fin_t.name if arm_fin_t else 'None'}", icon='ARMATURE_DATA' if arm_fin_t else 'QUESTION')


# --- Registration ---

classes = (
    MeshRigProperties, # Register PropertyGroup first
    RIG_OT_CreateMeshRig,
    RIG_OT_MergeSelectedBones,
    RIG_OT_ParentMeshToRigTool,
    RIG_OT_AddBoneConstraints,
    RIG_OT_AddSurfaceDeform,
    RIG_OT_BakeRigAction,
    RIG_OT_FinalizeDeform,
    RIG_PT_MeshRigPanel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    # Add the PointerProperty to the Scene, pointing to the PropertyGroup
    bpy.types.Scene.mesh_rig_tool_props = bpy.props.PointerProperty(type=MeshRigProperties)

    v = bl_info.get('version', (0, 0, 0))
    print(f"Mesh2Rig Addon (v{v[0]}.{v[1]}.{v[2]}) Registered") # Updated name slightly


def unregister():
     # Delete the PointerProperty from Scene first
     if hasattr(bpy.types.Scene, "mesh_rig_tool_props"):
         del bpy.types.Scene.mesh_rig_tool_props

     # Unregister classes in reverse order
     for cls in reversed(classes):
         try:
             bpy.utils.unregister_class(cls)
         except RuntimeError:
             pass # Ignore errors if already unregistered or Blender is quitting

     print("Mesh2Rig Addon Unregistered")

if __name__ == "__main__":
    # Allow re-running the script in Blender Text Editor
    try: unregister()
    except Exception: pass
    register()