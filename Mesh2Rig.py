bl_info = {
    "name": "Mesh2Rig",
    "author": "Sporenoe3D",
    "version": (2, 5, 0),
    "location": "View3D > Sidebar (N-Panel) > M2R", 
    "description": "Creates rig from mesh, merges bones, allows manual rig, bakes action, finalizes deform. Includes Batch Processing, Merging & Advanced Cleanup.",
    "warning": "",
    "doc_url": "",
    "category": "Rigging",
    "blender": (4, 2, 0), 
}

import bpy
import bmesh
import mathutils
from mathutils import Vector
from mathutils.kdtree import KDTree
import random
import time 

# --- Helper Functions ---

def get_first_selected_object_by_type(obj_type):
    """Returns the first selected object of the specified type."""
    active_obj = bpy.context.active_object
    if active_obj and active_obj.type == obj_type:
        if active_obj.name in bpy.context.view_layer.objects and bpy.context.view_layer.objects[active_obj.name].select_get():
             return active_obj
    selected_objs = [o for o in bpy.context.selected_objects if o.type == obj_type]
    if selected_objs:
        if active_obj and active_obj in selected_objs and active_obj.type == obj_type:
            return active_obj
        return selected_objs[0]
    return None

def get_selected_mesh_object():
    """Returns the first selected MESH object."""
    return get_first_selected_object_by_type('MESH')

def get_selected_armature_object():
    """Returns the first selected ARMATURE object."""
    return get_first_selected_object_by_type('ARMATURE')

def deselect_all_objects(context):
    """Context-safe way to deselect all objects"""
    for obj in context.selected_objects:
        obj.select_set(False)

def perform_cleanup(obj):
    """
    Cleans animation, shape keys, rigid bodies, and simulation modifiers from an object.
    """
    if not obj: return

    # 1. Clear Animation Data (Keyframes, Actions)
    if obj.animation_data:
        obj.animation_data_clear()
        
    # 2. Clear Rigid Body settings
    if obj.rigid_body:
        try:
            # Removing directly via property is safer/faster than ops in loops
            # But bpy.ops is needed to fully unregister from world collection sometimes
            # We try to remove the RB settings from the object data
            bpy.context.view_layer.objects.active = obj
            bpy.ops.rigidbody.object_remove()
        except:
            pass

    # 3. Clear Shape Keys (Only for Meshes)
    if obj.type == 'MESH' and obj.data.shape_keys:
        obj.shape_key_clear()

    # 4. Remove Simulation Modifiers
    sim_modifiers = {'CLOTH', 'SOFT_BODY', 'FLUID', 'COLLISION', 'PARTICLE_SYSTEM', 'DYNAMIC_PAINT'}
    modifiers_to_remove = []
    for mod in obj.modifiers:
        if mod.type in sim_modifiers:
            modifiers_to_remove.append(mod)
    
    for mod in modifiers_to_remove:
        try:
            obj.modifiers.remove(mod)
        except:
            pass

# --- Core Rigging Logic ---

def select_bone_vertices(mesh_obj, target_bone_count, proximity_threshold):
    """
    Selects vertices for bone head placement using proximity (KDTree).
    """
    mesh = mesh_obj.data
    if not mesh.vertices:
        print("Mesh has no vertices.")
        return []

    world_matrix = mesh_obj.matrix_world
    vert_coords_world = [world_matrix @ v.co for v in mesh.vertices]
    vert_indices = list(range(len(mesh.vertices))) 

    if not vert_coords_world:
        print("Could not get vertex coordinates.")
        return []

    potential_indices = []
    processed_indices = set() 

    # --- KDTree Logic (Proximity Check) ---
    print("Using KDTree proximity check for initial bone placement.")
    kdt = KDTree(len(vert_coords_world))
    for i, co in enumerate(vert_coords_world): kdt.insert(co, i)
    kdt.balance()
    effective_threshold = max(proximity_threshold, 0.0001) 

    indices_to_check = vert_indices 
    for i in indices_to_check:
        if i in processed_indices: continue 

        potential_indices.append(i)
        processed_indices.add(i) 

        nearby_results = kdt.find_range(vert_coords_world[i], effective_threshold)
        nearby_indices = {idx for co, idx, dist_sq in nearby_results if idx != i} 
        processed_indices.update(nearby_indices)

    print(f"Found {len(potential_indices)} potential bone locations after proximity filtering.")

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


def create_simple_one_bone_rig(context, mesh_obj, fixed_bone_length=0.05):
    """
    Creates a simple rig with a single bone at the object's origin.
    Used when 'Simple Mode' is active.
    """
    if not mesh_obj: return None, {}
    
    armature_data = bpy.data.armatures.new(mesh_obj.name + "_Rig_Data")
    armature_obj = bpy.data.objects.new(mesh_obj.name + "_Rig", armature_data)
    context.collection.objects.link(armature_obj)
    
    created_bone_map = {} # Will map "SimpleBone" -> -1 (indicating origin/no vertex)
    
    original_active = context.view_layer.objects.active
    original_mode = 'OBJECT'
    if original_active: original_mode = original_active.mode
    
    try:
        context.view_layer.objects.active = armature_obj
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)
        
        edit_bones = armature_data.edit_bones
        bone_name = "Root"
        
        head_pos = mesh_obj.matrix_world.translation
        # Default Up direction for single bone
        tail_pos = head_pos + Vector((0, 0, fixed_bone_length))
        
        edit_bone = edit_bones.new(bone_name)
        edit_bone.head = head_pos
        edit_bone.tail = tail_pos
        
        # -1 index implies no specific vertex match, it's object-centered
        created_bone_map[bone_name] = -1 
        
        bpy.ops.object.mode_set(mode='OBJECT')
        return armature_obj, created_bone_map
        
    except Exception as e:
        print(f"Error creating simple rig: {e}")
        if armature_obj.mode == 'EDIT': bpy.ops.object.mode_set(mode='OBJECT')
        if armature_obj.name in bpy.data.objects: bpy.data.objects.remove(armature_obj)
        return None, {}
        
    finally:
        if original_active and original_active.name in context.view_layer.objects:
             context.view_layer.objects.active = original_active
             if context.object and context.object.mode != original_mode:
                  try:
                      bpy.ops.object.mode_set(mode=original_mode)
                  except Exception: pass


def create_rig_from_vertices(context, mesh_obj, bone_head_indices, fixed_bone_length=0.005):
    """
    Creates an armature with bones at the specified vertex indices using bpy.data.
    """
    if not bone_head_indices:
        print("No vertex indices provided for bone creation.")
        return None, {} 

    mesh = mesh_obj.data
    world_matrix = mesh_obj.matrix_world

    if not mesh or not mesh.vertices:
        print(f"Error: Mesh '{mesh_obj.name}' seems invalid or has no vertices.")
        return None, {}

    armature_data = bpy.data.armatures.new(mesh_obj.name + "_Rig_Data")
    armature_obj = bpy.data.objects.new(mesh_obj.name + "_Rig", armature_data)
    context.collection.objects.link(armature_obj) 

    created_bone_map = {} 
    missing_verts = 0
    max_vert_index = len(mesh.vertices) - 1

    original_active = context.view_layer.objects.active
    original_mode = 'OBJECT' 
    if original_active:
        original_mode = original_active.mode
    active_obj_for_op = None 

    try:
        context.view_layer.objects.active = armature_obj
        active_obj_for_op = armature_obj
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)

        try:
            mesh.calc_normals_split()
        except Exception as ne:
             print(f"Warning: Error ensuring mesh normals: {ne}. Proceeding with potential issues.")

        edit_bones = armature_data.edit_bones
        existing_bone_names = set() 

        for i, vert_index in enumerate(bone_head_indices):
            if vert_index > max_vert_index or vert_index < 0:
                missing_verts += 1
                continue

            vert = mesh.vertices[vert_index]
            head_pos = world_matrix @ vert.co

            normal_world = Vector((0,0,1)) 
            try:
                 vert_normal_accum = Vector()
                 valid_loop_normals = 0
                 for loop in mesh.loops:
                      if loop.vertex_index == vert_index:
                           loop_normal_local = loop.normal
                           vert_normal_accum += loop_normal_local
                           valid_loop_normals +=1

                 if valid_loop_normals > 0:
                     normal_local = (vert_normal_accum / valid_loop_normals).normalized()
                 else: 
                     normal_local = vert.normal

                 normal_world = (world_matrix.to_3x3().inverted_safe().transposed() @ normal_local).normalized()
                 if normal_world.length < 0.1: 
                     normal_world = Vector((0,0,1)) 
            except Exception as e:
                print(f"Warning: Could not get/transform normal for vertex {vert_index}: {e}. Using fallback.")

            tail_pos = head_pos + normal_world * fixed_bone_length

            bone_name_base = f"Vtx_{vert_index}"
            bone_name = bone_name_base
            count = 1
            while bone_name in edit_bones or bone_name in existing_bone_names:
                 bone_name = f"{bone_name_base}_{count}"
                 count += 1
                 if count > len(bone_head_indices) + 50: 
                      bone_name = None; break
            if bone_name is None:
                 missing_verts += 1; continue

            try:
                edit_bone = edit_bones.new(bone_name)
                edit_bone.head = head_pos
                edit_bone.tail = tail_pos
                created_bone_map[bone_name] = vert_index
                existing_bone_names.add(bone_name) 
            except Exception as e:
                print(f"Error creating bone '{bone_name}' for vertex {vert_index}: {e}")
                missing_verts += 1

        if missing_verts > 0: print(f"Skipped {missing_verts} bones (invalid vertex indices or errors).")

        if armature_obj.mode == 'EDIT':
             bpy.ops.object.mode_set(mode='OBJECT')

        if not created_bone_map:
            print("No bones were created.")
            if armature_obj and armature_obj.name in bpy.data.objects:
                 armature_name = armature_obj.name
                 armature_data_name = armature_data.name
                 bpy.data.objects.remove(armature_obj, do_unlink=True)
                 if armature_data_name in bpy.data.armatures and bpy.data.armatures[armature_data_name].users == 0:
                      bpy.data.armatures.remove(bpy.data.armatures[armature_data_name], do_unlink=True)
            return None, {}

        print(f"Created {len(created_bone_map)} initial bones in armature '{armature_obj.name}'.")
        return armature_obj, created_bone_map

    except Exception as e:
        print(f"Critical error during armature creation: {e}")
        if active_obj_for_op and active_obj_for_op.name in context.view_layer.objects and active_obj_for_op.mode == 'EDIT':
            if context.view_layer.objects.active == active_obj_for_op:
                try: bpy.ops.object.mode_set(mode='OBJECT')
                except Exception: pass

        if armature_obj and armature_obj.name in bpy.data.objects:
            armature_data_name = armature_data.name
            bpy.data.objects.remove(armature_obj, do_unlink=True)
            if armature_data_name in bpy.data.armatures and bpy.data.armatures[armature_data_name].users == 0:
                 bpy.data.armatures.remove(bpy.data.armatures[armature_data_name], do_unlink=True)

        return None, {} 

    finally:
        if original_active and original_active.name in context.view_layer.objects:
             if context.view_layer.objects.active != original_active:
                 context.view_layer.objects.active = original_active
             if original_active.mode != original_mode:
                  try:
                      if context.object and context.object.mode != original_mode:
                           bpy.ops.object.mode_set(mode=original_mode)
                  except Exception: pass
        elif context.object and context.object.mode != 'OBJECT': 
            try: bpy.ops.object.mode_set(mode='OBJECT')
            except Exception: pass 


def parent_mesh_to_rig(context, mesh_obj, armature_obj):
    """
    Parents the mesh to the armature with automatic weights.
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

    original_active = context.view_layer.objects.active
    original_selected = context.selected_objects[:]
    original_mesh_mode = mesh_obj.mode
    original_armature_mode = armature_obj.mode

    parenting_successful = False
    try:
        if mesh_obj.mode != 'OBJECT':
             context.view_layer.objects.active = mesh_obj
             bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
        if armature_obj.mode != 'OBJECT':
             context.view_layer.objects.active = armature_obj
             bpy.ops.object.mode_set(mode='OBJECT', toggle=False)

        deselect_all_objects(context)

        mesh_obj.select_set(True)
        armature_obj.select_set(True)

        context.view_layer.objects.active = armature_obj

        print(f"Parenting '{mesh_obj.name}' to '{armature_obj.name}' with automatic weights...")
        bpy.ops.object.parent_set(type='ARMATURE_AUTO')
        print("Parenting successful.")
        parenting_successful = True

    except RuntimeError as e:
        print(f"Error during automatic weighting: {e}")
        if mesh_obj.parent == armature_obj: mesh_obj.parent = None
        mod_to_remove = None
        for mod in mesh_obj.modifiers:
             if mod.type == 'ARMATURE' and mod.object == armature_obj:
                 mod_to_remove = mod
                 break
        if mod_to_remove:
            try: mesh_obj.modifiers.remove(mod_to_remove)
            except Exception: pass 
        parenting_successful = False
    except Exception as e:
        print(f"Unexpected error during parenting: {e}")
        parenting_successful = False
    finally:
        if parenting_successful:
            pass 
        else:
             deselect_all_objects(context)
             for obj in original_selected:
                 if obj.name in context.view_layer.objects:
                     obj.select_set(True)

             if original_active and original_active.name in context.view_layer.objects:
                 context.view_layer.objects.active = original_active
             if mesh_obj.name in context.view_layer.objects and mesh_obj.mode != original_mesh_mode:
                  try:
                       if context.view_layer.objects.active != mesh_obj: context.view_layer.objects.active = mesh_obj
                       bpy.ops.object.mode_set(mode=original_mesh_mode)
                  except Exception: pass 
             if armature_obj.name in context.view_layer.objects and armature_obj.mode != original_armature_mode:
                  try:
                       if context.view_layer.objects.active != armature_obj: context.view_layer.objects.active = armature_obj
                       bpy.ops.object.mode_set(mode=original_armature_mode)
                  except Exception: pass
             if original_active and original_active.name in context.view_layer.objects and context.view_layer.objects.active != original_active:
                 context.view_layer.objects.active = original_active
             elif armature_obj.name in context.view_layer.objects: 
                  context.view_layer.objects.active = armature_obj

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
    bake_armature_target: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Working Armature",
        description="The armature to add constraints to, bake, and finalize. Set by Step 1 or manually.",
        poll=lambda self, obj: obj.type == 'ARMATURE'
    )
    add_copy_rotation: bpy.props.BoolProperty( 
        name="Add Copy Rotation",
        description="Include Copy Rotation constraint in Step 3",
        default=True
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
    bake_start_frame: bpy.props.IntProperty(
        name="Start Frame",
        description="Frame to start baking from",
        default=1 
    )
    bake_end_frame: bpy.props.IntProperty(
        name="End Frame",
        description="Frame to end baking at",
        default=250 
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
        description="Armature for the final Armature modifier (usually same as Working Armature)",
        poll=lambda self, obj: obj.type == 'ARMATURE'
    )
    # --- New Properties for Batch Processing ---
    batch_target_collection: bpy.props.PointerProperty(
        type=bpy.types.Collection,
        name="Batch Collection",
        description="Collection containing objects (fragments) to process sequentially"
    )
    batch_finalize: bpy.props.BoolProperty(
        name="Auto Finalize",
        description="Run Step 5 (Replace SurfaceDeform/Mods with Final Armature Mod) automatically in batch",
        default=True
    )
    batch_auto_clear_anim: bpy.props.BoolProperty(
        name="Auto Clear Anim",
        description="Automatically remove animation, shape keys, and simulations from mesh after processing",
        default=False
    )
    batch_simple_mode: bpy.props.BoolProperty(
        name="Simple Mode (1 Bone)",
        description="If checked, ignores bone count and creates a single bone at object center",
        default=False
    )


# --- Rig Creation Operator ---
class RIG_OT_CreateMeshRig(bpy.types.Operator):
    """Creates an armature for the selected mesh based on vertex proximity or simple mode"""
    bl_idname = "object.create_mesh_rig"
    bl_label = "1. Create Rig"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return get_selected_mesh_object() is not None

    def execute(self, context):
        start_time = time.time()
        props = context.scene.mesh_rig_tool_props
        mesh_obj = get_selected_mesh_object() 

        if not mesh_obj:
            self.report({'ERROR'}, "No MESH object selected.")
            return {'CANCELLED'}

        fixed_bone_length = props.fixed_bone_length 
        
        # --- SIMPLE MODE CHECK ---
        if props.batch_simple_mode:
            self.report({'INFO'}, f"Simple Mode: Creating 1 bone for '{mesh_obj.name}'...")
            armature_obj, bone_vertex_map = create_simple_one_bone_rig(context, mesh_obj, fixed_bone_length)
        else:
            # --- Standard Mode ---
            target_bone_count = props.bone_count
            proximity_threshold = props.proximity_threshold
            
            if target_bone_count <= 0: self.report({'ERROR'}, "Bone Count must be > 0."); return {'CANCELLED'}
            if proximity_threshold < 0.0: self.report({'ERROR'}, "Proximity Threshold must be >= 0."); return {'CANCELLED'}

            self.report({'INFO'}, f"Starting rig creation for '{mesh_obj.name}'...")
            
            selected_indices = select_bone_vertices(mesh_obj, target_bone_count, proximity_threshold)
            if not selected_indices:
                self.report({'WARNING'}, "Could not select vertices for bones.")
                return {'CANCELLED'}

            armature_obj, bone_vertex_map = create_rig_from_vertices(context, mesh_obj, selected_indices, fixed_bone_length=fixed_bone_length)

        if not armature_obj:
            self.report({'ERROR'}, "Failed to create armature.")
            return {'CANCELLED'}

        final_bone_count = len(armature_obj.data.bones) if armature_obj.data else 0
        end_time = time.time()
        report_msg = f"Rig '{armature_obj.name}' created ({final_bone_count} bones). Time: {end_time - start_time:.2f}s."
        self.report({'INFO'}, report_msg)

        props.parenting_mesh_target = mesh_obj
        props.parenting_armature_target = armature_obj
        props.constraint_target = mesh_obj 
        props.bake_armature_target = armature_obj 
        props.surfdeform_source_mesh = mesh_obj 
        props.surfdeform_target_mesh = mesh_obj 
        props.final_source_mesh = mesh_obj 
        props.final_target_armature = armature_obj 

        deselect_all_objects(context)
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
    bl_label = "1.5 Merge Selected Bones"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        obj = context.object
        if not (obj and obj.type == 'ARMATURE' and obj.mode == 'EDIT'): return False
        props = context.scene.mesh_rig_tool_props
        if not props: return False
        min_count = props.merge_min_count
        if not obj.data or not obj.data.edit_bones: return False
        return context.selected_editable_bones and len(context.selected_editable_bones) >= min_count

    def execute(self, context):
        start_time = time.time()
        props = context.scene.mesh_rig_tool_props
        armature_obj = context.object 
        armature_data = armature_obj.data

        proximity_threshold = props.proximity_threshold
        merge_factor = props.merge_threshold_factor
        min_merge_count = props.merge_min_count

        if merge_factor <= 1.0: self.report({'WARNING'}, "Merge Factor must be > 1.0."); return {'CANCELLED'}
        if min_merge_count < 2: self.report({'ERROR'}, "Min Bones to Merge must be >= 2."); return {'CANCELLED'}

        merge_radius = proximity_threshold * merge_factor
        self.report({'INFO'}, f"Starting merge of selected bones. Radius: {merge_radius:.4f}, Min Count: {min_merge_count}")

        selected_bones = context.selected_editable_bones[:] 
        if len(selected_bones) < min_merge_count:
            self.report({'INFO'}, f"Not enough bones selected ({len(selected_bones)}/{min_merge_count}).")
            return {'CANCELLED'}

        bone_data = {} 
        bone_heads = [] 
        bone_names_ordered = [] 
        edit_bones = armature_data.edit_bones 

        for bone in selected_bones:
            if bone.name not in edit_bones: continue 
            head = bone.head.copy(); tail = bone.tail.copy(); length = bone.length
            direction = bone.vector.normalized() if length > 1e-6 else Vector((0,0,1))
            bone_data[bone.name] = {'head': head, 'tail': tail, 'length': length, 'dir': direction, 'bone': bone}
            bone_heads.append(head); bone_names_ordered.append(bone.name)

        if not bone_heads: self.report({'WARNING'}, "Could not find heads of selected bones."); return {'CANCELLED'}

        kdt = KDTree(len(bone_heads))
        for i, head_pos in enumerate(bone_heads): kdt.insert(head_pos, i)
        kdt.balance()

        processed_bone_indices = set() 
        bones_to_remove_names = set() 
        bones_to_add_data = [] 
        merged_bone_names = set() 
        merge_counter = 0 

        for i, bone_name in enumerate(bone_names_ordered):
            if i in processed_bone_indices or bone_name not in bone_data: continue 

            current_head = bone_heads[i]
            cluster_results = kdt.find_range(current_head, merge_radius)
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

                final_bone_length = max(avg_length, props.fixed_bone_length, 0.001) 
                avg_tail = avg_head + avg_dir * final_bone_length

                new_bone_name_base = f"Merged_{merge_counter:03d}"; new_bone_name = new_bone_name_base; k = 1
                existing_and_pending_names = {b.name for b in edit_bones} | merged_bone_names 
                while new_bone_name in existing_and_pending_names:
                    new_bone_name = f"{new_bone_name_base}_{k}"; k += 1

                bones_to_add_data.append({'name': new_bone_name, 'head': avg_head, 'tail': avg_tail})
                merged_bone_names.add(new_bone_name) 
                bones_to_remove_names.update(cluster_edit_bone_names) 
                processed_bone_indices.update(actual_cluster_indices) 

                for name_to_remove in cluster_edit_bone_names:
                     if name_to_remove in bone_data: del bone_data[name_to_remove]

                self.report({'INFO'}, f"    -> Merging into '{new_bone_name}' at {avg_head.to_tuple(3)}")
            else:
                processed_bone_indices.add(i)

        removed_count = 0
        if bones_to_remove_names:
            self.report({'INFO'}, f"Removing {len(bones_to_remove_names)} bones...")
            for name in list(bones_to_remove_names):
                 bone_to_remove = edit_bones.get(name)
                 if bone_to_remove:
                     try:
                         edit_bones.remove(bone_to_remove)
                         removed_count += 1
                     except Exception as e:
                         self.report({'WARNING'}, f"  Error removing bone '{name}': {e}") 
            self.report({'INFO'}, f"Removed {removed_count} bones.")

        added_count = 0
        newly_added_bone_names = [] 
        if bones_to_add_data:
            self.report({'INFO'}, f"Adding {len(bones_to_add_data)} merged bones...")
            for data in bones_to_add_data:
                 final_name = data['name']; k=1
                 while final_name in edit_bones: final_name = f"{data['name']}_dup{k}"; k+=1
                 if final_name != data['name']: self.report({'INFO'}, f"  Name changed to '{final_name}'.") 
                 try:
                     new_bone = edit_bones.new(final_name)
                     new_bone.head, new_bone.tail = data['head'], data['tail']
                     newly_added_bone_names.append(final_name) 
                     added_count += 1
                 except Exception as e:
                     self.report({'ERROR'}, f"  Error adding bone '{final_name}': {e}")
            self.report({'INFO'}, f"Added {added_count} merged bones.")

        for bone in edit_bones: bone.select = False
        for name in newly_added_bone_names:
            new_bone = edit_bones.get(name)
            if new_bone:
                new_bone.select, new_bone.select_head, new_bone.select_tail = True, True, True

        end_time = time.time()
        self.report({'INFO'}, f"Merge complete. Removed: {removed_count}, Added: {added_count}. Time: {end_time - start_time:.2f}s.")

        return {'FINISHED'}


# --- Mesh Parenting Operator ---
class RIG_OT_ParentMeshToRigTool(bpy.types.Operator):
    """Parents the target mesh to the target armature with automatic weights"""
    bl_idname = "object.parent_mesh_to_rig_tool"
    bl_label = "2. Parent Mesh to Armature"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        if not props: return False 
        mesh_target = props.parenting_mesh_target
        armature_target = props.parenting_armature_target
        return (mesh_target and armature_target and
                mesh_target.name in context.view_layer.objects and
                armature_target.name in context.view_layer.objects)

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        mesh_obj = props.parenting_mesh_target
        armature_obj = props.parenting_armature_target

        if not mesh_obj or mesh_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Mesh not found."); return {'CANCELLED'}
        if not armature_obj or armature_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Armature not found."); return {'CANCELLED'}

        self.report({'INFO'}, f"Parenting '{mesh_obj.name}' to '{armature_obj.name}'...")
        parenting_successful = parent_mesh_to_rig(context, mesh_obj, armature_obj)

        if parenting_successful:
            self.report({'INFO'}, "Mesh parenting to armature complete.")
        else:
            self.report({'ERROR'}, "Error during mesh parenting.")
            return {'CANCELLED'}

        return {'FINISHED'}


# --- Operator: Constraints -> Apply Pose -> Remove Mod ---
class RIG_OT_AddBoneConstraints(bpy.types.Operator):
    """Removes initial Armature mod, Adds Loc/Rot constraints, Applies current pose as Rest Pose"""
    bl_idname = "object.add_bone_constraints"
    bl_label = "3. Constr -> Pose -> Del Mod"
    bl_options = {'REGISTER', 'UNDO'}

    constraint_prefix: bpy.props.StringProperty(default="RigTool_") 

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        if not props: return False
        armature_obj = props.bake_armature_target
        target_mesh_vg = props.constraint_target
        return (armature_obj and armature_obj.name in context.view_layer.objects and armature_obj.pose and
                target_mesh_vg and target_mesh_vg.name in context.view_layer.objects)

    def execute(self, context):
        start_time = time.time()
        props = context.scene.mesh_rig_tool_props
        armature_obj = props.bake_armature_target
        target_mesh_vg = props.constraint_target
        mesh_parented = props.parenting_mesh_target 
        parented_armature = props.parenting_armature_target 
        add_copy_rotation = props.add_copy_rotation 

        if not armature_obj or armature_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Working Armature not set/found."); return {'CANCELLED'}
        if not target_mesh_vg or target_mesh_vg.name not in context.view_layer.objects: self.report({'ERROR'}, "Target Mesh (VG) not set/found."); return {'CANCELLED'}
        if not armature_obj.pose: self.report({'ERROR'}, f"Armature '{armature_obj.name}' has no Pose."); return {'CANCELLED'}
        if not target_mesh_vg.vertex_groups: self.report({'WARNING'}, f"Mesh '{target_mesh_vg.name}' (VG Target) has no vertex groups.") 
        if not armature_obj.data.bones: self.report({'WARNING'}, f"Armature '{armature_obj.name}' has no bones."); return {'CANCELLED'} 

        constraints_type_to_add = "Loc" + ("+Rot" if add_copy_rotation else "")
        self.report({'INFO'}, f"Step 3: Remove Mod -> Constraints ({constraints_type_to_add}) -> Pose...")
        
        bones_constrained_count = 0 
        constraints_added_total = 0 
        missing_vg_count = 0
        apply_pose_success = False
        removed_modifier_flag = False
        modifier_to_remove_name = None 

        original_active = context.view_layer.objects.active
        original_armature_mode = armature_obj.mode

        try:
            if mesh_parented and mesh_parented.name in context.view_layer.objects and \
               parented_armature and parented_armature.name in context.view_layer.objects:
                modifier_to_remove = None
                for mod in mesh_parented.modifiers:
                    if mod.type == 'ARMATURE' and mod.object == parented_armature:
                        modifier_to_remove = mod
                        modifier_to_remove_name = mod.name 
                        break
                if modifier_to_remove:
                    try:
                        mesh_parented.modifiers.remove(modifier_to_remove)
                        removed_modifier_flag = True
                    except Exception as mod_rem_e:
                        self.report({'WARNING'}, f"Could not remove Armature mod ('{modifier_to_remove_name}') from '{mesh_parented.name}': {mod_rem_e}")

            if context.view_layer.objects.active != armature_obj:
                 context.view_layer.objects.active = armature_obj 
            if original_armature_mode != 'POSE':
                bpy.ops.object.mode_set(mode='POSE')

            pose_bones = armature_obj.pose.bones
            vg_names = {vg.name for vg in target_mesh_vg.vertex_groups} 

            for pbone in pose_bones:
                bone_name = pbone.name
                target_vg_name = bone_name 
                added_constraints_to_this_bone = 0 

                if target_vg_name in vg_names:
                    for c in list(pbone.constraints):
                         if c.name.startswith(self.constraint_prefix):
                             try:
                                 pbone.constraints.remove(c)
                             except Exception: pass

                    try:
                        constraint_loc = pbone.constraints.new(type='COPY_LOCATION')
                        constraint_loc.name = f"{self.constraint_prefix}Loc_{bone_name[:min(len(bone_name), 50)]}" 
                        constraint_loc.target = target_mesh_vg
                        constraint_loc.subtarget = target_vg_name 
                        constraint_loc.use_x, constraint_loc.use_y, constraint_loc.use_z = True, True, True
                        constraints_added_total += 1
                        added_constraints_to_this_bone += 1
                    except Exception as e_loc:
                        print(f"Error adding COPY_LOCATION constraint to bone '{bone_name}': {e_loc}")

                    if add_copy_rotation: 
                        try:
                            constraint_rot = pbone.constraints.new(type='COPY_ROTATION')
                            constraint_rot.name = f"{self.constraint_prefix}Rot_{bone_name[:min(len(bone_name), 50)]}"
                            constraint_rot.target = target_mesh_vg
                            constraint_rot.subtarget = target_vg_name
                            constraint_rot.use_x, constraint_rot.use_y, constraint_rot.use_z = True, True, True
                            constraints_added_total += 1
                            added_constraints_to_this_bone += 1
                        except Exception as e_rot:
                            print(f"Error adding COPY_ROTATION constraint to bone '{bone_name}': {e_rot}")
                    
                    if added_constraints_to_this_bone > 0:
                         bones_constrained_count += 1
                else:
                    missing_vg_count += 1

            self.report({'INFO'}, f"Applying pose as Rest Pose for '{armature_obj.name}'...")
            try:
                bpy.ops.pose.select_all(action='SELECT')
                bpy.ops.pose.armature_apply(selected=False) 
                apply_pose_success = True
            except RuntimeError as e:
                self.report({'ERROR'}, f"Error applying Rest Pose: {e}"); apply_pose_success = False

        except Exception as e:
            self.report({'ERROR'}, f"Error during Step 3 actions: {e}")
            apply_pose_success = False 
        finally:
            if armature_obj.name in context.view_layer.objects and armature_obj.mode != original_armature_mode:
                try:
                    if context.view_layer.objects.active != armature_obj:
                        context.view_layer.objects.active = armature_obj
                    bpy.ops.object.mode_set(mode=original_armature_mode)
                except Exception: pass

            if original_active and original_active.name in context.view_layer.objects:
                if context.view_layer.objects.active != original_active:
                    context.view_layer.objects.active = original_active
            elif armature_obj.name in context.view_layer.objects:
                 context.view_layer.objects.active = armature_obj

        end_time = time.time()
        if not apply_pose_success:
             return {'CANCELLED'} 
        
        return {'FINISHED'}


# --- Surface Deform Add Operator ---
class RIG_OT_AddSurfaceDeform(bpy.types.Operator):
    """Adds a SurfaceDeform modifier and performs Bind"""
    bl_idname = "object.add_surface_deform_tool"
    bl_label = "3.5 Add SurfaceDeform & Bind"
    bl_options = {'REGISTER', 'UNDO'}

    modifier_name: bpy.props.StringProperty(default="RigTool_SurfaceDeform") 

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        if not props: return False
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
        if source_mesh == target_mesh: self.report({'WARNING'}, "Source and Target meshes are the same. Binding may be unpredictable.")

        self.report({'INFO'}, f"Adding/Updating SurfaceDeform on '{source_mesh.name}', target '{target_mesh.name}'...")

        original_active = context.view_layer.objects.active
        original_source_mode = source_mesh.mode
        mod = None
        bind_success = False
        mod_needs_adding = False
        mod_needs_rebinding = False

        try:
            mod = source_mesh.modifiers.get(self.modifier_name)

            if mod:
                if mod.type != 'SURFACE_DEFORM':
                    self.report({'ERROR'}, f"Existing modifier '{self.modifier_name}' is not type SurfaceDeform."); return {'CANCELLED'}

                if mod.target != target_mesh:
                    if mod.is_bound:
                         mod_needs_rebinding = True 
                    else:
                         mod.target = target_mesh 
                elif not mod.is_bound:
                     mod_needs_rebinding = True 
                else:
                     bind_success = True 

            else:
                mod_needs_adding = True
                mod_needs_rebinding = True 

            if mod_needs_adding:
                mod = source_mesh.modifiers.new(name=self.modifier_name, type='SURFACE_DEFORM')
                mod.target = target_mesh

            if mod and mod_needs_rebinding:
                if context.view_layer.objects.active != source_mesh:
                    context.view_layer.objects.active = source_mesh
                if source_mesh.mode != 'OBJECT':
                    bpy.ops.object.mode_set(mode='OBJECT')

                if mod.is_bound:
                    try:
                        bpy.ops.object.surfacedeform_bind(modifier=mod.name)
                    except RuntimeError: pass
                    if mod and mod.target != target_mesh:
                        mod.target = target_mesh

                try:
                    bpy.ops.object.surfacedeform_bind(modifier=mod.name)
                    context.view_layer.update() 
                    mod = source_mesh.modifiers.get(self.modifier_name) 
                    if mod and mod.is_bound:
                        bind_success = True
                    else:
                        bind_success = False
                except RuntimeError as bind_e:
                     self.report({'ERROR'}, f"Bind operator failed: {bind_e}")
                     bind_success = False

        except Exception as e:
            self.report({'ERROR'}, f"Unexpected error during SurfaceDeform step: {e}")
            bind_success = False 
        finally:
            if source_mesh.name in context.view_layer.objects and source_mesh.mode != original_source_mode:
                 try:
                      if context.view_layer.objects.active != source_mesh: context.view_layer.objects.active = source_mesh
                      bpy.ops.object.mode_set(mode=original_source_mode)
                 except Exception: pass
            if original_active and original_active.name in context.view_layer.objects:
                if context.view_layer.objects.active != original_active:
                    context.view_layer.objects.active = original_active
            elif source_mesh.name in context.view_layer.objects:
                 context.view_layer.objects.active = source_mesh

        return {'FINISHED'} if bind_success else {'CANCELLED'}


# --- Animation Baking Operator ---
class RIG_OT_BakeRigAction(bpy.types.Operator):
    """Bakes animation for the target armature, clearing constraints"""
    bl_idname = "object.bake_rig_action_tool"
    bl_label = "4. Bake Rig Action"
    bl_options = {'REGISTER', 'UNDO'}

    fixed_visual_keying = True      
    fixed_clear_constraints = True  
    fixed_clear_parents = False     
    fixed_use_current_action = True 
    fixed_bake_types = {'POSE'}     

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        if not props: return False
        armature_target = props.bake_armature_target
        return (armature_target and armature_target.name in context.view_layer.objects and
                armature_target.pose and armature_target.pose.bones)

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        armature_obj = props.bake_armature_target
        start_frame = props.bake_start_frame
        end_frame = props.bake_end_frame

        if not armature_obj or armature_obj.name not in context.view_layer.objects: self.report({'ERROR'}, "Target armature ('Working Armature') not found."); return {'CANCELLED'}
        if not armature_obj.pose or not armature_obj.pose.bones: self.report({'ERROR'}, f"Armature '{armature_obj.name}' has no Pose Bones."); return {'CANCELLED'}
        if end_frame < start_frame: self.report({'ERROR'}, "End frame cannot be before start frame."); return {'CANCELLED'}

        self.report({'INFO'}, f"Baking animation for '{armature_obj.name}' from frame {start_frame} to {end_frame}...")

        original_active = context.view_layer.objects.active
        original_selected = context.selected_objects[:]
        original_mode = armature_obj.mode
        original_frame = context.scene.frame_current
        bake_success = False

        try:
            if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.select_all(action='DESELECT')
            armature_obj.select_set(True)
            context.view_layer.objects.active = armature_obj
            if armature_obj.mode != 'POSE': bpy.ops.object.mode_set(mode='POSE')
            bpy.ops.pose.select_all(action='SELECT')

            bpy.ops.nla.bake(
                frame_start=start_frame,
                frame_end=end_frame,
                step=1,              
                only_selected=True,  
                visual_keying=self.fixed_visual_keying,
                clear_constraints=self.fixed_clear_constraints, 
                clear_parents=self.fixed_clear_parents,
                use_current_action=self.fixed_use_current_action,
                bake_types=self.fixed_bake_types
            )
            self.report({'INFO'}, "Baking completed successfully.")
            bake_success = True

        except RuntimeError as e:
            self.report({'ERROR'}, f"Error during baking: {e}")
            bake_success = False
        except Exception as e:
            self.report({'ERROR'}, f"Unexpected error during baking: {e}")
            bake_success = False
        finally:
            context.scene.frame_set(original_frame)
            if armature_obj.name in context.view_layer.objects and armature_obj.mode != original_mode:
                 try:
                      if context.view_layer.objects.active != armature_obj: context.view_layer.objects.active = armature_obj
                      bpy.ops.object.mode_set(mode=original_mode)
                 except Exception: pass
            elif context.mode != 'OBJECT':
                 try: bpy.ops.object.mode_set(mode='OBJECT')
                 except Exception: pass

            deselect_all_objects(context)
            for o in original_selected:
                if o and o.name in context.view_layer.objects: 
                    o.select_set(True)
            if original_active and original_active.name in context.view_layer.objects:
                 context.view_layer.objects.active = original_active
            elif bake_success and armature_obj.name in context.view_layer.objects:
                 context.view_layer.objects.active = armature_obj

        return {'FINISHED'} if bake_success else {'CANCELLED'}


# --- Step 5 Operator: Finalize Deformation ---
class RIG_OT_FinalizeDeform(bpy.types.Operator):
    """Removes SurfaceDeform and adds final Armature modifier to the Source Mesh"""
    bl_idname = "object.finalize_deform_tool"
    bl_label = "5. Finalize Deform"
    bl_options = {'REGISTER', 'UNDO'}

    surfdeform_modifier_name: bpy.props.StringProperty(default="RigTool_SurfaceDeform")
    armature_modifier_name: bpy.props.StringProperty(default="Final_Armature_Deform")

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        if not props: return False
        source_mesh = props.final_source_mesh
        target_armature = props.final_target_armature
        return (source_mesh and source_mesh.name in context.view_layer.objects and
                target_armature and target_armature.name in context.view_layer.objects)

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        source_mesh = props.final_source_mesh
        target_armature = props.final_target_armature

        if not source_mesh or source_mesh.name not in context.view_layer.objects: self.report({'ERROR'}, "Source mesh (for Finalize) not found."); return {'CANCELLED'}
        if not target_armature or target_armature.name not in context.view_layer.objects: self.report({'ERROR'}, "Target armature (for Finalize) not found."); return {'CANCELLED'}

        removed_sd = False
        added_armature = False
        removed_existing_armature = False

        original_active = context.view_layer.objects.active
        original_source_mode = source_mesh.mode

        try:
            sd_mod = source_mesh.modifiers.get(self.surfdeform_modifier_name)
            if sd_mod:
                if sd_mod.type == 'SURFACE_DEFORM':
                    if sd_mod.is_bound:
                         try:
                              if context.view_layer.objects.active != source_mesh:
                                   context.view_layer.objects.active = source_mesh
                              if source_mesh.mode != 'OBJECT':
                                   bpy.ops.object.mode_set(mode='OBJECT')
                              bpy.ops.object.surfacedeform_bind(modifier=sd_mod.name)
                         except Exception: pass
                    try:
                        source_mesh.modifiers.remove(sd_mod)
                        removed_sd = True
                    except Exception as e:
                        self.report({'WARNING'}, f"Could not remove SurfaceDeform modifier: {e}")

            existing_arm_mod_to_remove = None
            for mod in source_mesh.modifiers:
                 if mod.type == 'ARMATURE' and mod.object == target_armature:
                      existing_arm_mod_to_remove = mod
                      break 
            if existing_arm_mod_to_remove:
                 try:
                     source_mesh.modifiers.remove(existing_arm_mod_to_remove)
                     removed_existing_armature = True
                 except Exception: pass

            try:
                arm_mod = source_mesh.modifiers.new(name=self.armature_modifier_name, type='ARMATURE')
                arm_mod.object = target_armature 

                if source_mesh.vertex_groups and target_armature.data.bones:
                     bone_names = {b.name for b in target_armature.data.bones}
                     vg_names = {vg.name for vg in source_mesh.vertex_groups}
                     matching_groups = bone_names.intersection(vg_names) 

                     if matching_groups:
                         arm_mod.use_vertex_groups = True 
                     else:
                         arm_mod.use_vertex_groups = False 
                else:
                     arm_mod.use_vertex_groups = False 

                added_armature = True 
            except Exception as e:
                self.report({'ERROR'}, f"Failed to add Armature modifier: {e}")
                added_armature = False 

        except Exception as e:
            self.report({'ERROR'}, f"Unexpected error during finalization: {e}")
            added_armature = False 
        finally:
            if source_mesh.name in context.view_layer.objects and source_mesh.mode != original_source_mode:
                 try:
                      if context.view_layer.objects.active != source_mesh: context.view_layer.objects.active = source_mesh
                      bpy.ops.object.mode_set(mode=original_source_mode)
                 except Exception: pass
            if original_active and original_active.name in context.view_layer.objects:
                if context.view_layer.objects.active != original_active:
                    context.view_layer.objects.active = original_active
            elif source_mesh.name in context.view_layer.objects:
                 context.view_layer.objects.active = source_mesh

        if added_armature:
             return {'FINISHED'}
        else:
             return {'CANCELLED'}


# --- BATCH PROCESS OPERATOR ---
class RIG_OT_BatchProcess(bpy.types.Operator):
    """Batch process all meshes in a collection through the rigging pipeline"""
    bl_idname = "object.batch_process_rigs"
    bl_label = "Batch Process Collection"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        return props and props.batch_target_collection

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        collection = props.batch_target_collection
        
        mesh_objects = [obj for obj in collection.objects if obj.type == 'MESH']
        
        if not mesh_objects:
            self.report({'WARNING'}, f"No Mesh objects found in collection '{collection.name}'")
            return {'CANCELLED'}

        self.report({'INFO'}, f"Starting Batch Process for {len(mesh_objects)} objects...")
        
        original_active = context.view_layer.objects.active
        original_selected = context.selected_objects[:]
        
        success_count = 0
        
        for i, mesh_obj in enumerate(mesh_objects):
            self.report({'INFO'}, f"--- Processing object {i+1}/{len(mesh_objects)}: '{mesh_obj.name}' ---")
            
            # Ensure safe selection state
            if context.mode != 'OBJECT':
                try: bpy.ops.object.mode_set(mode='OBJECT')
                except: pass
            
            deselect_all_objects(context)
            
            mesh_obj.select_set(True)
            context.view_layer.objects.active = mesh_obj
            
            # 1. Create Rig (Implicitly handles Simple Mode via properties)
            res = bpy.ops.object.create_mesh_rig()
            if 'FINISHED' not in res:
                self.report({'ERROR'}, f"Batch: Failed to create rig for '{mesh_obj.name}'. Skipping.")
                continue
            
            # 2. Parent
            res = bpy.ops.object.parent_mesh_to_rig_tool()
            if 'FINISHED' not in res:
                self.report({'ERROR'}, f"Batch: Failed to parent '{mesh_obj.name}'.")
                continue
                
            # 3. Constraints & Pose
            res = bpy.ops.object.add_bone_constraints()
            if 'FINISHED' not in res:
                self.report({'ERROR'}, f"Batch: Failed to add constraints for '{mesh_obj.name}'.")
                continue
            
            # 4. Bake
            res = bpy.ops.object.bake_rig_action_tool()
            if 'FINISHED' not in res:
                self.report({'ERROR'}, f"Batch: Failed to bake '{mesh_obj.name}'.")
                continue
            
            # 5. Finalize
            if props.batch_finalize:
                res = bpy.ops.object.finalize_deform_tool()
                if 'FINISHED' not in res:
                    self.report({'WARNING'}, f"Batch: Finalize issues for '{mesh_obj.name}'.")
            
            # 6. Auto Clear Anim (Optional)
            if props.batch_auto_clear_anim:
                perform_cleanup(mesh_obj)
            
            success_count += 1
            
        deselect_all_objects(context)
        for o in original_selected:
            if o.name in context.view_layer.objects:
                o.select_set(True)
        if original_active and original_active.name in context.view_layer.objects:
            context.view_layer.objects.active = original_active

        self.report({'INFO'}, f"Batch Complete. Successfully processed {success_count}/{len(mesh_objects)} objects.")
        return {'FINISHED'}


# --- MERGE ARMATURES & MESHES OPERATOR (CONSTRAINT + BAKE METHOD) ---
class RIG_OT_MergeBatchArmatures(bpy.types.Operator):
    """Merge all armatures and meshes in the collection using Constraints & Baking (Reliable Method)"""
    bl_idname = "object.merge_batch_armatures"
    bl_label = "Merge Rigs in Collection"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        props = context.scene.mesh_rig_tool_props
        return props and props.batch_target_collection

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        collection = props.batch_target_collection
        
        # 1. Identify pairs
        mesh_objects = [obj for obj in collection.objects if obj.type == 'MESH']
        mesh_armature_pairs = []
        
        for mesh in mesh_objects:
            arm_mod = None
            for mod in mesh.modifiers:
                if mod.type == 'ARMATURE' and mod.object:
                    arm_mod = mod
                    break
            if arm_mod:
                mesh_armature_pairs.append({'mesh': mesh, 'arm': arm_mod.object})

        if not mesh_armature_pairs:
            self.report({'WARNING'}, "No suitable Mesh+Armature pairs found.")
            return {'CANCELLED'}

        self.report({'INFO'}, f"Merging {len(mesh_armature_pairs)} pairs...")
        
        # Determine Frame Range
        start_frame = context.scene.frame_start
        end_frame = context.scene.frame_end
        
        # --- 2. Create MASTER RIG ---
        if context.mode != 'OBJECT': bpy.ops.object.mode_set(mode='OBJECT')
        deselect_all_objects(context)
        
        master_data = bpy.data.armatures.new("Master_Rig_Data")
        master_rig = bpy.data.objects.new("Master_Rig", master_data)
        context.collection.objects.link(master_rig)
        context.view_layer.objects.active = master_rig
        
        # --- 3. Duplicate Bones & Constrain ---
        
        # 3a. Edit Mode: Create Bones
        bpy.ops.object.mode_set(mode='EDIT')
        edit_bones = master_data.edit_bones
        
        bone_mapping = [] # List of tuples (MasterBoneName, SourceArmObj, SourceBoneName, SourceMesh)
        
        for pair in mesh_armature_pairs:
            src_arm = pair['arm']
            src_mesh = pair['mesh']
            # Using mesh name as prefix ensures uniqueness
            prefix = f"{src_mesh.name}_" 
            
            for src_bone in src_arm.data.bones:
                new_name = prefix + src_bone.name
                
                new_bone = edit_bones.new(new_name)
                
                # --- FIX FOR ROTATION OFFSET ---
                mat_world = src_arm.matrix_world
                
                # 1. Set Geometry (Head/Tail)
                new_bone.head = mat_world @ src_bone.head_local
                new_bone.tail = mat_world @ src_bone.tail_local
                
                # 2. Set Orientation (Roll)
                # Get the local Z-axis vector of the source bone (rotation part of matrix)
                # matrix_local is relative to armature object, which works fine here as we use mat_world
                z_axis_local = src_bone.matrix_local.to_3x3().col[2]
                
                # Transform vector to world space (rotation only)
                z_axis_world = mat_world.to_3x3() @ z_axis_local
                
                # Align the new bone's roll to this world-space vector
                try:
                    new_bone.align_roll(z_axis_world)
                except ValueError:
                    pass
                
                bone_mapping.append({
                    'master_bone': new_name,
                    'source_arm': src_arm,
                    'source_bone': src_bone.name,
                    'source_mesh': src_mesh,
                    'original_bone_name': src_bone.name
                })

        # 3b. Pose Mode: Add Constraints
        bpy.ops.object.mode_set(mode='POSE')
        pose_bones = master_rig.pose.bones
        
        for item in bone_mapping:
            pb = pose_bones.get(item['master_bone'])
            if pb:
                c = pb.constraints.new('COPY_TRANSFORMS')
                c.target = item['source_arm']
                c.subtarget = item['source_bone']
                # Ensure we copy world space to world space
                c.target_space = 'WORLD'
                c.owner_space = 'WORLD'

        # --- 4. BAKE ACTION ---
        # This transfers all animation from sources to the master rig
        self.report({'INFO'}, "Baking animation to Master Rig...")
        try:
            bpy.ops.nla.bake(
                frame_start=start_frame,
                frame_end=end_frame,
                step=1,
                only_selected=False, # Bake all bones
                visual_keying=True,
                clear_constraints=True, # Remove constraints after baking
                use_current_action=True,
                bake_types={'POSE'}
            )
        except Exception as e:
            self.report({'ERROR'}, f"Baking failed: {e}")
            return {'CANCELLED'}
        
        # --- 5. Prepare Meshes (Rename VGs) ---
        bpy.ops.object.mode_set(mode='OBJECT')
        
        for item in bone_mapping:
            mesh = item['source_mesh']
            orig_name = item['original_bone_name']
            new_name = item['master_bone']
            
            vg = mesh.vertex_groups.get(orig_name)
            if vg:
                vg.name = new_name
                
        # --- 6. Join Meshes ---
        deselect_all_objects(context)
        for pair in mesh_armature_pairs:
            pair['mesh'].select_set(True)
            
        # Set active object for join
        if mesh_armature_pairs:
            master_mesh = mesh_armature_pairs[0]['mesh']
            context.view_layer.objects.active = master_mesh
            
            # Remove old Armature modifiers first to avoid confusion
            for pair in mesh_armature_pairs:
                m = pair['mesh']
                for mod in m.modifiers:
                    if mod.type == 'ARMATURE':
                        m.modifiers.remove(mod)

            bpy.ops.object.join()
            master_mesh.name = "Master_Batch_Mesh"
            
            # Add new Armature Modifier
            new_mod = master_mesh.modifiers.new(name="Master_Armature_Mod", type='ARMATURE')
            new_mod.object = master_rig
            
            # --- 7. Cleanup Source Armatures ---
            objs_to_delete = [pair['arm'] for pair in mesh_armature_pairs]
            objs_to_delete = list(set(objs_to_delete))
            
            deselect_all_objects(context)
            for obj in objs_to_delete:
                obj.select_set(True)
            bpy.ops.object.delete()
            
            # Select result
            deselect_all_objects(context)
            master_rig.select_set(True)
            master_mesh.select_set(True)
            context.view_layer.objects.active = master_rig

        self.report({'INFO'}, "Merge Complete (Constraint Method).")
        return {'FINISHED'}


# --- CLEAR OTHER ANIMATION OPERATOR ---
class RIG_OT_SmartClearAnimation(bpy.types.Operator):
    """Clears animation, shape keys, rigid bodies, and simulations from selected objects AND all meshes in the batch collection"""
    bl_idname = "object.clear_selected_animation"
    bl_label = "Smart Clear Cleanup"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.mesh_rig_tool_props
        
        # Gather objects
        objects_to_clean = set(context.selected_objects)
        
        # Add objects from Batch Collection if it exists
        if props.batch_target_collection:
            for obj in props.batch_target_collection.objects:
                if obj.type == 'MESH':
                    objects_to_clean.add(obj)
        
        if not objects_to_clean:
            self.report({'WARNING'}, "No objects selected or in Batch Collection.")
            return {'CANCELLED'}
        
        count = 0
        for obj in objects_to_clean:
            perform_cleanup(obj)
            count += 1
        
        self.report({'INFO'}, f"Cleaned up {count} objects.")
        return {'FINISHED'}


# --- UI Panel ---
class RIG_PT_MeshRigPanel(bpy.types.Panel):
    bl_label = "Mesh2Rig" 
    bl_idname = "RIG_PT_MeshRigPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'M2R' 

    def draw(self, context):
        layout = self.layout
        if not hasattr(context.scene, "mesh_rig_tool_props"):
             layout.label(text="Error: Properties not found.", icon='ERROR')
             return
        props = context.scene.mesh_rig_tool_props 

        # --- Section: Batch Processing ---
        box_batch = layout.box()
        col = box_batch.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="Batch Processing (Cell Fracture)", icon='GROUP')
        col.prop(props, "batch_target_collection")
        col.prop(props, "batch_finalize")
        col.prop(props, "batch_auto_clear_anim") # Auto Clear Anim Checkbox
        col.prop(props, "batch_simple_mode")
        
        col.separator()
        op_row_batch = col.row()
        op_row_batch.enabled = bool(props.batch_target_collection)
        op_row_batch.operator(RIG_OT_BatchProcess.bl_idname, text="Batch Process Collection", icon='PLAY')
        
        op_row_merge = col.row()
        op_row_merge.enabled = bool(props.batch_target_collection)
        op_row_merge.operator(RIG_OT_MergeBatchArmatures.bl_idname, text="Merge Generated Rigs", icon='OUTLINER_OB_ARMATURE')
        
        col.separator()
        col.label(text="Cleanup:")
        col.operator(RIG_OT_SmartClearAnimation.bl_idname, text="Clear Anim (Sel + Col)", icon='TRASH')
        
        layout.separator()

        # --- Section 1: Create Rig ---
        box_create = layout.box(); col = box_create.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="1. Create Rig (Optional)", icon='ARMATURE_DATA')
        settings = col.column(align=True)
        settings.prop(props, "bone_count")
        settings.prop(props, "proximity_threshold")
        settings.prop(props, "fixed_bone_length")
        col.separator()
        
        mesh_sel = get_selected_mesh_object() 
        op_row_create = col.row()
        op_row_create.enabled = (mesh_sel is not None) 
        op_row_create.operator(RIG_OT_CreateMeshRig.bl_idname, text="Create Rig from Selected Mesh", icon='PLUS')
        
        col.label(text="Select a MESH first. Sets targets below.", icon='INFO')
        info_row = col.row()
        info_row.enabled = False
        if mesh_sel:
            info_row.label(text=f"Source: {mesh_sel.name}", icon='MESH_DATA')
        else:
            info_row.label(text="No MESH selected", icon='ERROR')
        layout.separator()

        # --- Section 1.5: Merge Bones ---
        box_merge = layout.box(); col = box_merge.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="1.5 Merge Bones (Edit Mode)", icon='MOD_DECIM')
        settings = col.column(align=True)
        settings.prop(props, "merge_threshold_factor", text="Factor")
        settings.prop(props, "merge_min_count", text="Min Count")
        col.separator()
        
        active_obj = context.object
        is_arm_edit = (active_obj and active_obj.type == 'ARMATURE' and active_obj.mode == 'EDIT')
        sel_bones_count = len(context.selected_editable_bones) if is_arm_edit else 0
        min_count = props.merge_min_count
        can_merge = bool(is_arm_edit and sel_bones_count >= min_count) 
        op_row_merge = col.row()
        op_row_merge.enabled = can_merge
        op_row_merge.operator(RIG_OT_MergeSelectedBones.bl_idname, text="Merge Selected Bones", icon='AUTOMERGE_ON')
        
        info_row = col.row()
        info_row.enabled = False
        if not is_arm_edit:
            info_row.label(text="Requires Armature Edit Mode", icon='INFO')
        elif sel_bones_count < min_count:
            info_row.label(text=f"Select >= {min_count} bones ({sel_bones_count} sel.)", icon='INFO')
        else:
            info_row.label(text=f"Ready (Selected: {sel_bones_count})", icon='CHECKMARK')
        layout.separator()

        # --- Section 2: Parent Mesh ---
        box_parent = layout.box(); col = box_parent.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="2. Parent Mesh to Rig", icon='LINKED')
        settings = col.column(align=True)
        settings.prop(props, "parenting_mesh_target")
        settings.prop(props, "parenting_armature_target")
        col.separator()
        
        mesh_p = props.parenting_mesh_target; arm_p = props.parenting_armature_target
        valid_parent_targets = bool(mesh_p and arm_p and mesh_p.name in context.view_layer.objects and arm_p.name in context.view_layer.objects)
        op_row_parent = col.row()
        op_row_parent.enabled = valid_parent_targets
        op_row_parent.operator(RIG_OT_ParentMeshToRigTool.bl_idname, text="Parent with Auto Weights", icon='LINKED')
        
        info_row = col.row()
        info_row.enabled = False
        info_row.label(text=f"Mesh: {mesh_p.name if mesh_p else 'None'}", icon='MESH_DATA' if mesh_p else 'QUESTION')
        info_row = col.row()
        info_row.enabled = False
        info_row.label(text=f"Armature: {arm_p.name if arm_p else 'None'}", icon='ARMATURE_DATA' if arm_p else 'QUESTION')
        if not valid_parent_targets:
             col.label(text="Set valid Mesh and Armature targets.", icon='ERROR')
        layout.separator()

        # --- Section 3: Constraints -> Pose -> Del Mod ---
        box_con = layout.box(); col = box_con.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="3. Constr -> Pose -> Del Mod", icon='CONSTRAINT_BONE')
        settings = col.column(align=True)
        settings.prop(props, "bake_armature_target", text="Armature")
        settings.prop(props, "constraint_target")
        settings.prop(props, "add_copy_rotation")
        col.separator()
        
        arm_bake_c = props.bake_armature_target
        mesh_vg = props.constraint_target
        valid_constr_targets = bool(arm_bake_c and mesh_vg and arm_bake_c.name in context.view_layer.objects and mesh_vg.name in context.view_layer.objects and arm_bake_c.pose)
        op_row_constr = col.row()
        op_row_constr.enabled = valid_constr_targets
        op_row_constr.operator(RIG_OT_AddBoneConstraints.bl_idname, text="Execute Step 3", icon='HOOK')
        
        mesh_mod = props.parenting_mesh_target; par_arm = props.parenting_armature_target
        col.label(text=f"Armature: {arm_bake_c.name if arm_bake_c else 'Select Working Armature!'}", icon='ARMATURE_DATA' if arm_bake_c else 'ERROR')
        col.label(text=f"VG Target: {mesh_vg.name if mesh_vg else 'Select VG Target Mesh!'}", icon='MESH_DATA' if mesh_vg else 'ERROR')
        col.label(text=f"Del Mod from: {mesh_mod.name if mesh_mod else '(Not Set/Used)'}", icon='MODIFIER' if mesh_mod else 'INFO')
        col.label(text=f"  (Targeting: {par_arm.name if par_arm else 'N/A'})", icon='MODIFIER_OFF' if (mesh_mod and par_arm) else 'INFO')
        if not valid_constr_targets:
             col.label(text="Set valid Armature (with Pose) and VG Target.", icon='ERROR')
        layout.separator()

        # --- Section 3.5: Surface Deform ---
        box_sd = layout.box(); col = box_sd.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="3.5 Surface Deform", icon='MOD_MESHDEFORM')
        settings = col.column(align=True)
        settings.prop(props, "surfdeform_source_mesh")
        settings.prop(props, "surfdeform_target_mesh")
        col.separator()
        
        mesh_sd_s = props.surfdeform_source_mesh; mesh_sd_t = props.surfdeform_target_mesh
        valid_sd_targets = bool(mesh_sd_s and mesh_sd_t and mesh_sd_s.name in context.view_layer.objects and mesh_sd_t.name in context.view_layer.objects)
        op_row_sd = col.row()
        op_row_sd.enabled = valid_sd_targets
        op_row_sd.operator(RIG_OT_AddSurfaceDeform.bl_idname, text="Add/Update SD & Bind", icon='MOD_MESHDEFORM')
        
        col.label(text=f"Source: {mesh_sd_s.name if mesh_sd_s else 'None'}", icon='MESH_DATA' if mesh_sd_s else 'QUESTION')
        col.label(text=f"Target: {mesh_sd_t.name if mesh_sd_t else 'None'}", icon='MESH_DATA' if mesh_sd_t else 'QUESTION')
        if not valid_sd_targets:
             col.label(text="Set valid Source and Target Meshes.", icon='ERROR')
        layout.separator()

        # --- Section 4: Bake Action ---
        box_bake = layout.box(); col = box_bake.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="4. Bake Action", icon='ACTION')
        settings = col.column(align=True)
        settings.prop(props, "bake_armature_target", text="Armature")
        row_frames = settings.row(align=True)
        row_frames.prop(props, "bake_start_frame", text="Start")
        row_frames.prop(props, "bake_end_frame", text="End")
        col.separator()
        
        arm_bake_b = props.bake_armature_target
        valid_bake_target = bool(arm_bake_b and arm_bake_b.name in context.view_layer.objects and arm_bake_b.pose and arm_bake_b.pose.bones)
        op_row_bake = col.row()
        op_row_bake.enabled = valid_bake_target
        op_row_bake.operator(RIG_OT_BakeRigAction.bl_idname, text="Bake Action for Armature", icon='RENDER_ANIMATION')
        
        col.label(text=f"Armature: {arm_bake_b.name if arm_bake_b else 'Select Armature!'}", icon='ARMATURE_DATA' if arm_bake_b else 'ERROR')
        if not valid_bake_target:
             col.label(text="Set valid Armature with Pose Bones.", icon='ERROR')
        layout.separator()

        # --- Section 5: Finalize Deform ---
        box_fin = layout.box(); col = box_fin.column()
        row = col.row(align=True); row.alignment = 'LEFT'; row.label(text="5. Finalize Deform", icon='MOD_ARMATURE')
        settings = col.column(align=True)
        settings.prop(props, "final_source_mesh")
        settings.prop(props, "final_target_armature", text="Armature")
        col.separator()
        
        mesh_fin_s = props.final_source_mesh; arm_fin_t = props.final_target_armature
        valid_fin_targets = bool(mesh_fin_s and arm_fin_t and mesh_fin_s.name in context.view_layer.objects and arm_fin_t.name in context.view_layer.objects)
        op_row_fin = col.row()
        op_row_fin.enabled = valid_fin_targets
        op_row_fin.operator(RIG_OT_FinalizeDeform.bl_idname, text="Remove SD, Add Armature Mod", icon='CHECKMARK')
        
        col.label(text=f"Source Mesh: {mesh_fin_s.name if mesh_fin_s else 'None'}", icon='MESH_DATA' if mesh_fin_s else 'QUESTION')
        col.label(text=f"Target Armature: {arm_fin_t.name if arm_fin_t else 'None'}", icon='ARMATURE_DATA' if arm_fin_t else 'QUESTION')
        if not valid_fin_targets:
             col.label(text="Set valid Source Mesh and Target Armature.", icon='ERROR')

# --- Registration ---

classes = (
    MeshRigProperties,
    RIG_OT_CreateMeshRig,
    RIG_OT_MergeSelectedBones,
    RIG_OT_ParentMeshToRigTool,
    RIG_OT_AddBoneConstraints,
    RIG_OT_AddSurfaceDeform,
    RIG_OT_BakeRigAction,
    RIG_OT_FinalizeDeform,
    RIG_OT_BatchProcess,      
    RIG_OT_MergeBatchArmatures, 
    RIG_OT_SmartClearAnimation, 
    RIG_PT_MeshRigPanel,
)

def register():
    print("Registering Mesh2Rig Addon...")
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except Exception as e:
            print(f"  Failed to register class {cls.__name__}: {e}")
    try:
        bpy.types.Scene.mesh_rig_tool_props = bpy.props.PointerProperty(type=MeshRigProperties)
        print("  Registered Scene.mesh_rig_tool_props")
    except Exception as e:
        print(f"  Failed to register Scene PointerProperty: {e}")
    print("Mesh2Rig Registration Complete.")


def unregister():
     print("Unregistering Mesh2Rig Addon...")
     if hasattr(bpy.types.Scene, "mesh_rig_tool_props"):
         try:
             del bpy.types.Scene.mesh_rig_tool_props
             print("  Unregistered Scene.mesh_rig_tool_props")
         except Exception as e:
             print(f"  Could not delete mesh_rig_tool_props from Scene: {e}")

     for cls in reversed(classes):
         try:
             bpy.utils.unregister_class(cls)
         except RuntimeError:
             pass
         except Exception as e:
              print(f"  Error unregistering class {cls.__name__}: {e}")
     print("Mesh2Rig Addon Unregistered.")

if __name__ == "__main__":
    try:
        unregister()
    except Exception as e:
        print(f"Error during automatic unregister on script run: {e}")
    register()
