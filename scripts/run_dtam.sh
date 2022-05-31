#Name of the .blend file inside "blender_scenes" directory
cd code

# Blender datasets
# dataset_name='bunny_scene_cycles'
dataset_name='bunny_scene'
# dataset_name='cube_scene'

# TUM datasets
# dataset_name='sequence_02'
# dataset_name='sequence_03'
# dataset_name='sequence_30'
# dataset_name='sequence_44'


# run
./build/executables/test_dso ${dataset_name}



# ./build/executables/test_tracking ${dataset_name}
# ./build/executables/test_mapping ${dataset_name}
# ./build/executables/eval_orb_initializer ${dataset_name}

# blender ../blender_scenes/${dataset_name}.blend --python ../blender_scenes/python_scripts/run.py
