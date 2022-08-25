#Name of the .blend file inside "blender_scenes" directory
cd code

# Blender datasets
# dataset_name='bunny_scene_cycles'
# dataset_name='bunny_scene'
# dataset_name='cube_scene'

dataset_name='bunny_1'
# dataset_name='bunny_2'
# dataset_name='bunny_3'
# dataset_name='bunny_4'
# dataset_name='bunny_5'

# TUM datasets
# dataset_name='sequence_02'
# dataset_name='sequence_03'
# dataset_name='sequence_30'
# dataset_name='sequence_44'


# run
./build/executables/test_dso ${dataset_name}
