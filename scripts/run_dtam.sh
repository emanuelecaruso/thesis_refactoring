#Name of the .blend file inside "blender_scenes" directory
cd code

# Blender datasets
# dataset_name='bunny_scene_cycles'
# dataset_name='bunny_scene'
# dataset_name='cube_scene'

# dataset_name='bunny_1'
# dataset_name='bunny_2'
# dataset_name='bunny_3'
# dataset_name='bunny_4'
dataset_name='bunny_5'
# dataset_name='courtyard_1'
# dataset_name='courtyard_2'
# dataset_name='courtyard_3'
# dataset_name='courtyard_4'
# dataset_name='courtyard_5'
# dataset_name='cube_1'
# dataset_name='cube_2'
# dataset_name='cube_3'
# dataset_name='cube_4'
# dataset_name='cube_5'
# dataset_name='spaceship_1'
# dataset_name='spaceship_2'
# dataset_name='spaceship_3'
# dataset_name='spaceship_4'
# dataset_name='spaceship_5'
# dataset_name='spaceship_5'

# TUM datasets
# dataset_name='sequence_01'
# dataset_name='sequence_03'
# dataset_name='sequence_30'
# dataset_name='sequence_44'


# run
# ./build/executables/test_dso ${dataset_name}
./build/executables/test_coeffs ${dataset_name}
# ./build/executables/test_tracking ${dataset_name}
