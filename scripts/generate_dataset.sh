#Name of the .blend file inside "blender_scenes" directory
# dataset_name='bunny_scene'
# dataset_name='cube_scene'
# dataset_name='scene_roll'
# dataset_name='bunny_scene_roll'
# dataset_name='bunny_scene_zoom'
# dataset_name='bunny_scene_exposure'

dataset_name='bunny_1'
# dataset_name='bunny_2'
# dataset_name='bunny_3'
# dataset_name='bunny_4'
# dataset_name='bunny_5'

#Number of samples for CYCLES pathtracer
samples=128

cd ./blender_scenes

#Render with CYCLES engine
# blender --background ./${dataset_name}.blend --python ./python_scripts/script.py CYCLES ${samples}

#Render with EEVEE engine
blender --background ./${dataset_name}.blend --python ./python_scripts/script.py

dataset_name='courtyard_3'
blender --background ./${dataset_name}.blend --python ./python_scripts/script.py
