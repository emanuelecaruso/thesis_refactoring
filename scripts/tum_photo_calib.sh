
# TUM datasets
# dataset_name='sequence_02'
# dataset_name='sequence_03'
dataset_name='sequence_30'
# dataset_name='sequence_44'

cd ./code/dataset/${dataset_name}
mkdir -p calibrated
cd ./calibrated

# run
# ../../tum_photo_calib/mono_dataset_code-master/build/bin/responseCalib
../../../../tum_photo_calib/mono_dataset_code-master/build/bin/playDataset ../ w
