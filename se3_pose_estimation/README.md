# Geomstats Pose Estimation Example

This example trains a pose estimation network using the SE3 Geodesic Loss function.

## Requirements:

* geomstats
* imageio
* numpy
* scikit-image
* tensorflow
* tqdm

## Generating Dataset

This example uses the original King's College [dataset](http://mi.eng.cam.ac.uk/projects/relocalisation/#dataset) by Kendall et al. To create TFRecords of the dataset, run:

```
python make_dataset_kingscollege.py \
	--root_dir path/to/KingsCollege \
	--dataset dataset_train.txt \
	--out_file dataset_train.tfrecord

```

This command needs to be run twice; once for ```dataset_train.txt``` and once for ```dataset_test.txt```

## Train Network

To train the network run:

```
python train_se3_kingscollege.py --dataset path/to/dataset_train.tfrecord
```

Optional Parameters:

```
Train SE3 PoseNet Inception v1 Model.

optional arguments:
  -h, --help            show this help message and exit
  --batch_size BATCH_SIZE
                        Batch size to train.
  --init_lr INIT_LR     Initial Learning rate.
  --max_iter MAX_ITER   The number of iteration to train.
  --epsilon EPSILON     Gradient Epsilon
  --snapshot SNAPSHOT   Save model weights every X iterations
  --dataset DATASET     Training dataset
  --model_dir MODEL_DIR
                        The path to the model directory.
  --logs_path LOGS_PATH
                        The path to the logs directory.
  --resume              Resume training from previous saved checkpoint.
  --cuda CUDA           Specify default GPU to use.
  --debug               Enables debugging mode.
```


## Evaluating 

To evaluate the trained model:

```
python test_se3_kingscollege.py --dataset path/to/dataset_test.tfrecord --model_dir /path/to/model_saved_weights
```

Optional Parameters:

```
Test SE3 PoseNet Inception v1 Model.

optional arguments:
  -h, --help            show this help message and exit
  --model_dir MODEL_DIR
                        The path to the model directory.
  --dataset DATASET     The path to the TFRecords dataset.
  --cuda CUDA           Specify default GPU to use.
  --debug               Enables debugging mode.
```