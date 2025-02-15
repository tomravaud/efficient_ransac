# Efficient RANSAC

## Installation

```bash
git clone https://github.com/TomRavaud/efficient_ransac.git
cd efficient_ransac
conda env create -f environment.yaml
conda activate efficient_ransac
```

## Usage

### Preprocessing

Our implementation assumes that the input point cloud is centered and that the normals are available. If not, you can use the following script to preprocess the point cloud.

```bash
python examples/preprocess_cloud.py
```

Note that the parameters of the preprocessing may need to be adjusted depending on the input point cloud.

### Detection

```bash
python examples/run_detection.py
```


## TODO

- [ ] Add documentation
- [ ] Add tests
