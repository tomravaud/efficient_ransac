# Efficient RANSAC

## Installation

```bash
git clone https://github.com/TomRavaud/efficient_ransac.git
cd efficient_ransac
conda env create -f environment.yaml
conda activate efficient_ransac
```

## Usage

### Data

You can use the sample cloud provided in the `data` folder or use your own point cloud. The point cloud should be a
`.ply` file with the following properties:

- `x`, `y`, `z`: coordinates of the points
- `normal_x`, `normal_y`, `normal_z`: normals of the points (optional, else you should consider preprocessing the point cloud)
- `curvature`: curvature of the points (optional, else you should consider preprocessing the point cloud)

We also provide a script to generate a synthetic point cloud composed of random planes, cylinders, and spheres (point coordinates only, normals are to be computed).

```bash
python examples/generate_synthetic_cloud.py "data/synthetic_cloud.ply"
```


### Preprocessing

Our implementation assumes that the input point cloud is centered and that the normals are available. If not, you can use the following script to preprocess the point cloud.

```bash
python examples/preprocess_cloud.py "data/synthetic_cloud.ply"
```

Note that the parameters of the preprocessing may need to be adjusted depending on the input point cloud.


### Detection

```bash
python examples/run_detection.py "data/synthetic_cloud_preprocessed.ply"
```

![image_detection](assets/image_detection.jpeg)


## TODO

- [ ] Add documentation
- [ ] Add tests
