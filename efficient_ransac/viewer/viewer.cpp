#include "viewer.h"


Viewer::Viewer() {
    viewer = nullptr;
}

void Viewer::setup() {
    if (viewer) {
        return;
    }
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("Viewer")
    );
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.05);
    viewer->initCameraParameters();
    viewer->setCameraPosition(
        0, 0, 3,  // position
        0, 0, 0,  // viewpoint
        0, -1, 0  // up
    );

    viewer->addText("Press 'q' to quit", 10, 20, 20, 1, 1, 1, "quit");
    viewer->addText("Press 'r' to reset camera", 10, 40, 20, 1, 1, 1, "reset");
}

void Viewer::showCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    if (!viewer) {
        setup();
    }
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1,
        "cloud"
    );
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Viewer::showCloud(const std::filesystem::path &filepath) {
    if (!viewer) {
        setup();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath.string(), *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filepath.string().c_str());
        return;
    }
    showCloud(cloud);
}
