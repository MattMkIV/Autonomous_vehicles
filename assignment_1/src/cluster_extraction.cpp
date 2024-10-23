#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include <cmath>
#include <bits/stdc++.h>

using namespace std;

//#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree,
               float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
    if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point{cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};

        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize,
                                                int setMaxClusterSize)
{
    my_visited_set_t visited{}; //already visited points
    std::vector<pcl::PointIndices> clusters; //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster; //vector of int that is used to store the points that the function proximity will give me back

    //for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)
    //    end if
    //  end if
    //end for
    for (int i = 0; i < cloud->size(); i++)
    {
        if (visited.find(i) == visited.end())
        {
            proximity(cloud, i, tree, distanceTol, visited, cluster, setMaxClusterSize);
        }
        if (cluster.size() > setMinClusterSize && cluster.size() < setMaxClusterSize)
        {
            pcl::PointIndices p;
            for (int j = 0; j < cluster.size(); j++)
            {
                p.indices.push_back(cluster[j]);
            }
            clusters.push_back(p);
        }
        cluster.clear();
    }
    return clusters;
}

void ProcessAndRenderPointCloud(Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const string& showPlane = 0)
{
    // Defining colors for rendering
    std::vector<Color> colors = {
        Color(1, 0, 0),           // 0. Red
        Color(1, 1, 0),           // 1. Yellow
        Color(0, 0, 1),           // 2. Blue
        Color(1, 0, 1),           // 3. Magenta
        Color(0, 1, 1),           // 4. Cyan
        Color(0.2f, 0.2f, 0.2f),  // 5. Dark Gray
        Color(0.8f, 0.8f, 0.3f),         // 6. Darker Light Yellow
        Color(1, 1, 0.5f),         // 7. Light Yellow
    };

    // TODO: 1) Downsample the dataset
    // Defining the point cloud which will be updated through the steps
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Voxel filtering, point cloud reduction
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.30f, 0.30f, 0.30f); // set the Voxel grid leaf size (0.25 = 25cm): float x, float y, float z
    sor.filter(*cloud_filtered); // Applying filter

    // TODO: 2) Here we crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(Eigen::Vector4f(-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f(30, 7, 5, 1));
    cb.filter(*cloud_filtered);

    // TODO: 3) Segmentation with inliers removal and apply RANSAC
    // Defining a new point cloud to apply the plane catting
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZ>);
    // Point cloud containing the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Defining Model and Method algorithm
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(400);
    seg.setDistanceThreshold(0.2); // 0.2 instead of 1 crease the accuracy of plane detection

    // Determines how close a point must be to the model in order to be considered an inlier
    int i = 0, nr_points = (int)cloud_filtered->size();

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // The resultant model coefficients
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // While 70% of the original cloud is still there, higher value -> lower planar removal
    while (cloud_filtered->size() > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        /*
        Base method for segmentation of a model in a PointCloud given by <setInputCloud (), setIndices ()>
        [out]	inliers	the resultant point indices that support the model found (inliers)
        [out]	model_coefficients	the resultant model coefficients that describe the plane
        */
        seg.segment(*inliers, *coefficients); // We get one of the planes, and we put it into the inliers variable
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Extract the inliers (here we extract the points of the plane moving the indices representing the plane to cloud_segmented)
        extract.setInputCloud(cloud_filtered);

        // PCL defines a way to define a region of interest / list of point indices that the algorithm should operate on,
        // rather than the entire cloud, via setIndices
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Retrieve indices to all points in cloud_filtered but only those referenced by inliers:
        extract.filter(*cloud_segmented); // We effectively retrieve JUST the plane

        // Rendering the plane removed in CloudSegmented if showPlane == 1
        if (showPlane == "1")
            renderer.RenderPointCloud(cloud_segmented, "CloudSegmented", colors[5]);

        // Here we will extract the plane from the original filtered point cloud
        extract.setNegative(true); // original cloud - plane
        extract.filter(*cloud_aux); // We write into cloud_aux the cloud without the extracted plane

        cloud_filtered.swap(cloud_aux); // Here we swap the cloud (the removed plane one) with the original
        i++;
    }

#ifdef USE_PCL_LIBRARY
    // TODO: 5) Create the KDTree and the vector of PointIndices
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    //Set the spatial tolerance for new cluster candidates
    //If you take a very small value, it can happen that an actual object can be seen as multiple clusters.
    //On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster
    ec.setClusterTolerance(0.40);

    //We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
    ec.setMinClusterSize(25);
    ec.setMaxClusterSize(500);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);

    // Here we are creating a vector of PointIndices, which contain the actual index information in a vector<int>. The indices of each detected cluster are saved here.
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
#else
    // Here we are creating a vector of PointIndices, which contain the actual index information in a vector<int>.
    // The indices of each detected cluster are saved here.
    std::vector<pcl::PointIndices> cluster_indices;

    my_pcl::KdTree treeM;
    treeM.set_dimension(3);
    setupKdtree(cloud_filtered, &treeM, 3);
    cluster_indices = euclideanCluster(cloud_filtered, &treeM, 0.4, 25, 500);
#endif
    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices.

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    Eigen::Vector3f lidar_origin(0.0f, 0.0f, 0.0f);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud_filtered)[*pit]);

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // TODO: 7) render the cluster and plane without rendering the original cloud
        renderer.RenderPointCloud(cloud_filtered, "FilteredCloud" + std::to_string(j), colors[1]);

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        getMinMax3D(*cloud_cluster, minPt, maxPt);

        size_t num_points = cloud_cluster->size();
        float volume = (maxPt.x - minPt.x) * (maxPt.y - minPt.y) * (maxPt.z - minPt.z);
        float density = (num_points > 0) ? static_cast<float>(num_points) / volume : 0.0f;

        if ((maxPt.z - minPt.z > 1 && maxPt.z - minPt.z < 2 && density > 6 && density < 50.0))
        {
            // TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
            // Adding Ray, calculating the center point of the cluster
            Eigen::Vector4f centroid;
            compute3DCentroid(*cloud_cluster, centroid);

            pcl::PointCloud<pcl::PointXYZ>::Ptr centerCloud(new pcl::PointCloud<pcl::PointXYZ>);

            // Create the center point and add it to the new cloud
            pcl::PointXYZ centerPoint;
            centerPoint.x = centroid[0];
            centerPoint.y = centroid[1];
            centerPoint.z = centroid[2];

            centerCloud->points.push_back(centerPoint);
            centerCloud->width = 1; // Set width to 1 since we have one point
            centerCloud->height = 1; // Set height to 1 for a single point cloud
            centerCloud->is_dense = true;

            renderer.RenderRays(lidar_origin, centerCloud);

            // Distance between the origin lidar point and the center of the cluster
            float distance = sqrt(pow(lidar_origin[0] - centerPoint.x, 2) +
                pow(lidar_origin[1] - centerPoint.y, 2) +
                pow(lidar_origin[2] - centerPoint.z, 2));

            std::ostringstream stream;
            stream << std::fixed << std::setprecision(2) << distance;
            std::string r = stream.str();

            renderer.addText(centroid[0], centroid[1], centroid[2], r);

            //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
            //please take a look at the function RenderBox to see how to color the box
            Box box{
                minPt.x, minPt.y, minPt.z,
                maxPt.x, maxPt.y, maxPt.z
            };

            // Check if the distance between the car and box is 5<distance<10 (Dark yellow color)
            if(distance > 5 && distance < 10)
                renderer.RenderBox(box, j, colors[6]);

            // Check if the distance is >10 (Light Yellow)
            else if(distance > 10)
                renderer.RenderBox(box, j, colors[7]);

            // Checks if the box is in front of the car (Cyan)
            else if (distance < 5.00 && lidar_origin[0] > centroid[0])
                renderer.RenderBox(box, j, colors[4]);

            // Otherwise if the box is in the back of the car (Magenta)
            else if (distance < 5.00 && lidar_origin[0] < centroid[0])
                renderer.RenderBox(box, j, colors[3]);

            ++clusterId;
            j++;
        }
    }
}


int main(int argc, char* argv[])
{
    std::string showPlane;
    if (argc > 1)
         showPlane= argv[1];
    else
        showPlane = "0";

    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);

    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(
        boost::filesystem::directory_iterator{"/home/linux/Desktop/Autonomous_vehicles/assignment_1/datasets/dataset_2"},
        boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;

        reader.read(streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer, input_cloud, showPlane);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        std::string color;
        if (elapsedTime.count() > 20)
        {
            color = "\033[31m"; // Red
        }
        else
        {
            color = "\033[32m"; // Green
        }
        std::string resetColor = "\033[0m"; // Reset to default color

        std::cout << "Loaded "
            << input_cloud->points.size() << " data points from " << streamIterator->string() <<
            " plane segmentation took " << color << elapsedTime.count() << " milliseconds" << resetColor << std::endl;

        ++streamIterator;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }

}
