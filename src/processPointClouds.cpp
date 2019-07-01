// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"

namespace {
    template<typename PointT>
    void proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int idx, std::vector<uint8_t>& processed,
                   KdTree<PointT>* tree, float distanceTol, pcl::PointIndices& cluster)
    {
        processed[idx] = 1;
        cluster.indices.push_back(idx);
        std::vector<int> found = tree->search(cloud->points[idx], distanceTol);
        for (size_t i = 0; i < found.size(); ++i) {
            if (processed[found[i]])
                continue;
            proximity(cloud, found[i], processed, tree, distanceTol, cluster);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    auto tree = new KdTree<PointT>();
    tree->build(cloud);

    std::vector<pcl::PointIndices> clusters;
    const size_t n = cloud->points.size();
	std::vector<uint8_t> processed(n, 0);
	for (size_t i = 0; i < n; ++i)
	{
		if (processed[i])
			continue;

		pcl::PointIndices cluster;
		proximity(cloud, i, processed, tree, distanceTol, cluster);
        if (cluster.indices.size() >= minSize && cluster.indices.size() <= maxSize)
		    clusters.emplace_back(std::move(cluster));
	}

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;
    for (auto it = clusters.begin(); it != clusters.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloudCluster->points.push_back(cloud->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        cloudClusters.emplace_back(std::move(cloudCluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return cloudClusters;
}

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time filtering process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr filteredCloud{new pcl::PointCloud<PointT>};

    // Filter point cloud
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filteredCloud);

    // Crop the region of interest
    pcl::CropBox<PointT> roi;
    roi.setInputCloud(filteredCloud);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.filter(*filteredCloud);

    // Remove ego car points
    const Eigen::Vector4f minPointCar = {-1.5f, -1.5f, -1.0f, 1.0f};
    const Eigen::Vector4f maxPointCar = {2.7f, 1.5f, -0.4f, 1.0f};
    pcl::PointIndices::Ptr carPoints (new pcl::PointIndices());
    pcl::CropBox<PointT> roof;
    roof.setInputCloud(filteredCloud);
    roof.setMin(minPointCar);
    roof.setMax(maxPointCar);
    roof.setNegative(true);
    roof.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plainCloud{new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr obstCloud{new pcl::PointCloud<PointT>};

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plainCloud);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, plainCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));
	
	const size_t n = cloud->points.size();
    std::vector<int> tmp;
    tmp.reserve(n);
    std::vector<int> inliersResult;
    inliersResult.reserve(n);
	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	for(int i = 0 ; i < maxIterations; ++i) {
		// select three points
		int p1 = std::rand() % n;
		int p2 = p1, p3 = p1;
		while (p1 == p2) {
			p2 = std::rand() % n;
		}

		while (p3 == p1 || p3 == p2) {
			p3 = std::rand() % n;
		}

        const auto & point1 = cloud->points[p1];
        const auto & point2 = cloud->points[p2];
        const auto & point3 = cloud->points[p3];

		const float x1 = point1.x, x2 = point2.x, x3 = point3.x;
		const float y1 = point1.y, y2 = point2.y, y3 = point3.y;
		const float z1 = point1.z, z2 = point2.z, z3 = point3.z;

		const float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		const float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		const float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		const float d = -(a*x1 + b*y1 + c*z1);

		const float norm = std::sqrt(a*a + b*b + c*c);
        //std::cout << "Norm: " << norm << std::endl;
		tmp.clear();
		for (size_t j = 0; j < cloud->points.size(); ++j) {
			const auto& point = cloud->points[j];
			const float dist = std::abs(a*point.x + b*point.y + c*point.z + d) / norm;
        
			if (dist <= distanceThreshold) {
                //std::cout << "Dist: " << dist << std::endl;
				tmp.push_back(j);
			}
		}

		if (tmp.size() > inliersResult.size()) {
			swap(inliersResult, tmp);
		}
	}
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    inliers->indices = std::move(inliersResult);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract (cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.emplace_back(std::move(cloud_cluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // PCA
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // Ignore z rotation.
    // First rearrange eigen vector matrix
    // e3 vector will be a vector with max Z coordinate
    // We want to project e1 and e2 on XY plane, so if one of them haz max Z we should swap it with e3
    Eigen::Vector3f::Index idx;
    eigenVectorsPCA.row(2).cwiseAbs().maxCoeff(&idx);
    eigenVectorsPCA.col(idx) = eigenVectorsPCA.col(2);
    // making e1, e2, e3 a right handed vector system
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    
    // We need to rotate basis so that e3 become collinear with z
    // x axis rotation
    const float y = eigenVectorsPCA.col(2)[1];
    const float z1 = eigenVectorsPCA.col(2)[2];
    const float norm1 = std::sqrt(y * y + z1 * z1);
    Eigen::Matrix3f xAxisRot;
    xAxisRot << 1, 0, 0,
                0, z1/norm1, -y/norm1,
                0, y/norm1, z1/norm1;
    eigenVectorsPCA = xAxisRot * eigenVectorsPCA;
    // y axis rotation
    const float x = eigenVectorsPCA.col(2)[0];
    const float z2 = eigenVectorsPCA.col(2)[2];
    const float norm2 = std::sqrt(x * x + z2 * z2);
    Eigen::Matrix3f yAxisRot;
    yAxisRot << z2/norm2, 0, -x/norm2,
                0, 1, 0,
                x/norm2, 0, z2/norm2;
    eigenVectorsPCA = yAxisRot * eigenVectorsPCA;
 
    // Form transformation matrix.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    // Transform centroid coordinates
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    // Transform the original cloud.
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    BoxQ box;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    // Bbox quaternion and transforms
    box.bboxQuaternion = Eigen::Quaternionf(eigenVectorsPCA);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}