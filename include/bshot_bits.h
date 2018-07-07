#ifndef BSHOT_BITS_H
#define BSHOT_BITS_H

#include <bshot_headers_bits.h>

template< typename T >
T minVect(const T *v, int n, int *ind=NULL)
{
    assert(n > 0);

    T min = v[0];
    if (ind != NULL) *ind = 0;
    for (int i=1; i<n; i++)
        if (v[i] < min) {
            min = v[i];
            if (ind != NULL) *ind=i;
        }

    return min;
}


class bshot_descriptor
{
public:
    std::bitset< 352 > bits;
};


class bshot
{

public :

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    pcl::PointCloud<pcl::Normal> cloud1_normals, cloud2_normals;
    pcl::PointCloud<pcl::PointXYZ> cloud1_keypoints, cloud2_keypoints;

    pcl::PointCloud<pcl::SHOT352> cloud1_shot, cloud2_shot;

    std::vector<bshot_descriptor> cloud1_bshot, cloud2_bshot;

    void calculate_normals ( float radius )
    {
        // Estimate the normals.
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
        // normalEstimation.setKSearch(radius);
        normalEstimation.setRadiusSearch(radius);
        normalEstimation.setNumberOfThreads(12);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        // normalEstimation.setSearchMethod(kdtree);
        pcl::search::KdTree<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud1.makeShared());

        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
       
        cloud1_normals.is_dense = true;
        cloud1_normals.points.resize(cloud1.size());

        #ifdef _OPENMP
        #pragma omp parallel for private (nn_indices, nn_dists) num_threads(12)
        #endif
        // Iterating over the entire index vector
        for (int idx = 0; idx < static_cast<int> (cloud1_keypoints.size ()); ++idx)
        {
            if (kdtree.radiusSearch(cloud1_keypoints[idx], radius, nn_indices, nn_dists, 300) == 0)
            {
                cloud1_normals.points[idx].normal[0] = cloud1_normals.points[idx].normal[1] = cloud1_normals.points[idx].normal[2] = cloud1_normals.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();
     
                cloud1_normals.is_dense = false;
                continue;
            }
     
            Eigen::Vector4f n;
            pcl::computePointNormal<PointXYZ> (cloud1, nn_indices, n,
                                               cloud1_normals.points[idx].curvature);
            cloud1_normals.points[idx].normal_x = n[0];
            cloud1_normals.points[idx].normal_y = n[1];
            cloud1_normals.points[idx].normal_z = n[2];
       
            pcl::flipNormalTowardsViewpoint (cloud1_keypoints[idx], 0, 0, 0,
                                             cloud1_normals.points[idx].normal[0], 
                                             cloud1_normals.points[idx].normal[1], 
                                             cloud1_normals.points[idx].normal[2]);
        }

        // normalEstimation.setInputCloud(cloud1.makeShared());
        // normalEstimation.compute(cloud1_normals);

        // normalEstimation.setInputCloud(cloud2.makeShared());
        // normalEstimation.compute(cloud2_normals);
    }


    void  calculate_voxel_grid_keypoints ( float leaf_size )
    {
        // Find Keypoints on the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

        voxel_grid.setInputCloud(cloud1.makeShared());
        voxel_grid.filter(cloud1_keypoints);

        // voxel_grid.setInputCloud(cloud2.makeShared());
        // voxel_grid.filter(cloud2_keypoints);


    }


    void calculate_SHOT ( float radius )
    {

        // SHOT estimation object.
        pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
        shot.setRadiusSearch(radius);

        shot.setNumberOfThreads(12);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        shot.setSearchMethod(kdtree);

        shot.setInputCloud(cloud1_keypoints.makeShared());
        shot.setSearchSurface(cloud1.makeShared());
        shot.setInputNormals(cloud1_normals.makeShared());
        shot.compute(cloud1_shot);

        // shot.setInputCloud(cloud2_keypoints.makeShared());
        // shot.setSearchSurface(cloud2.makeShared());
        // shot.setInputNormals(cloud2_normals.makeShared());
        // shot.compute(cloud2_shot);

    }


    void compute_bshot()
    {
        compute_bshot_from_SHOT( cloud1_shot, cloud1_bshot);
        // compute_bshot_from_SHOT( cloud2_shot, cloud2_bshot);
    }

    void compute_bshot_from_SHOT(pcl::PointCloud<pcl::SHOT352>& shot_descriptors_here, std::vector<bshot_descriptor>& bshot_descriptors)
    {
        bshot_descriptors.resize(shot_descriptors_here.size());
        for (int i = 0; i < (int)shot_descriptors_here.size(); i++)
        {
            std::bitset < 352 > temp;
            temp.reset();

            for (int j = 0 ; j < 88 ; j++)
            {
                float vec[4] = { 0 };
                for (int k = 0 ; k < 4 ; k++)
                {
                    vec[k] = shot_descriptors_here[i].descriptor[ j*4 + k ];

                }

                std::bitset< 4 > bit;
                bit.reset();

                float sum = vec[0]+vec[1]+vec[2]+vec[3];

                if (vec[0] == 0 and vec [1] == 0 and vec[2] == 0 and vec[3] == 0)
                {
                    //bin[0] = bin[1] = bin[2] = bin[3] = 0;
                    // by default , they are all ZEROS
                }
                else if ( vec[0] > (0.9 * (sum) ) )
                {
                    bit.set(0);
                }
                else if ( vec[1] > (0.9 * (sum) ) )
                {
                    bit.set(1);
                }
                else if ( vec[2] > (0.9 * (sum) ) )
                {
                    bit.set(2);
                }
                else if ( vec[3] > (0.9 * (sum) ) )
                {

                    bit.set(3);
                }
                else if ( (vec[0]+vec[1]) > (0.9 * (sum))  )
                {

                    bit.set(0);
                    bit.set(1);
                }
                else if ( (vec[1]+vec[2]) > (0.9 * (sum)) )
                {

                    bit.set(1);
                    bit.set(2);
                }

                else if ( (vec[2]+vec[3]) > (0.9 * (sum)) )
                {
                    ;
                    bit.set(2);
                    bit.set(3);
                }
                else if ( (vec[0]+vec[3]) > (0.9 * (sum)) )
                {

                    bit.set(0);
                    bit.set(3);
                }
                else if ( (vec[1]+vec[3]) > (0.9 * (sum)) )
                {

                    bit.set(1);
                    bit.set(3);
                }
                else if ( (vec[0]+vec[2]) > (0.9 * (sum)) )
                {

                    bit.set(0);
                    bit.set(2);
                }
                else if ( (vec[0]+ vec[1] +vec[2]) > (0.9 * (sum)) )
                {

                    bit.set(0);
                    bit.set(1);
                    bit.set(2);
                }
                else if ( (vec[1]+ vec[2] +vec[3]) > (0.9 * (sum)) )
                {

                    bit.set(1);
                    bit.set(2);
                    bit.set(3);
                }
                else if ( (vec[0]+ vec[2] +vec[3]) > (0.9 * (sum)) )
                {

                    bit.set(0);
                    bit.set(2);
                    bit.set(3);
                }
                else if ( (vec[0]+ vec[1] +vec[3]) > (0.9 * (sum)) )
                {

                    bit.set(0);
                    bit.set(1);
                    bit.set(3);
                }
                else
                {

                    bit.set(0);
                    bit.set(1);
                    bit.set(2);
                    bit.set(3);
                }

                if (bit.test(0))
                    temp.set(j*4);

                if (bit.test(1))
                    temp.set(j*4 + 1);

                if (bit.test(2))
                    temp.set(j*4 + 2);

                if (bit.test(3))
                    temp.set(j*4 + 3);

            }

            bshot_descriptors[i].bits = temp;
        }
    }


};

#endif

