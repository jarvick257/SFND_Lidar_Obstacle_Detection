#pragma once
#include "kdtree.h"

namespace project
{

template<typename PointT>
class EuclideanClusterExtraction
{
  public:
    EuclideanClusterExtraction<PointT>() = default;
    ~EuclideanClusterExtraction<PointT>() = default;


    void setClusterTolerance(float f_tol) {
        m_clusterTolerance = f_tol;
    }
    void setMinClusterSize(int f_minSize){
        m_minSize = f_minSize;
    }
    void setMaxClusterSize(int f_maxSize){
        m_maxSize = f_maxSize;
    }
    void setSearchMethod(std::shared_ptr<project::KdTree<PointT>> f_tree_p){
        m_tree_p = f_tree_p;
    }
    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr& f_cloud_p){
        m_cloud_p = f_cloud_p;
    }

    void extract(std::vector<pcl::PointIndices>& f_clusterIndices){
        std::vector<bool> processed(m_cloud_p->points.size(), false);

        int i = 0;
        for(const auto& point : m_cloud_p->points){
            m_tree_p->insert(point, i++);
        }

        for (int i = 0; i < processed.size(); ++i) {
            if (processed[i]) {
                continue;
            }

            pcl::PointIndices cluster;
            this->clusterHelper(i, cluster, processed);
            f_clusterIndices.push_back(cluster);
        }

    }

  private:
    float m_clusterTolerance;
    int m_minSize;
    int m_maxSize;
    std::shared_ptr<project::KdTree<PointT>> m_tree_p;
    typename pcl::PointCloud<PointT>::Ptr m_cloud_p;

    void clusterHelper(int indice, pcl::PointIndices &cluster, std::vector<bool> &processed){
        processed[indice] = true;
        cluster.indices.push_back(indice);
        std::vector<int> nearest = m_tree_p->search(m_cloud_p->points[indice], m_clusterTolerance);
        for (int id : nearest) {
            if (!processed[id]) {
                clusterHelper(id, cluster, processed);
            }
        }
    }
};

}

