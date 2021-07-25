#pragma once
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <unordered_set>

namespace project
{
    template<typename PointT>
    class SACSegmentation{
      public:
        SACSegmentation() = default;
        ~SACSegmentation() = default;

        void setOptimizeCoefficients(bool f_optimize){/*noop*/};
        void setModelType(int f_model){/*noop*/};
        void setMethodType(int f_method){/*noop*/};
        void setMaxIterations(int f_iterations) {
            m_iterations = f_iterations;
        }
        void setDistanceThreshold(float f_thresh) {
            m_distanceThreshold = f_thresh;
        };
        void setInputCloud(typename pcl::PointCloud<PointT>::Ptr f_cloud_p){
            m_inputCloud_p = f_cloud_p;
        };

        void segment(pcl::PointIndices& f_inliers, pcl::ModelCoefficients& coefficients){
            std::unordered_set<int> inliersResult;
            srand(time(NULL));

            // TODO: Fill in this function

            // For max iterations
            while (m_iterations--) {
                std::unordered_set<int> inliers;

                // Randomly sample subset and fit line
                while (inliers.size() < 3) {
                    inliers.insert(rand() % m_inputCloud_p->points.size());
                }

                auto it = inliers.begin();
                const PointT p1 = m_inputCloud_p->points[*it++];
                const PointT p2 = m_inputCloud_p->points[*it++];
                const PointT p3 = m_inputCloud_p->points[*it];
                const PointT v1 = PointT(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
                const PointT v2 = PointT(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

                const PointT norm = PointT(
                        v1.y * v2.z - v1.z * v2.y,
                        v1.z * v2.x - v1.x * v2.z,
                        v1.x * v2.y - v1.y * v2.x);

                const float A = norm.x;
                const float B = norm.y;
                const float C = norm.z;
                const float D = -(A*p1.x + B*p1.y, + C*p1.z);

                const float denom = sqrt(A*A + B*B + C*C);

                for (int index = 0; index < m_inputCloud_p->points.size(); index++) {
                    if (inliers.count(index) > 0) {
                        continue;
                    }

                    const PointT p = m_inputCloud_p->points[index];


                    const float nom = fabs(A*p.x + B*p.y + C*p.z + D);
                    const float d =  nom / denom;
                    if (d <= m_distanceThreshold) {
                        inliers.insert(index);
                    }
                }

                if (inliers.size() > inliersResult.size()) {
                    inliersResult = inliers;
                }

                // Measure distance between every point and fitted line
                // If distance is smaller than threshold count it as inlier
            }

            for (const auto& i: inliersResult){
                f_inliers.indices.push_back(i);
            }
        }

      private:
        int m_iterations;
        float m_distanceThreshold;
        typename pcl::PointCloud<PointT>::Ptr m_inputCloud_p;
    };
}
