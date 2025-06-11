

#include "ransac3d.h"


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{
		std::unordered_set<int>inliers;
		while(inliers.size()<3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        ++itr;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z; 

        Eigen::Vector3d v1(x2-x1,y2-y1,z2-z1);
        Eigen::Vector3d v2(x3-x1,y3-y1,z3-z1);
        Eigen::Vector3d cross = v1.cross(v2);
    
    

		float a,b,c,d;
		a = cross(0);
		b = cross(1);
		c = cross(2);
        d = -(cross(0)*x1 + cross(1)*y1 + cross(2)*z1);

		for(int index = 0;index < cloud->points.size();++index)
		{
			if(inliers.count(index)>0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];
			float x = point.x;
			float y = point.y;
            float z = point.z;

			float distance = fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
			if(distance <= distanceTol)
				inliers.insert(index);
		}
		if(inliers.size()>inliersResult.size())
			inliersResult = inliers;
	}
	
	return inliersResult;

}