/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT, size_t dim = 3>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void build(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		const size_t n = cloud->points.size();
		std::vector<int> indices(n, 0);
    	std::iota(indices.begin(), indices.end(), 0);
		build(root, cloud, indices, 0, n, 0);
	}

	void insert(const PointT& point, int id)
	{
		insert(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const PointT& target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, target, ids, distanceTol, 0);
		return ids;
	}
	
private:

	int rand_median(typename pcl::PointCloud<PointT>::Ptr cloud, int coord,  std::vector<int>& indices, int st, int end, int i) {
		if (st == end - 1) {
			return st;
		}

		int x = cloud->points[indices[end-1]].data[coord];
		int pos = st;
		for (int i = st; i < end-1; ++i) {
			if (cloud->points[indices[i]].data[coord] <= x) {
				std::swap(indices[i], indices[pos]);
				++pos;
			}
		}
		
		std::swap(indices[end-1], indices[pos]);
		int k = pos - st;
		
		if (i == k) {
			return pos;
		} else if (i < k) {
			return rand_median(cloud, coord, indices, st, pos, i);
		} else {
			return rand_median(cloud, coord, indices, pos + 1, end, i - k - 1);
		}
	}

	void build(Node<PointT>*& node, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& indices, int st, int end, int depth)
	{
		if (st == end) {
			return;
		}

		const int median = rand_median(cloud, depth % dim, indices, st, end, (end - st) / 2);
		const int id = indices[median];
		node = new Node<PointT>(cloud->points[id], id);

		build(node->left, cloud, indices, st, median, depth + 1);
		build(node->right, cloud, indices, median + 1, end, depth + 1);
	}

	void insert(Node<PointT>*& node, const PointT& point, int id, int depth)
	{
		const int idx = depth % dim;
		if (node == nullptr)
		{
			node = new Node<PointT>(point, id);
		} 
		else if (point.data[idx] < node->point.data[idx])
		{
			insert(node->left, point, id, depth + 1);
		}
		else
		{
			insert(node->right, point, id, depth + 1);
		}
	}

	void search(Node<PointT>* node, const PointT& target, std::vector<int>& ids, float distanceTol, int depth)
	{
		if (node == nullptr)
			return;

		bool check = true;
		for (size_t i = 0; check && i < dim; ++i) {
			check = check &&
			        target.data[i] + distanceTol >= node->point.data[i] &&
					target.data[i] - distanceTol <= node->point.data[i];
		}

		if (check && dist(target, node->point) <= distanceTol)
		{
			ids.push_back(node->id);
		}
		
		const int idx = depth % dim;
		if (target.data[idx] - distanceTol < node->point.data[idx])
		{
			search(node->left, target, ids, distanceTol, depth + 1);
		}
		
		if (target.data[idx] + distanceTol > node->point.data[idx])
		{
			search(node->right, target, ids, distanceTol, depth + 1);
		}
	}

	float dist(const PointT& p1, const PointT& p2) {
		float res = 0.0f;
		for (int i = 0; i < dim; ++i)
		{
			res += (p1.data[i] - p2.data[i]) * (p1.data[i] - p2.data[i]);
		}

		return std::sqrt(res);
	}
};




