/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(const std::vector<float>& point, int id)
	{
		insert(root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, target, ids, distanceTol, 0);
		return ids;
	}
	
private:
	void insert(Node*& node, const std::vector<float>& point, int id, int idx)
	{
		if (node == nullptr)
		{
			node = new Node(point, id);
		} 
		else if (point[idx] < node->point[idx])
		{
			insert(node->left, point, id, (idx + 1) % point.size());
		}
		else
		{
			insert(node->right, point, id, (idx + 1) % point.size());
		}
	}

	void search(Node* node, const std::vector<float>& target,
	                        std::vector<int>& ids, float distanceTol, int idx)
	{
		if (node == nullptr)
			return;

		bool check = true;
		for (size_t dim = 0; check && dim < target.size(); ++dim) {
			check = check &&
			        target[dim] + distanceTol >= node->point[dim] &&
					target[dim] - distanceTol <= node->point[dim];
		}

		if (check && dist(target, node->point) < distanceTol)
		{
			ids.push_back(node->id);
		}
		
		const int new_idx = (idx + 1) % target.size();
		if (target[idx] - distanceTol < node->point[idx])
		{
			search(node->left, target, ids, distanceTol, new_idx);
		}
		
		if (target[idx] + distanceTol > node->point[idx])
		{
			search(node->right, target, ids, distanceTol, new_idx);
		}
	}

	float dist(const std::vector<float>& p1, const std::vector<float>& p2) {
		float res = 0.0f;
		for (int i = 0; i < p1.size(); ++i)
		{
			res += (p1[i] - p2[i]) * (p1[i] - p2[i]);
		}

		return std::sqrt(res);
	}
};




