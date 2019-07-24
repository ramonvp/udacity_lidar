/* \author Ramon Viedma */
// Quiz on implementing kd tree

#include <vector>
#include <cmath>

// Structure to represent node of kd tree

template<typename PointT>
struct KdTree
{
	struct Node
	{
		const PointT * point;
		int id;
		Node* left;
		Node* right;

		Node(const PointT * p, int setId)
		:	point(p), id(setId), left(NULL), right(NULL)
		{}

		~Node()
		{
			delete left;
			delete right;
		}
	};

	Node* root;
	const uint dimensions;

	KdTree(uint dim)
	: root(NULL), dimensions(dim)
	{}

	~KdTree()
	{
		delete root;
	}


	void insert(const PointT * point, int id)
	{
		Node ** next_node = &root;
		uint depth = 0;

		while( *next_node )
		{
			if ( point->data[depth%dimensions] < (*next_node)->point->data[depth%dimensions] )
			{
				next_node = &(*next_node)->left;
			}
			else
			{
				next_node = &(*next_node)->right;
			}
			depth++;
		}

		*next_node = new Node(point, id);
	}

	void searchFromNode(std::vector<int> & ids, const PointT * target, float distanceTol, Node * node, uint depth)
	{
		if( node == NULL || target == NULL) return;

		float box_x1 = target->data[0] - distanceTol;
		float box_x2 = target->data[0] + distanceTol;
		float box_y1 = target->data[1] - distanceTol;
		float box_y2 = target->data[1] + distanceTol;

		float point_x = node->point->data[0];
		float point_y = node->point->data[1];

		// point is inside the tolerance box
		if( point_x >= box_x1 && point_x <= box_x2 && point_y >= box_y1 && point_y <= box_y2)
		{
			// check if it is in real tolerance
			float distance = sqrt(pow(target->data[0]-point_x, 2) + pow(target->data[1]-point_y,2));
			if( distance <= distanceTol )
			{
				ids.push_back(node->id);
			}
		}
		
		uint dim = depth%dimensions;

		if(target->data[dim] - distanceTol < node->point->data[dim])
		{
			searchFromNode(ids, target, distanceTol, node->left, depth+1);
		}

		if(target->data[dim] + distanceTol > node->point->data[dim])
		{
			searchFromNode(ids, target, distanceTol, node->right, depth+1);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const PointT * target, float distanceTol)
	{
		std::vector<int> ids;

		assert(root != nullptr);
		assert(dimensions > 0);

		searchFromNode(ids, target, distanceTol, root, 0);

		return ids;
	}
	

};
