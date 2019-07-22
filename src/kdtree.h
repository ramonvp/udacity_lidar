/* \author Ramon Viedma */
// Quiz on implementing kd tree

#include <vector>
#include <cmath>

#if 0
// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(const PointT & arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	typedef boost::shared_ptr<Node<PointT> > Ptr;
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insert(const PointT & point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		Node<PointT> ** next_node = &root;
		uint depth = 0;
		uint dimensions = point.size();

		while( *next_node )
		{
			if ( point[depth%dimensions] < (*next_node)->point[depth%dimensions] )
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

	void searchFromNode(std::vector<int> & ids, std::vector<float> target, float distanceTol, Node * node, uint depth)
	{
		if( node == NULL ) return;

		float box_x1 = target[0] - distanceTol;
		float box_x2 = target[0] + distanceTol;
		float box_y1 = target[1] - distanceTol;
		float box_y2 = target[1] + distanceTol;

		float point_x = node->point[0];
		float point_y = node->point[1];

		// point is inside the tolerance box
		if( point_x >= box_x1 && point_x <= box_x2 && point_y >= box_y1 && point_y <= box_y2)
		{
			// check if it is in real tolerance
			float distance = sqrt(pow(target[0]-point_x, 2) + pow(target[1]-point_y,2));
			if( distance <= distanceTol )
			{
				ids.push_back(node->id);
			}
		}

		uint dim = depth%2;

		if(target[dim] - distanceTol < node->point[dim])
		{
			searchFromNode(ids, target, distanceTol, node->left, depth+1);
		}

		if(target[dim] + distanceTol > node->point[dim])
		{
			searchFromNode(ids, target, distanceTol, node->right, depth+1);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchFromNode(ids, target, distanceTol, root, 0);

		return ids;
	}
	

};
#endif


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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;
	uint dimensions;

	KdTree()
	: root(NULL), dimensions(0)
	{}

	~KdTree()
	{
		delete root;
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		Node ** next_node = &root;
		uint depth = 0;

		if(dimensions == 0)
			dimensions = point.size();

		assert(dimensions == point.size());

		while( *next_node )
		{
			if ( point[depth%dimensions] < (*next_node)->point[depth%dimensions] )
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

	void searchFromNode(std::vector<int> & ids, std::vector<float> target, float distanceTol, Node * node, uint depth)
	{
		if( node == NULL ) return;

		float box_x1 = target[0] - distanceTol;
		float box_x2 = target[0] + distanceTol;
		float box_y1 = target[1] - distanceTol;
		float box_y2 = target[1] + distanceTol;

		float point_x = node->point[0];
		float point_y = node->point[1];

		// point is inside the tolerance box
		if( point_x >= box_x1 && point_x <= box_x2 && point_y >= box_y1 && point_y <= box_y2)
		{
			// check if it is in real tolerance
			float distance = sqrt(pow(target[0]-point_x, 2) + pow(target[1]-point_y,2));
			if( distance <= distanceTol )
			{
				ids.push_back(node->id);
			}
		}
		
		//uint dim = depth%2;
		uint dim = depth%dimensions;

		if(target[dim] - distanceTol < node->point[dim])
		{
			searchFromNode(ids, target, distanceTol, node->left, depth+1);
		}

		if(target[dim] + distanceTol > node->point[dim])
		{
			searchFromNode(ids, target, distanceTol, node->right, depth+1);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		assert(root != nullptr);
		assert(dimensions > 0);

		searchFromNode(ids, target, distanceTol, root, 0);

		return ids;
	}
	

};




