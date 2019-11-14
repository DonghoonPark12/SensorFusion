/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

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

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id){
		if(*node == NULL){
			*node = new Node(point, id);//awesome grammer.
		}
		else{
			uint flag = depth % 3; //2 //whether to compare x or y.
			//cout<<"cd: "<<cd<<endl;
			if( point[flag] < ((*node)->point[flag]) )
				insertHelper( &((*node)->left), depth+1, point, id );
			else
				insertHelper( &((*node)->right), depth+1, point, id );
		}
	}

	void insert(std::vector<float> point, int id) 
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		insertHelper(&root, 0, point, id);

		/* What if I don't use recustion */
		/* [TODO] Make it works!!!!
		Node *inserted_node = new Node(point, id);
		Node *cur = root;
		if(root == NULL){
			root = inserted_node;
			cur = root;
		}
		else{
			int flag = 0;
			while(cur != NULL){
				cout<<flag<<endl; //Why in hear flag become 0 again??
				if( (cur->point[flag]) < point[flag]){
					cur = cur->right;
					flag = (flag+1)%2;
					cout<<flag<<endl;
					cout<<"Right!"<<endl;
				}
				else{
					cur = cur->left;
					flag = (flag+1)%2;
					cout<<flag<<endl;
					cout<<"Left!"<<endl;
				}
			}
			cur = inserted_node;
		}
		*/
		
	}

	//[TODO] insert_pcl, insertHelper_pcl, search_pcl, seachHelper_pcl
	// void insert_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr, int id){
	// 	insertHelper(&root, 0, point, id);
	// }

	void searchHelper3D(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids){
		if(node != NULL){
			if( (node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol)) 
			 && (node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol))
			 && (node->point[2] >= (target[2] - distanceTol)) && (node->point[2] <= (target[2] + distanceTol))){
				float distance = sqrt(pow(node->point[0]-target[0],2)
								    + pow(node->point[1]-target[1],2)
									+ pow(node->point[2]-target[2],2));
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}
			
			//check accross boundary
			if( (target[depth%3]-distanceTol) < node->point[depth%3] )
				searchHelper3D(target, node->left, depth+1, distanceTol, ids);
			if( (target[depth%3]+distanceTol) > node->point[depth%3] )
				searchHelper3D(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper3D(target, root, 0, distanceTol, ids);
		return ids;
	}
};

#endif



