/* \author Aaron Brown */
// Quiz on implementing kd tree
#pragma once
#include "../../render/render.h"
#include <math.h> 

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

	void insertHelper(Node *&node, std::vector<float> point, int id, int level)
	{

		bool isXSplit = level % 2 == 0;
		// std::cout << point[0] << ", " << point[1] << " level: " << level << " isXsplit: " << isXSplit <<std::endl;
		level++;
		if(node == NULL)
		{
			node = new Node(point, id);
		}
		else if((isXSplit && point[0] < node->point[0]) || (!isXSplit && point[1] < node->point[1]))
		{
			insertHelper(node->left, point, id, level);
		}
		else 
		{
			insertHelper(node->right, point, id, level);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if (*&root == NULL)
		{
			root = new Node(point, id);
		}
		else
		{	
			insertHelper(root, point, id, 0);
		}
		

	}

	void searchHelper(Node *&node, std::vector<int> &ids, uint level, std::vector<float> target, float distanceTol)
	{
		if (node == NULL) return;
		// std::cout << node->point[0] << ", " << node->point[1] << " level: " << level << " id: " << node->id << std::endl;
		// float tolFactor = fabs(target[0] * distanceTol);
		float tolFactor = distanceTol;
		float left_x = target[0] - tolFactor;
		float right_x = target[0] + tolFactor;
		float top_y = target[1] + tolFactor;
		float bottom_y = target[1] - tolFactor;

		uint l = level % 2;
		// check node is inside box
		// std::cout << left_x << ", " << right_x << ", " << bottom_y << ", " << top_y << std::endl;
		if (node->point[0] >= left_x && node->point[0] <= right_x && node->point[1] <= top_y && node->point[1] >= bottom_y)
		{
			// yes, check distance to target
			float a = node->point[0] - target[0];
			float b = node->point[1] - target[1];
			int d = sqrt(a*a + b*b);
			// std::cout << " distance: " << d << ", " << a << ", " << b << std::endl;
			if (d <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		if ((target[l] - tolFactor) < node->point[l])
		{
			searchHelper(node->left, ids, level+1, target, distanceTol);
		}

		if ((target[l] + tolFactor) > node->point[l])
		{
			searchHelper(node->right, ids, level+1, target, distanceTol);
		}
		
		
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, ids, 0, target, distanceTol);
		return ids;
	}
	

};




