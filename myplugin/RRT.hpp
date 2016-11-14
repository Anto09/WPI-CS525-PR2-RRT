#ifndef RRT_H
#define RRT_H

#include <cfloat>
#include <vector>
#include <algorithm>
#include <cmath>
#include <openrave/plugin.h>
#include <openrave/planningutils.h>
#include <boost/bind.hpp>
#include <random>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <ctime>

#define DOF = 7

using namespace OpenRAVE;
using namespace std;

class RRTNode;
class NodeTree;

class RRTNode
{
	private:
		int _parent;
		// RRTNode _parent;
		int _index; //index in the vector
		vector<double> _config;

	public:
		//constructors
		RRTNode() : _index(-1) {}
		RRTNode(vector<double> config) : _index(-1), _config(config) {}
		//RRTNode(RRTNode p, vector<double> config) : _parent(p), _index(-1), _config(config) {}
		RRTNode(int p, vector<double> config) : _parent(p), _index(-1), _config(config) {}
		//Get functions
		//inline RRTNode GetParent(){ return this->_parent; };
		inline int GetParent(){ return this->_parent; };
		inline vector<double> GetConfig(){ return this->_config; }
		inline const vector<double>& GetConfig() const { return _config; }
		inline vector<double>& GetMutableConfig() { return _config; }
		inline int GetIndex(){ return this->_index; }
		
		//Set functions
		inline void SetIndex(int index){ this->_index = index; }
		//inline void SetParent(RRTNode parent){ this->_parent = parent; }
		inline void SetParent(int parent){ this->_parent = parent; }
		inline void SetConfig(vector<double> config){ this->_config = config; }
		inline void SetConfigElement(uint index, double angle)
		{ 
			if (index < this->_config.size())
				this->_config[index] = angle; 
			else
				this->_config.push_back(angle);
		}
		
		//functions to be defined outside header
		RRTNode NearestNode(NodeTree tree, std::vector<double> weights, bool kdtree);
		double Distance(const RRTNode& node, OpenRAVE::RobotBasePtr probot);
		double ModDistance(const RRTNode& node, OpenRAVE::RobotBasePtr probot, std::vector<double> weights);
};

class NodeTree
{
	private:
		vector<RRTNode> _nodes;
		double _minDist;
		double _size;
		RRTNode _start;
		RRTNode _goal;
		OpenRAVE::RobotBasePtr _probot;
	public:
		//constructors
		NodeTree() : _minDist(0), _size(0) {}
		NodeTree(RRTNode start, RRTNode goal, std::vector<RRTNode> nodes, OpenRAVE::RobotBasePtr probot)
		{
			this->_minDist = 0;
			this->_size = 0;
			
			this->_start = start;
			this->_goal = goal;
			
			this->_nodes = nodes;
			this->_nodes.push_back(start);
			this->_size++;

			this->_probot = probot;
		}

		//Get functions
		inline double GetMinDist(){ return this->_minDist; }
		inline RRTNode GetNode(int index)
		{ 
			RRTNode ret;
			if (index < this->_size)
				ret = this->_nodes[index];
			return ret;
		}
		inline vector<RRTNode> GetNodes(){ return this->_nodes; }
		inline RobotBasePtr GetRobot(){ return _probot; }
		inline int GetSize(){ return this->_size; }

		//Set functions
		inline void SetNode(std::vector<RRTNode> nodes){ this->_nodes = nodes; }
		inline void SetStart(RRTNode start){ this->_start = start; }
		inline void SetGoal(RRTNode goal){ this->_goal = goal; }
		inline void SetRobot(OpenRAVE::RobotBasePtr probot){ this->_probot = probot; }

		//functions to be defined outside header
		void AddNode(RRTNode node);
		void DeleteNode(RRTNode node);
		void AddEdge(RRTNode a, RRTNode b);
};

#endif