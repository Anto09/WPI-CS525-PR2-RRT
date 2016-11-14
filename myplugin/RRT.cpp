#include "RRT.h"
#define MAX_ITERATIONS = 10000
#define DELTA = 0.01

//RRTNode non-constructor and non-get methods
float RRTNode::Distance(RRTNode* node)
{
	float sq_dist = 0.0f;
	for (int i = 0; i < this->_config.size(); i++)
	{
		sq_dist += (this->_config[i] - node->_config[i]) * (this->_config[i] - node->_config[i]);
	}
	return sq_dist;
}

RRTNode* RRTNode::NearestNode(NodeTree* tree, bool kdtree = false)
{
	if (kdtree)
	{

	}
	else
	{
		float min_dist = 999999.0f;
		RRTNode* returnNode = NULL;
		vector<RRTNode*>nodes = tree->GetNodes();
		for (int i = 0; i < nodes.size(); i++)
		{
			RRTNode* curNode = (nodes[i]);
			float dist = this->Distance(curNode);
			if (dist < min_dist)
			{
				min_dist = dist;
				returnNode = curNode;
			}
		}
	}
}


//NodeTree non-constructor and non-get methods
void NodeTree::AddNode(RRTNode* node)
{
	this->_size++;
	this->_nodes.resize(this->_size);
	this->_nodes[this->_size-1] = node;
	node->SetIndex(this->_size - 1);
}

void NodeTree::DeleteNode(RRTNode* node)
{
	this->_nodes.resize(remove(this->_nodes.begin(), this->_nodes.end(), node) - this->_nodes.begin());
}

void NodeTree::AddEdge(RRTNode* a, RRTNode* b)
{
	b->SetParent(a);
}

//RRT Fields
RobotBasePtr _probot;
RobotBase::ManipulatorPtr _pmanip;
Transform _tmanipprev, _tmanipmidreal;
KinBody::LinkPtr _ppolelink;
KinBody::JointPtr _ppolejoint;
EnvironmentBasePtr penv;
CollisionCheckerBasePtr pchecker; 

std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(0.0,1.0);

//RRT Methods
RRTNode* Sample(int sampleCount) //sampleCount is for goal-biasing
{
	RRTNode* ret = new RRTNode();

	for (int i = 0; i < 7; i++)
	{
		float number = distribution(generator);
		ret->SetConfigElement(i, number);
	}
	return ret;
}

bool CheckCollision(q)
{
	//check collision with env

	// save state before modifying it
	RobotBase::RobotStateSaverPtr savestate1(new RobotBase::RobotStateSaver(_probot));
	_probot.SetActiveDOFValues(q->GetConfig());

	return true;
}

bool NewConfig(RRTNode* q, RRTNode* qnear, RRTNode* qnew)
{
	if (!CheckCollision(q))
	{
		vector<float> unew;
	}
}

int Extend(NodeTree* tree, RRTNode* q, RRTNode* qnew) //2 = reached, 1 = advanced, 0 = trapped
{
	qnear = q->NearestNode(tree);
	//RRTNode* qnew = NULL;
	if (NewConfig(q, qnear, qnew))
	{
		tree->AddNode(qnew);
		tree->AddEdge(qnear, qnew);
		if (qnew == q)
			return 2;
		else
			return 1;
	}
	return 0;
}

int Connect(NodeTree* tree, RRTNode* q)
{
	int s = 0;
	do
	{
		s = Extend(tree, q);
	}
	while(s != 1);
	return s;
}

void Swap(NodeTree* a, NodeTree* b)
{
	NodeTree* temp = a;
	a = b;
	b = a;
}

NodeTree* RRT_Connect(RRTNode* start, RRTNode* goal)
{
	vector<RRTNode*> nodes;
	NodeTree* tree_a = new NodeTree(start, goal, nodes);
	NodeTree* tree_b = new NodeTree(goal, start, nodes);

	_probot->SetActiveDOFs(_pmanip->GetArmIndices());
	
	int K = MAX_ITERATIONS;
	for (int k = 0; k < K; k++)
	{

		RRTNode* qrand = Sample(k);
		RRTNode* qnew = NULL;
		if (!Extend(tree_a, qrand, qnew) == 0)
			if(Connect(tree_b,qnew) == 2)
				return Path(tree_a, tree_b);
		Swap(tree_a, tree_b);
		
	}
	return tree;
}

int main()
{
	string scenefilename = "../hw3.env.xml";
	penv->Load(scenefilename);
	_probot = pmanip->GetRobot();
	CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker(penv,argv[i+1]);
	return 0;
}