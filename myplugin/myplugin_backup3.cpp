#include "RRT.hpp"

#define MAX_TIME 180.00
#define DELTA 0.05
#define GOAL_SAMPLE 0.1
#define GOAL_ERROR 0.05

//RRTNode non-constructor and non-get methods
double RRTNode::Distance(const RRTNode& node, OpenRAVE::RobotBasePtr probot)
{
    vector<double>config = this->GetConfig();
    probot->SubtractActiveDOFValues(config, node.GetConfig());
    double dist = 0;
    for (double& diff : config)
    {
        dist += (diff * diff);
    }
    return sqrt(dist);
}

double RRTNode::ModDistance(const RRTNode& node, OpenRAVE::RobotBasePtr probot, std::vector<double> weights)
{
    vector<double>config = this->GetConfig();
    probot->SubtractActiveDOFValues(config, node.GetConfig());
    double dist = 0.0;
    int count = 0;
    for (double& diff : config)
    {
        dist += (diff * diff * weights[count]);
        count++;
    }
    return sqrt(dist);
}

RRTNode RRTNode::NearestNode(NodeTree tree, std::vector<double>weights, bool kdtree = false)
{
    RRTNode returnNode;
    if (kdtree)
    {

    }
    else
    {
        double min_dist = DBL_MAX;
        vector<RRTNode>nodes = tree.GetNodes();
        for (int i = 0; i < nodes.size(); i++)
        {
            RRTNode curNode = (nodes[i]);
            double dist = this->ModDistance(curNode, tree.GetRobot(), weights);
            if (dist < min_dist)
            {
                min_dist = dist;
                returnNode = curNode;
            }
        }
    }
    return returnNode;
}

//NodeTree non-constructor and non-get methods
void NodeTree::AddNode(RRTNode node)
{
    this->_size++;
    this->_nodes.resize(this->_size);
    this->_nodes[this->_size-1] = node;
    node.SetIndex(this->_size - 1);
}

void NodeTree::DeleteNode(RRTNode node)
{
    int index = node.GetIndex();
    this->_nodes.erase(this->_nodes.begin()+index);
    for (int i = index; i < this->_nodes.size(); i++)
        this->_nodes[index].SetIndex(this->_nodes[index].GetIndex()-1);
}

void NodeTree::AddEdge(RRTNode a, RRTNode b)
{
    //b.SetParent(a);
    b.SetParent(a.GetIndex());
}

//Plugin classes and functions
class RRT : public ModuleBase
{

private:
    //RRT Fields
    OpenRAVE::RobotBasePtr _probot;
    OpenRAVE::EnvironmentBasePtr _penv;
    OpenRAVE::CollisionCheckerBasePtr pchecker; 
    std::vector<double>_startConfig;
    std::vector<double>_goalConfig;
    std::vector<double>_jointWeights;
    RRTNode _startNode;
    RRTNode _goalNode;

    int bidirect;

public:
    RRT(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&RRT::MyCommand,this,_1,_2),
                        "This is an example command"),
        RegisterCommand("Start_RRT",boost::bind(&RRT::Start_RRT,this,_1,_2),
                        "Command to start RRT");
        RegisterCommand("Set_Bidirectional",boost::bind(&RRT::Set_Bidirectional,this,_1,_2),
                        "Command to perform bidirectional RRT");
        RegisterCommand("Set_Joint_Weights",boost::bind(&RRT::Set_Joint_Weights,this,_1,_2),
                        "Command to set joint weights");
        this->_penv = penv;
    }
    virtual ~RRT() {}
    
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        while (sinput.good())
        {
            std::string input;
            sinput >> input;
            sout << "output";

            cout << input << endl;
        }
        _probot = _penv->GetRobot("PR2");

        return true;
    }

    bool Set_Joint_Weights(std::ostream& sout, std::istream& sinput)
    {
        while (sinput.good())
        {
            std::string input;
            sinput >> input;
            this->_jointWeights.push_back(stod(input));

            cout << _jointWeights[_jointWeights.size()-1] << endl;
        }
        return true;
    }

    bool Set_Bidirectional(std::ostream& sout, std::istream& sinput)
    {
        int count = 0;
        while (sinput.good() && count < 1)
        {
            std::string input;
            sinput >> input;
            bidirect = stoi(input);
            count++;
        }
    }

    bool Start_RRT(std::ostream& sout, std::istream& sinput)
    {
        while (sinput.good())
        {
            std::string input;
            sinput >> input;
            this->_goalConfig.push_back(stod(input));
        }
        EnvironmentMutex::scoped_lock lock(_penv->GetMutex());

        this->_probot = _penv->GetRobot("PR2");
        this->_probot->GetActiveDOFValues(this->_startConfig);

        this->_startNode.SetConfig(this->_startConfig);
        this->_goalNode.SetConfig(this->_goalConfig);

        std::vector<RRTNode> path = RRT_Connect(this->_startNode, this->_goalNode, this->_probot);

        this->_probot->SetActiveDOFValues(this->_startConfig);
        OpenRAVE::TrajectoryBasePtr traj = OpenRAVE::RaveCreateTrajectory(GetEnv(), "");
        traj->Init(_probot->GetActiveConfigurationSpecification());
        for (RRTNode& node : path)
        {
             traj->Insert(traj->GetNumWaypoints(), node.GetConfig());
        }
        if (path.size() > 0)
        {
            cout << "Executing Trajectory" << endl;

            OpenRAVE::planningutils::RetimeActiveDOFTrajectory(traj, _probot, false, 0.2, 0.2, "LinearTrajectoryRetimer");
            _probot->GetController()->SetPath(traj);
            return true;
        }
        else
        {
            cout << "No Path Found" << endl;
        }
        return true;
    }
    /*
        Method for getting a random sample. 
        Returns the random sample node
    */
    RRTNode Sample(int gaussian = 1) //sampleCount is for goal-biasing
    {
        std::random_device rd;
        std::mt19937_64 gen(rd());
        std::uniform_real_distribution<double> g_dist(0.0, std::nextafter(1.0, DBL_MAX));
        double g_prob = g_dist(gen);

        if (g_prob <= GOAL_SAMPLE)
            return _goalNode;

        RRTNode ret;
        std::vector<int> indices = _probot->GetActiveDOFIndices();
        std::vector<double> u_limits;
        std::vector<double> l_limits;

        _probot->GetActiveDOFLimits(l_limits, u_limits);

        int curr = 0;
        for (int& index : indices)
        {
            std::random_device rd;
            std::mt19937_64 gen(rd());
            std::uniform_real_distribution<double> distribution(l_limits[curr], std::nextafter(u_limits[curr], DBL_MAX));
            double number = distribution(gen);

            if (abs(l_limits[curr]) > 2*M_PI || u_limits[curr] > 2*M_PI)
                number = atan2(sin(number), cos(number));

            ret.SetConfigElement(curr, number);
            curr++;
        }
        return ret; 
    }

    bool CheckCollision(RRTNode q)
    {
        _probot->SetActiveDOFValues(q.GetConfig());
        return GetEnv()->CheckCollision(_probot) || _probot->CheckSelfCollision();
    }

    bool CompareConfig(RRTNode a, const RRTNode& b)
    {
        return a.Distance(b, _probot) < GOAL_ERROR;
    }

    void Swap(NodeTree* a, NodeTree* b)
    {
        NodeTree* temp = a;
        a = b;
        b = temp;
    }

    std::vector<RRTNode> Path(NodeTree a, RRTNode q)
    {
        std::vector<RRTNode> path;
        path.push_back(q);

        while (q.GetParent() > -1)
        {
            q = a.GetNodes()[q.GetParent()];
            path.push_back(q);  
        }
        path.push_back(_startNode);
        std::reverse(path.begin(), path.end());
        return path;
    }

    NodeTree Path(const NodeTree& a, const NodeTree& b, RRTNode q)
    {
        NodeTree returnTree;
        return returnTree;
    }

    RRTNode NewConfig(RRTNode& q, RRTNode& qnear, NodeTree& tree)
    {
        double distance = qnear.Distance(q, tree.GetRobot());
        std::vector<double> diff = q.GetConfig();
        _probot->SubtractActiveDOFValues(diff, qnear.GetConfig()); //C-space vector from qnear = q

        vector<double> init = qnear.GetConfig();

        double sum = 0.0;
        for (int i = 0; i < init.size(); i++)
        {  //new configuration is qnear config + stepsize of DELTA in respective directions
            double elem = diff[i];
            if (DELTA < distance)
                init[i] += (elem/distance) * DELTA;
            else
                init[i] += (elem/distance) * distance;
            sum += elem/distance;
        }

        RRTNode qnew(qnear.GetIndex(), init);
        return qnew;
    }

    int Extend(NodeTree& tree, RRTNode& q) //2 = reached, 1 = advanced, 0 = trapped
    {
        RRTNode qnear = q.NearestNode(tree, _jointWeights);
        std::vector<double>diff = q.GetConfig();
        _probot->SubtractActiveDOFValues(diff, qnear.GetConfig());
        RRTNode qnew = NewConfig(q, qnear, tree);

        if (!CheckCollision(qnew))
        {
            qnew.SetIndex(tree.GetNodes().size());
            tree.AddNode(qnew);
            tree.AddEdge(qnear, qnew);

            if (CompareConfig(q, qnew))
                return 2;
            else
            {
                return 1;
            }
        }
        return 0;
    }

    int Bidirect_Connect(NodeTree& tree, RRTNode& q)
    {
        int s = 0;
        do
        {
            s = Extend(tree, q);
        }
        while(s == 1);
        return s;
    }

    int Connect(NodeTree& tree, RRTNode& q)
    {
        RRTNode qnear = q.NearestNode(tree, _jointWeights);
        double dist = qnear.Distance(q, _probot);
        int max_steps = ceil(dist/DELTA);

        RRTNode qprev = qnear;
        for (int i = 0; i < max_steps; i++)
        {
            double distance = qprev.Distance(q, tree.GetRobot());
            std::vector<double> diff = q.GetConfig();
            _probot->SubtractActiveDOFValues(diff, qprev.GetConfig());
            vector<double> init = qprev.GetConfig();

            double sum = 0.0;
            for (int i = 0; i < init.size(); i++)
            {
                double elem = diff[i];
                if (DELTA < distance)
                    init[i] += (elem/distance) * DELTA;
                else
                    init[i] += elem;
            }

            RRTNode qnew(qnear.GetIndex(), init);
            if (CheckCollision(qnew))
                return 0;   

            qnew.SetIndex(tree.GetNodes().size());
            tree.AddNode(qnew);
            tree.AddEdge(qprev, qnew);

            qprev = qnew;
        }

        RRTNode qfinal = tree.GetNodes()[tree.GetNodes().size()-1];
        if (CompareConfig(q, qfinal))
            return 2;
        else
        {
            return 1;
        }
    }

    std::vector<RRTNode> RRT_Connect(RRTNode start, RRTNode goal, RobotBasePtr probot)
    {
        std::vector<RRTNode> nodes;

        if (bidirect == 1)
        {
            NodeTree tree_a(start, goal, nodes, probot);
            NodeTree tree_b(goal, start, nodes, probot);
            
            time_t start, cur;
            time(&start);
            time(&cur);
            while (difftime(cur,start) <= MAX_TIME)
            {
                RRTNode qrand = Sample();
                if (Extend(tree_a, qrand) != 0)
                {
                    RRTNode qnew = tree_a.GetNodes()[tree_a.GetNodes().size()-1];
                    if(Bidirect_Connect(tree_b, qnew) == 2)
                    {
                        if (CompareConfig(qrand, this->_goalNode))
                            return Path(tree_a, qnew);
                    }
                }
                Swap(&tree_a, &tree_b);
                time(&cur);
            }
            std::vector<RRTNode> p;
            return p;
        }
        else
        {
            NodeTree tree(start, goal, nodes, probot);

            time_t start, cur;
            time(&start);
            time(&cur);
            while (difftime(cur,start) <= MAX_TIME)
            {
                RRTNode qrand = Sample();
                if(Connect(tree, qrand) == 2)
                {
                    RRTNode last = tree.GetNodes()[tree.GetNodes().size()-1];
                    if (CompareConfig(qrand, _goalNode))
                    {
                        cout << "Found Path in " << difftime(cur,start) << " seconds\n";
                        return Path(tree, last);                    
                    }
                }
                time(&cur);
            }
            //std::vector<RRTNode> p;
            //return p;

            double min_dist = DBL_MAX;
            RRTNode min_node;

            for (RRTNode& node : tree.GetNodes())
            {
                double dist = node.Distance(_goalNode, _probot);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_node = node;
                }
            }
            cout << "MIN_DIST " << min_dist << endl;

            cout << "CLOSEST NODE ";
            for (double& joint : min_node.GetConfig())
                cout << joint << " ";
            cout << endl;
            return Path(tree, min_node);
        }
    }
};

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrt" ) {
        return InterfaceBasePtr(new RRT(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRT");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

