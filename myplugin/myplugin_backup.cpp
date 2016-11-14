#include "RRT.hpp"

#define MAX_ITERATIONS 100000
#define MAX_TIME 180.00
#define DELTA 0.01
#define MAX_DIST 99999999
#define GOAL_SAMPLE 0.05
#define GOAL_ERROR 0.01
#define VARIANCE 0.01

//RRTNode non-constructor and non-get methods
double RRTNode::Distance(const RRTNode &node, OpenRAVE::RobotBasePtr probot)
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

RRTNode RRTNode::NearestNode(NodeTree tree, bool kdtree = false)
{
    RRTNode returnNode;
    if (kdtree)
    {

    }
    else
    {
        double min_dist = MAX_DIST;
        vector<RRTNode>nodes = tree.GetNodes();
        for (int i = 0; i < nodes.size(); i++)
        {
            RRTNode curNode = (nodes[i]);
            double dist = this->Distance(curNode, tree.GetRobot());
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
    RRTNode _startNode;
    RRTNode _goalNode;

    int bidirect;

public:
    RRT(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&RRT::MyCommand,this,_1,_2),
                        "This is an example command"),
        RegisterCommand("Start_RRT",boost::bind(&RRT::Start_RRT,this,_1,_2),
                        "This is an example command");
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

    bool Start_RRT(std::ostream& sout, std::istream& sinput)
    {
        int count = 0;
        while (sinput.good())
        {
            std::string input;
            sinput >> input;

            if (count == 0)
                bidirect = stoi(input);
            else
                this->_goalConfig.push_back(stod(input));
            count++;
        }
        EnvironmentMutex::scoped_lock lock(_penv->GetMutex());

        this->_probot = _penv->GetRobot("PR2");
        this->_probot->GetActiveDOFValues(this->_startConfig);


        this->_startNode.SetConfig(this->_startConfig);
        this->_goalNode.SetConfig(this->_goalConfig);

        cout << (GetEnv()->CheckCollision(_probot) || _probot->CheckSelfCollision()) << endl;

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
        Method for getting the midpoint between two configurations for use in midpoint sampling
        Returns (a + b) * 0.5;
    */
    std::vector<double> MidConfig(std::vector<double> a, std::vector<double> b)
    {
        std::vector<double> ret;
        ret.resize(a.size());
        for (int i = 0; i < a.size(); i++)
            ret[i] = (a[i] + b[i]) * 0.5;
        return ret;
    }

    /*
        Method for getting a random sample. 
        @Param gaussian = 1: Parameter for enabling or disabling gaussian-based sampling, which also
                             incorporates midpoint sampling as a ginal results
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

        if (gaussian == 1)
        {
            RRTNode ret2;
            curr = 0;
            for (int& index : indices)
            {
                double mean = ret.GetConfig()[index];
                double lb = abs(mean - l_limits[curr]);
                double ub = abs(mean - u_limits[curr]);

                double variance = min(VARIANCE, min(lb, ub));
                std::random_device rd;
                std::mt19937_64 gen(rd());
                std::normal_distribution<double> distribution(mean, variance);
                double number = distribution(gen);

                if (abs(l_limits[curr]) > 2*M_PI || u_limits[curr] > 2*M_PI)
                    number = atan2(sin(number), cos(number));

                ret2.SetConfigElement(curr, number);
                curr++;
            }

            bool col_ret = CheckCollision(ret);
            bool col_ret2 = CheckCollision(ret2); 
            if(!col_ret != !col_ret2)
            {
                if (col_ret)
                    return ret2;
            }
            return ret;
        }
        else
        {
            return ret; 
        }

    }

    bool CheckCollision(RRTNode q)
    {
        //save state before modifying it
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

    NodeTree Path(const NodeTree& a, const NodeTree& b)
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
        for (int i = 0; i < init.size(); i++)
        {  //new configuration is qnear config + stepsize of DELTA in respective directions
            double elem = diff[i];
            if (DELTA < distance)
                init[i] += (elem/distance) * DELTA;
            else
                init[i] += (elem/distance) * distance;
        }

        RRTNode qnew(qnear.GetIndex(), init);
        return qnew;
    }

    int Extend(NodeTree& tree, RRTNode& q) //2 = reached, 1 = advanced, 0 = trapped
    {
        RRTNode qnear = q.NearestNode(tree);
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

    int Connect(NodeTree& tree, RRTNode& q)
    {
        int s = 0;
        do
        {
            s = Extend(tree, q);
        }
        while(s == 1);
        return s;
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
                    if(Connect(tree_b, qnew) == 2)
                    {
                        if (CompareConfig(qrand, this->_goalNode))
                        {
                            cout << "PATH FOUND\n";
                            return Path(tree_a, qnew);
                        }
                        else
                            cout << "REACHED\n";
                    }
                    else
                        cout << "ADVANCED\n";
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
                if (Extend(tree, qrand) != 0)
                {
                    RRTNode qnew = tree.GetNodes()[tree.GetNodes().size()-1];

                    /*
                    //check qrand by printing
                    cout << "qrand ";
                    for (double& joint : qrand.GetConfig())
                        cout << joint << " ";
                    cout << endl;
                    //end qrand check

                    //check qnew by printing
                    cout << "qnew ";
                    for (double& joint : qnew.GetConfig())
                        cout << joint << " ";
                    cout << endl;
                    cout << "qnew parent: " << qnew.GetParent() << endl;
                    //end qnew check
                    */

                    if(Connect(tree, qrand) == 2)
                    {
                        RRTNode last = tree.GetNodes()[tree.GetNodes().size()-1];
                        if (CompareConfig(qrand, _goalNode))
                        {
                            cout << "PATH FOUND " << tree.GetNodes().size() << endl;
                            return Path(tree, last);
                        }
                        //else
                            //cout << "REACHED\n";
                    }
                    //else
                        //cout << "ADVANCED\n";
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
            return Path(tree, min_node);        }
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

