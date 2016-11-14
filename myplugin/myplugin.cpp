#include "RRT.hpp"

#define MAX_TIME 180.00
#define DELTA 0.05
#define GOAL_SAMPLE 0.1
#define GOAL_ERROR 0.05
#define SMOOTHING_ITER 200

double goal_bias;
double step_size;

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
        for (uint i = 0; i < nodes.size(); i++)
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
    for (uint i = index; i < this->_nodes.size(); i++)
        this->_nodes[index].SetIndex(this->_nodes[index].GetIndex()-1);
}

/*
Method used to set the parent of the node.
The parent is an integer giving the position of the node's parent 
in the tree.
*/
void NodeTree::AddEdge(RRTNode a, RRTNode b)
{
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
    RobotBase::ManipulatorPtr _pmanip; 
    std::vector<double>_startConfig;
    std::vector<double>_goalConfig;
    std::vector<double>_jointWeights;
    std::vector<OpenRAVE::GraphHandlePtr> _graphPtrVec;
    RRTNode _startNode;
    RRTNode _goalNode;

    int bidirect;

public:
    RRT(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("Set_Goal_Bias",boost::bind(&RRT::Set_Goal_Bias,this,_1,_2),
                        "Command to set goal bias"),
        RegisterCommand("Set_Bidirectional",boost::bind(&RRT::Set_Bidirectional,this,_1,_2),
                        "Command to perform bidirectional RRT"),
        RegisterCommand("Set_Joint_Weights",boost::bind(&RRT::Set_Joint_Weights,this,_1,_2),
                        "Command to set joint weights"),
        RegisterCommand("Start_RRT",boost::bind(&RRT::Start_RRT,this,_1,_2),
                        "Command to start RRT");
        this->_penv = penv;
    }
    virtual ~RRT() {}

    /*
    Method to start the goal bias value
    Called as a command.
    */
    bool Set_Goal_Bias(std::ostream& sout, std::istream& sinput)
    {
        int count = 0;
        while (sinput.good() && count < 1)
        {
            std::string input;
            sinput >> input;
            goal_bias = stod(input);
            count++;
        }
        return true;
    }

    /*
    Method to send the joint weight vector to the plugin from the Python module
    Called as a command.
    */
    bool Set_Joint_Weights(std::ostream& sout, std::istream& sinput)
    {
        while (sinput.good())
        {
            std::string input;
            sinput >> input;
            this->_jointWeights.push_back(stod(input));
        }
        return true;
    }

    /*
    Method to set whether or not the planner will solve the problem using single
    or bidirectional RRT
    Called as a command.
    */
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
        return true;
    }

    /*
    Method to start the RRT Procedure from the Python module
    Called as a command.
    */
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
        _probot->SetActiveManipulator("leftarm");
        this->_pmanip = this->_probot->GetActiveManipulator();

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
            cout << "No Trajectory Found" << endl;
        }
        
        return true;
    }

    /*
        Method for getting a random sample. 
        Returns the random sample node
    */
    RRTNode Sample()
    {
        std::random_device rd;
        std::mt19937_64 gen(rd());
        std::uniform_real_distribution<double> g_dist(0.0, std::nextafter(1.0, DBL_MAX));
        double g_prob = g_dist(gen);

        if (g_prob <= goal_bias)
            return _goalNode;

        RRTNode ret;
        std::vector<int> indices = _probot->GetActiveDOFIndices();
        std::vector<double> u_limits;
        std::vector<double> l_limits;

        _probot->GetActiveDOFLimits(l_limits, u_limits);

        std::mt19937_64 ran(12 + rd());
        for (uint c = 0; c < u_limits.size(); c++)
        {
            std::uniform_real_distribution<double> distribution(l_limits[c], std::nextafter(u_limits[c], DBL_MAX));
            double number = distribution(ran);

            if (abs(l_limits[c]) > 2*M_PI || u_limits[c] > 2*M_PI)
                number = atan2(sin(number), cos(number));

            ret.SetConfigElement(c, number);
        }
        return ret; 
    }

    bool CheckCollision(RRTNode q)
    {
        _probot->SetActiveDOFValues(q.GetConfig());
        return GetEnv()->CheckCollision(_probot) || _probot->CheckSelfCollision();
    }

    bool CompareConfig(RRTNode a, const RRTNode& b, std::vector<double> weights)
    {
        if (weights.size() > 0)
            return a.ModDistance(b, _probot, weights) < GOAL_ERROR;
        return a.Distance(b, _probot) < GOAL_ERROR;
    }

    void Swap(NodeTree* a, NodeTree* b)
    {
        NodeTree* temp  = a;
        a               = b;
        b               = temp;
    }

    std::vector<RRTNode> Path(NodeTree a, RRTNode q)
    {
        /*
        ofstream myfile;
        myfile.open ("final_output_3.txt.txt",  ios::out | ios::app);
        myfile << "Number of nodes sampled " << a.GetNodes().size() << "\n";
        myfile.close();
        */
        std::vector<RRTNode> path;

        path.push_back(q);

        while (q.GetParent() > -1)
        {
            q = a.GetNodes()[q.GetParent()];
            path.push_back(q);  
        }
        path.push_back(_startNode);
        std::reverse(path.begin(), path.end());

        float orig_path_length = 0.0;
        std::vector<float> prev_pos;
        for (uint i = 0; i < path.size(); i++)
        {
            RRTNode node = path[i];
            _probot->SetActiveDOFValues(node.GetConfig());
            RaveVector<double> trans = _pmanip->GetEndEffectorTransform().trans;

            std::vector<float> pos;
            pos.push_back(trans.x);
            pos.push_back(trans.y);
            pos.push_back(trans.z);

            std::vector<float> color;
            color.push_back(1);
            color.push_back(0);
            color.push_back(0);
            color.push_back(1);

            _graphPtrVec.push_back(GetEnv()->plot3(&pos[0],1,sizeof(pos),6.0,&color[0],0));

            if (i > 0)
                orig_path_length += sqrt((pos[0]-prev_pos[0]) * (pos[0]-prev_pos[0]) +
                                         (pos[1]-prev_pos[1]) * (pos[1]-prev_pos[1]) +
                                         (pos[2]-prev_pos[2]) * (pos[2]-prev_pos[2]));
            prev_pos = pos;
        }

        path = ShortcutSmoothing(path);
        _probot->SetActiveDOFValues(_startConfig);

        float new_path_length = 0.0;
        for (uint i = 0; i < path.size(); i++)
        {
            RRTNode node = path[i];
            _probot->SetActiveDOFValues(node.GetConfig());
            RaveVector<double> trans = _pmanip->GetEndEffectorTransform().trans;

            std::vector<float> pos;
            pos.push_back(trans.x);
            pos.push_back(trans.y);
            pos.push_back(trans.z);

            std::vector<float> color;
            color.push_back(0);
            color.push_back(0);
            color.push_back(1);
            color.push_back(1);

            _graphPtrVec.push_back(GetEnv()->plot3(&pos[0],1,sizeof(pos),6.0,&color[0],0));

            if (i > 0)
                new_path_length += sqrt((pos[0]-prev_pos[0]) * (pos[0]-prev_pos[0]) +
                                         (pos[1]-prev_pos[1]) * (pos[1]-prev_pos[1]) +
                                         (pos[2]-prev_pos[2]) * (pos[2]-prev_pos[2]));
            prev_pos = pos;
        }

        /*
        myfile.open ("final_output_3.txt.txt", ios::out | ios::app);
        myfile << "original path length " << orig_path_length << endl;
        myfile << "smoothed path length " << new_path_length << endl;
        myfile.close();
        */

        _probot->SetActiveDOFValues(_startConfig);
        return path;
    }

    NodeTree Path(const NodeTree& a, const NodeTree& b, RRTNode q)
    {
        NodeTree returnTree;
        return returnTree;
    }

    RRTNode NewConfig(RRTNode& q, RRTNode& qnear, NodeTree& tree)
    {
        //double distance           = qnear.Distance(q, _probot);
        double distance             = qnear.ModDistance(q, _probot, _jointWeights);
        std::vector<double> diff    = q.GetConfig();

        _probot->SubtractActiveDOFValues(diff, qnear.GetConfig()); //C-space vector from qnear = q

        vector<double> init = qnear.GetConfig();

        double sum = 0.0;
        for (uint i = 0; i < init.size(); i++)
        {  //new configuration is qnear config + stepsize of DELTA in respective directions
            double elem = diff[i];
            if (DELTA < distance)
                init[i] += (elem/distance) * DELTA;
            else
                init[i] += elem;
            sum += elem/distance;
        }

        RRTNode qnew(qnear.GetIndex(), init);
        return qnew;
    }

    int Extend(NodeTree& tree, RRTNode& q) //2 = reached, 1 = advanced, 0 = trapped
    {
        RRTNode qnear           = q.NearestNode(tree, _jointWeights);
        RRTNode qnew            = NewConfig(q, qnear, tree);
        std::vector<double>diff = q.GetConfig();
        _probot->SubtractActiveDOFValues(diff, qnear.GetConfig());

        if (!CheckCollision(qnew))
        {
            qnew.SetIndex(tree.GetNodes().size());
            tree.AddNode(qnew);
            tree.AddEdge(qnear, qnew);

            if (CompareConfig(q, qnew, _jointWeights))
                return 2;
            else
                return 1;
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

    /*
        Steps toward random sample until either collision or convergence
    */
    int Connect(NodeTree& tree, RRTNode& q)
    {
        RRTNode qnear = q.NearestNode(tree, _jointWeights);
        double dist = qnear.ModDistance(q, _probot, _jointWeights);
        int max_steps = ceil(dist/DELTA);

        RRTNode qprev = qnear;
        for (int i = 0; i < max_steps; i++)
        {
            double distance             = qprev.ModDistance(q, _probot, _jointWeights);
            std::vector<double> diff    = q.GetConfig();
            std::vector<double> init    = qprev.GetConfig();

            _probot->SubtractActiveDOFValues(diff, qprev.GetConfig());
            for (uint i = 0; i < init.size(); i++)
            {
                double elem = diff[i];
                if (DELTA < distance)
                    init[i] += (elem/distance) * DELTA;
                else
                    init[i] += elem;
            }

            RRTNode qnew(qprev.GetIndex(), init);
            if (CheckCollision(qnew))
                return 0;   

            qnew.SetIndex(tree.GetNodes().size());
            tree.AddNode(qnew);
            tree.AddEdge(qprev, qnew);

            qprev = qnew;
        }

        RRTNode qfinal = tree.GetNodes()[tree.GetNodes().size()-1];
        if (CompareConfig(q, qfinal, _jointWeights))
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
                        if (CompareConfig(qrand, this->_goalNode, _jointWeights))
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
                    if (CompareConfig(qrand, _goalNode, _jointWeights))
                    {
                        /*
                        ofstream myfile;
                        myfile.open ("final_output_3.txt.txt",  ios::out | ios::app);
                        myfile << "Found Path in " << difftime(cur,start) << " seconds w/ goal bias " << goal_bias << "\n";
                        myfile.close();
                        */

                        return Path(tree, last);                    
                    }   
                }
                time(&cur);
            }
            std::vector<RRTNode> p;

            /*
            ofstream myfile;
            myfile.open ("final_output_3.txt.txt",  ios::out | ios::app);
            myfile << "No path found\n";
            myfile << "Number of nodes sampled " << tree.GetNodes().size() << "\n";
            myfile.close();
            */

            return p;
        }
    }

    std::vector<RRTNode> ShortcutSmoothing(std::vector<RRTNode> path)
    {
        std::vector<double> u_limits;
        std::vector<double> l_limits;

        _probot->GetActiveDOFLimits(l_limits, u_limits);

        const long double startTime = time(0);
        const long double startimeMS = startTime*1000;

        for (int m = 0; m < SMOOTHING_ITER; m++)
        {
            std::random_device rd;
            std::mt19937_64 gen(rd());

            std::uniform_real_distribution<double> rand_one(0, path.size());
            int rand_min = rand_one(gen);

            std::uniform_real_distribution<double> rand_two(rand_min+1, path.size());
            int rand_max = rand_two(gen);

            RRTNode start_conf = path[rand_min];
            RRTNode end_conf = path[rand_max];

            if (rand_min >= path.size() || rand_min == rand_max)
                continue;

            RRTNode qprev    = start_conf;
            double distance  = start_conf.Distance(end_conf, _probot);
            int max_steps    = ceil(distance/DELTA);

            for (int i = 0; i < max_steps; i++)
            {
                std::vector<double> diff = end_conf.GetConfig();
                std::vector<double> init = qprev.GetConfig();
                double dist              = qprev.Distance(end_conf, _probot);

                _probot->SubtractActiveDOFValues(diff, qprev.GetConfig());
                for (uint i = 0; i < init.size(); i++)
                {
                    double elem = diff[i];
                    if (DELTA < dist)
                        init[i] += (elem/dist) * DELTA;
                    else
                        init[i] += elem;
                }

                qprev.SetConfig(init);
                if (CheckCollision(qprev))
                    break;
            }
            if (CompareConfig(end_conf, qprev, _jointWeights))
            {
                std::vector<RRTNode> temp;
                for (int i = 0; i < rand_min; i++)
                {
                    temp.push_back(path[i]);
                }
                temp.push_back(start_conf);
                temp.push_back(end_conf);
                for (uint i = rand_max+1; i < path.size(); i++)
                {
                    temp.push_back(path[i]);
                }
                path = temp;
            }

            float new_path_length = 0.0;
            std::vector<float> prev_pos;
            for (uint i = 0; i < path.size(); i++)
            {
                RRTNode node = path[i];
                _probot->SetActiveDOFValues(node.GetConfig());
                RaveVector<double> trans = _pmanip->GetEndEffectorTransform().trans;
                std::vector<float> pos;
                pos.push_back(trans.x);
                pos.push_back(trans.y);
                pos.push_back(trans.z);


                if (i > 0)
                    new_path_length += sqrt((pos[0]-prev_pos[0]) * (pos[0]-prev_pos[0]) +
                                             (pos[1]-prev_pos[1]) * (pos[1]-prev_pos[1]) +
                                             (pos[2]-prev_pos[2]) * (pos[2]-prev_pos[2]));

                prev_pos = pos;
            }
        }
        //Add intermediate nodes in smoothed nodes for a better plotted path;
        std::vector<RRTNode> final;
        for (uint i = 0; i < path.size()-1; i++)
        {
            RRTNode start_conf = path[i];
            RRTNode end_conf = path[i+1];

            RRTNode qprev    = start_conf;
            double distance  = start_conf.Distance(end_conf, _probot);
            int max_steps    = ceil(distance/DELTA);

            for (int i = 0; i < max_steps; i++)
            {
                std::vector<double> diff = end_conf.GetConfig();
                std::vector<double> init = qprev.GetConfig();
                double dist              = qprev.Distance(end_conf, _probot);

                _probot->SubtractActiveDOFValues(diff, qprev.GetConfig());
                for (uint i = 0; i < init.size(); i++)
                {
                    double elem = diff[i];
                    if (DELTA < dist)
                        init[i] += (elem/dist) * DELTA;
                    else
                        init[i] += elem;
                }
                qprev.SetConfig(init);
                RRTNode inter(qprev.GetParent(), init);
                final.push_back(inter);
            }
        }
        const long double curTime = time(0);
        const long double curTimeMS = curTime*1000;

        /*
        ofstream myfile;
        myfile.open ("final_output_3.txt.txt", ios::out | ios::app);
        myfile << "smoothing computation time " << (curTimeMS - startimeMS) << endl;
        myfile.close();
        */

        return final;
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

