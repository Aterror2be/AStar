struct Node
{
    int x, y; // Coordinates
    int g, h, f; // Costs
    Node* parent; // Pointer to parent node

    // Constructor
    Node(int x, int y, int g, int h, int f, Node* parent) : x(x), y(y), g(g), h(h), f(f), parent(parent) {}
};

struct PairHash
{
    template <typename T1, typename T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2;
    }
};

struct PairEqual
{
    template <typename T1, typename T2>
    bool operator() (const std::pair<T1, T2>& p1, const std::pair<T1, T2>& p2) const {
        return p1.first == p2.first && p1.second == p2.second;
    }
};

class AStar
{
public:
    enum class Heuristic
    {
        Manhattan,
        Euclidean,
        Octagonal
    };

    AStar(int start_x, int start_y, int end_x, int end_y, std::vector<std::vector<int>> input_map)
    {
        //save a copy of the map to a instance of the class
        map = input_map;

        //create the start and end nodes with the given x and y values
        start_node = new Node(start_x, start_y, 0, 0, 0, nullptr);
        end_node = new Node(end_x, end_y, 0, 0, 0, nullptr);
    }

    ~AStar()
    {
    }

    std::vector<std::pair<int, int>> FindPath(bool allow_diagonals = false, Heuristic heuristic = AStar::Heuristic::Manhattan)
    {
        open_list.push_back(start_node);

        while (open_list.size() > 0)
        {
            Node* current_node = open_list[0];

            //find the lowest f cost and set that to the current node if equal find the one with lower h cost
            int iterator = 0;
            for (int i = 1; i < open_list.size(); i++)
            {
                if (open_list[i]->f < current_node->f || open_list[i]->f == current_node->f)
                {
                    if (open_list[i]->h < current_node->h)
                    {
                        current_node = open_list[i];
                        iterator = i;
                    }
                }
            }
            open_list.erase(open_list.begin() + iterator);
            closed_list.insert({ current_node->x, current_node->y });

            pointer_list.push_back(current_node);

            if (current_node->x == end_node->x && current_node->y == end_node->y)
            {
                //printf("Done Time To Retrace The Path\n");

                std::vector<std::pair<int, int>> result;

                while (current_node->parent != nullptr)
                {
                    result.push_back({ current_node->x, current_node->y });

                    current_node = current_node->parent;
                }

                result.push_back({ start_node->x, start_node->y });

                std::reverse(result.begin(), result.end());

                Cleanup();
                return result;
            }

            /*
            +-------+-------+-------+                    +-------+
            |  -1,1 |  0,1  |  1,1  |                    |  0,1  |
            +-------+-------+-------+            +-------+-------+-------+
            |-1,0   |   +   |  1,0  |     vs     |-1,0   |   +   |  1,0  |
            +-------+-------+-------+            +-------+-------+-------+
            | -1,-1 |  0,-1 | 1,-1  |                    |  0,-1 |
            +-------+-------+-------+                    +-------+
            */

            std::vector<std::pair<int, int>> directions;

            (allow_diagonals) ? directions = { {-1, 1}, {0, 1}, {1, 1}, {-1, 0}, {1, 0}, {-1, -1}, {0, -1}, {1, -1} } : directions = { {0, 1}, {-1, 0}, {1, 0}, {0, -1} };

            for (int i = 0; i < directions.size(); i++)
            {
                int x = current_node->x + directions[i].first, y = current_node->y + directions[i].second;

                if (map[x][y] == 1) continue;

                if (closed_list.find({x, y}) != closed_list.end())
                {
                    continue;
                }

                bool should_skip = false;
                for (int p = 0; p < open_list.size(); p++)
                {
                    if (open_list[p]->x == x && open_list[p]->y == y)
                    {
                        should_skip = true;
                        break;
                    }
                }

                if (should_skip) continue;

                //The cost from the start to your current position following the current path
                int g_cost = GetDistance(current_node->x, current_node->y, x, y, heuristic) + current_node->g;
                //the estimated distance from the current node to the end ignoring all walls
                int h_cost = GetDistance(x, y, end_node->x, end_node->y, heuristic);
                //the total cost of g_cost and h_cost combined
                int f_cost = g_cost + h_cost;

                Node* neighbour = new Node(x, y, g_cost, h_cost, f_cost, current_node);

                open_list.push_back(neighbour);
            }
        }

        //printf("[!] Failed To Find Path\n");
        Cleanup();
        return {};
    }

private:
    std::vector<std::vector<int>> map;

    std::vector<Node*> open_list;
    std::unordered_set<std::pair<int, int>, PairHash, PairEqual> closed_list;

    //A list of saved pointers for cleanup later
    std::vector<Node*> pointer_list;

    Node* start_node;
    Node* end_node;

    int GetDistance(int a_x, int a_y, int b_x, int b_y, Heuristic heuristic)
    {
        switch (heuristic)
        {
        case AStar::Heuristic::Manhattan:
        {
            return ManhattanDistance(a_x, a_y, b_x, b_y);
            break;
        }
        case AStar::Heuristic::Euclidean:
        {
            return EuclideanDistance(a_x, a_y, b_x, b_y);
            break;
        }
        case AStar::Heuristic::Octagonal:
        {
            return OctagonalDistance(a_x, a_y, b_x, b_y);
            break;
        }
        default:
            break;
        }
        return -1;
    }

    int EuclideanDistance(int a_x, int a_y, int b_x, int b_y)
    {
        int distance_x = std::abs(a_x - b_x);
        int distance_y = std::abs(a_y - b_y);
    
        return static_cast<int>(10 * std::sqrt(std::pow(distance_x, 2) + std::pow(distance_y, 2)));
    }
    
    int  OctagonalDistance(int a_x, int a_y, int b_x, int b_y)
    {
        int distance_x = std::abs(a_x - b_x);
        int distance_y = std::abs(a_y - b_y);
    
        return 10 * (distance_x + distance_y) + (-6) * min(distance_x, distance_y);
    }
    
    int ManhattanDistance(int a_x, int a_y, int b_x, int b_y)
    {
        int distance_x = std::abs(a_x - b_x);
        int distance_y = std::abs(a_y - b_y);
    
        if (distance_x > distance_y)
        {
            return 14 * distance_y + 10 * (distance_x - distance_y);
        }
        return 14 * distance_x + 10 * (distance_y - distance_x);
    }

    void Cleanup()
    {
        //printf("AStar's Memory Has Been Released %li\n", open_list.size());

        for (auto const& element : open_list)
        {
            delete element;
        }

        for (auto const& element : pointer_list)
        {
            delete element;
        }

        free(end_node);
    }
};
