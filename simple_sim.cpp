#include <iostream>
#include <cmath>
#include <random>
#include <vector>
#include <string>
#include <memory>
#include <limits>
#include <chrono>
#include <thread>
#include <set>
#include <queue>

struct Position{    // Represents a position in the grid with x and y coordinates
    int x;
    int y;

    bool operator<(const Position& other) const{ // Compare two positions based on their x and y coordinates for ordering in a set
        if (x == other.x) return y < other.y; // If x coordinates are equal, compare y coordinates
        return x < other.x; // If x coordinates are not equal, compare x coordinates
    }
    bool operator==(const Position& other) const{ // Check if two positions are equal based on their x and y coordinates
        return x == other.x && y == other.y; // Return true if both x and y coordinates are equal, otherwise return false
    }
};

struct AStarNode{   // Represents a node in the A* algorithm with a position and f_score
    Position pos;
    int f_score;
};

struct CompareNodes{    // Comparator for the priority queue used in A* algorithm to compare nodes based on their f_score
    bool operator()(const AStarNode& a, const AStarNode& b) const{  
        return a.f_score > b.f_score;
    }
};

class Environment{  
    private:
        int width;
        int height;

        // FOV
        // u-r; r; d-r; u-l; l; d-l; u; d
        // const int dx[8] = {1, 1, 1, -1, -1, -1, 0, 0}; 
        // const int dy[8] = {1, 0, -1, 1, 0, -1, 1, -1};

        // 4-directional movement (right, down, left, up)
        const int dx[4] = {1, 0, -1, 0};
        const int dy[4] = {0, 1, 0, -1};
    public:
        int getWidth() const { return width; };
        int getHeight() const { return height; };
        std::vector<std::vector<int>>grid;
        std::vector<std::vector<int>>visits;
        std::set<Position> active_frontiers;   // Set of positions representing the active frontiers (unknown tiles) in the environment
        // -1...unknown, 0...known free, 1...known obstacle
        Environment(int w, int h, int default_value) : width(w), height(h){ // Initialize the grid and visits matrices with the specified dimensions and default values
            grid = std::vector<std::vector<int>>(height, std::vector<int>(width, default_value));
            visits = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));
        }

        void addObstacles(int x, int y){    // Mark the specified position in the grid as an obstacle (1) if it's within bounds
            if (x >=0 && x < width && y >=0 && y < height){
                grid[y][x] = 1;
            }
        }

        // Check if there is a clear line of sight between two points (x0, y0) and (x1, y1) in the grid, considering obstacles
        bool hasLineOfSight(int x0, int y0, int x1, int y1, std::vector<std::vector<int>>& map_data){
            int dx = std::abs(x1 - x0); // Calculate the absolute difference in x-coordinates between the two points
            int dy = std::abs(y1 - y0); // Calculate the absolute difference in y-coordinates between the two points
            int sx = (x0 < x1) ? 1 : -1; // Determine the step direction in the x-axis (1 for right, -1 for left)
            int sy = (y0 < y1) ? 1 : -1; // Determine the step direction in the y-axis (1 for down, -1 for up)
            int err = dx - dy; // Initialize the error term for Bresenham's line algorithm
            
            while (true){
                if (x0 == x1 && y0 == y1){ // If the current point has reached the endpoint, return true (line of sight exists)
                    return true;
                }

                // Check if the current point is not the endpoint and is an obstacle (1), return false (no line of sight)
                if ((x0 != x1 || y0 != y1) && map_data[y0][x0] != 0){
                    return false;
                }

                int e2 = 2 * err; // Calculate the error term for the next step
                int prev_x = x0; // Store the previous x-coordinate before moving to the next point
                int prev_y = y0; // Store the previous y-coordinate before moving to the next point

                if (e2 > -dy){ // If the error term is greater than -dy, move in the x-direction
                    err -= dy; // Update the error term by subtracting dy
                    x0 += sx; // Move to the next point in the x-direction based on the step direction
                }
                if (e2 < dx){ // If the error term is greater than dx, move in the y-direction
                    err += dx; // Update the error term by adding dx
                    y0 += sy; // Move to the next point in the y-direction based on the step direction
                }
                if (prev_x != x0 && prev_y != y0){ // If both x and y coordinates have changed (diagonal movement)
                    if (map_data[prev_y][x0] != 0 || map_data[y0][prev_x] != 0){ // If the horizontal or vertical neighbors of the current point are obstacles (1)
                        return false; // Return false (no line of sight)
                    }
                }
            }
            return true; // If the loop completes without encountering an obstacle, return true (line of sight exists)
        }

        // Update the field of view (FOV) of the drone based on its current position and the true map
        void updateFOV(int drone_x, int drone_y, std::vector<std::vector<int>>& true_map){  
            grid[drone_y][drone_x] = 0; // Mark the drone's current position as known free space (0) in the grid
            int radius = 4; // Define the radius of the field of view (FOV) around the drone's position
            int radius_squared = radius * radius; // Calculate the square of the radius for distance comparison

            // Update the 3x3 area around the drone's position to reflect the true map
            for (int dy = -radius; dy <= radius; ++dy){  
                for (int dx = -radius; dx <= radius; ++dx){
                    if (dx * dx + dy * dy > radius_squared){ // Skip cells that are outside the circular FOV radius
                        continue;
                    }

                    int view_x = drone_x + dx;  // Calculate the x-coordinate of the cell in the FOV based on the drone's position and the offset
                    int view_y = drone_y + dy;  // Calculate the y-coordinate of the cell in the FOV based on the drone's position and the offset

                    if (view_x >= 0 && view_x < width && view_y >= 0 && view_y < height){ // Check if the calculated coordinates are within the bounds of the grid
                        if (hasLineOfSight(drone_x, drone_y, view_x, view_y, true_map)){ // Check if there is a clear line of sight between the drone's position and the cell in the FOV, considering obstacles in the true map
                            grid[view_y][view_x] = true_map[view_y][view_x]; // Update the cell in the grid to reflect the corresponding value from the true map (0 for free space, 1 for obstacle) if there is a clear line of sight
                        }
                    }   
                }
            }
        }

        // Check if the specified position (x, y) is safe for the drone to move to (within bounds and not an obstacle)
        bool isSafe(int x, int y, bool require_known = false){
            if (x < 0 || x >= width || y < 0 || y >= height){
                return false;   // Return false if the position is out of bounds
            }
            if (grid[y][x] == 1){
               return false;
            }
            if (require_known && grid[y][x] == -1){
                return false;   // Return false if the position is unknown and require_known is true
            }
            return true;    // Return true if the position is safe for the drone to move to
        }

        // Get a list of legal moves (positions) the drone can move to from its current position (current_x, current_y)
        std::vector <Position> getLegalMoves(int current_x, int current_y, bool require_known = false){
            std::vector <Position> legal_moves;
            for (int i = 0; i < 4; ++i){
                int next_x = current_x + dx[i];
                int next_y = current_y + dy[i];

                // Check if the next position is safe (within bounds and not an obstacle) and add it to the list of legal moves
                if (isSafe(next_x, next_y, require_known)){
                    legal_moves.push_back({next_x, next_y});    
                }
            }
            return legal_moves;
        }

        // Explore the field of view (FOV) of the drone at its current position (drone_x, drone_y) and return the number of newly discovered tiles
        int exploreFOV(int drone_x, int drone_y){
            int newly_discovered = 0;   // Count of newly discovered tiles in the FOV
            int radius = 4; // Define the radius of the field of view (FOV) around the drone's position
            int radius_squared = radius * radius; // Calculate the square of the radius for distance comparison

            // Check the 3x3 area around the drone's position to see if any unknown tiles (-1) are discovered
            for (int dy = -radius; dy <= radius; ++dy){
                for (int dx = -radius; dx <= radius; ++dx){
                    if (dx * dx + dy * dy > radius_squared){ // Skip cells that are outside the circular FOV radius
                        continue;
                    }

                    int view_x = drone_x + dx; 
                    int view_y = drone_y + dy;

                    // Check if the calculated coordinates are within the bounds of the grid and if the tile is unknown (-1)
                    if (view_x >= 0 && view_x < width && view_y >= 0 && view_y < height){
                        if (hasLineOfSight(drone_x, drone_y, view_x, view_y, grid)){
                            if (grid[view_y][view_x] == -1){
                                newly_discovered += 1;  // Increment the count of newly discovered tiles
                                grid[view_y][view_x] = 0;   // Mark the tile as known free space (0) in the grid
                            }
                        }
                    }
                }    
            }
            return newly_discovered;    // Return the count of newly discovered tiles in the FOV
        }

        // Fill isolated -1 artifacts left by Bresenham raycaster gaps
        // Detects 1x1 -1 tiles completely surrounded by 0s and fills them
        void fillFOVHoles(int center_x, int center_y, int radius = 4){
            int radius_squared = radius * radius;
            
            for (int dy = -radius; dy <= radius; ++dy){
                for (int dx = -radius; dx <= radius; ++dx){
                    if (dx * dx + dy * dy > radius_squared) continue; // Only check within circular FOV
                    
                    int x = center_x + dx;
                    int y = center_y + dy;
                    
                    if (x < 0 || x >= width || y < 0 || y >= height) continue;
                    if (grid[y][x] != -1) continue; // Only examine unknown tiles
                    
                    // Check if this -1 tile is isolated (completely surrounded by 0s or boundaries)
                    bool is_isolated = true;
                    
                    // Scan all 8 neighbors
                    for (int ny = y - 1; ny <= y + 1; ++ny){
                        for (int nx = x - 1; nx <= x + 1; ++nx){
                            if (nx == x && ny == y) continue; // Skip self
                            
                            // If in bounds and contains another -1, it's not isolated
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height){
                                if (grid[ny][nx] == -1){
                                    is_isolated = false;
                                    break;
                                }
                            }
                            // Out of bounds treated as solid wall (boundary)
                        }
                        if (!is_isolated) break;
                    }
                    
                    // Fill isolated artifact with 0 (cleared free space)
                    if (is_isolated){
                        grid[y][x] = 0;
                    }
                }
            }
        }

        // Rebuild entire frontier set from scratch (cures amnesia bug from local erasures)
        // Call this periodically when stuck or when frontier count drops unexpectedly
        void scanGlobalFrontiers(){
            active_frontiers.clear();  // Wipe out old frontiers
            
            for (int y = 0; y < height; ++y){
                for (int x = 0; x < width; ++x){
                    if (isFrontier(x, y)){
                        active_frontiers.insert({x, y});
                    }
                }
            }
        }

        // Check if the entire environment has been fully explored
        // Only returns true if all REACHABLE -1 tiles are gone (ignores walled-off regions)
        bool isFullyExplored(){
            // Quick scan: any -1 at all?
            bool has_unknown = false;
            for (int y = 0; y < height; ++y){
                for (int x = 0; x < width; ++x){
                    if (grid[y][x] == -1){
                        has_unknown = true;
                        break;
                    }
                }
                if (has_unknown) break;
            }
            
            // No unknown tiles = fully explored
            if (!has_unknown) return true;
            
            // If unknown tiles exist, check if any are reachable from known free space via BFS
            // Unreachable/walled-off regions don't prevent completion
            std::queue<Position> frontier_q;
            std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
            
            // Seed BFS with all known free tiles
            for (int y = 0; y < height; ++y){
                for (int x = 0; x < width; ++x){
                    if (grid[y][x] == 0){
                        frontier_q.push({x, y});
                        visited[y][x] = true;
                    }
                }
            }
            
            // BFS to find if any -1 is reachable
            while (!frontier_q.empty()){
                Position current = frontier_q.front();
                frontier_q.pop();
                
                int offset[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (int i = 0; i < 4; ++i){
                    int nx = current.x + offset[i][0];
                    int ny = current.y + offset[i][1];
                    
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                    if (visited[ny][nx]) continue;
                    
                    visited[ny][nx] = true;
                    
                    // Found a reachable unknown tile = not fully explored
                    if (grid[ny][nx] == -1){
                        return false;
                    }
                    
                    // Continue BFS through known free space
                    if (grid[ny][nx] == 0){
                        frontier_q.push({nx, ny});
                    }
                }
            }
            
            // All reachable space is explored (unreachable -1s are ignored)
            return true;
        }


        Position current_target = {-1, -1};   // Current target position for the drone to move towards (initialized to an invalid position)
        void updateTargetLock(int drone_x, int drone_y){
            // If the current target is still valid and unknown, keep it as the target
            if (current_target.x != -1 && current_target.y != -1){
                if (active_frontiers.find(current_target) != active_frontiers.end()){
                    return; // If the current target is still unknown, keep it as the target
                }
            }
            current_target = getNearestFrontier({drone_x, drone_y}); // Find the nearest unknown tile (frontier) to the drone's current position    
        }

        bool isFrontier(int x, int y){
            if (grid[y][x] != 0){
                return false;   // If the tile is not known free space (0), it cannot be a frontier
            }
            int offset[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}; // Define the offsets for the four neighboring tiles (right, left, down, up)

            for (int i = 0; i < 4; ++i){
                int nx = x + offset[i][0]; // Calculate the x-coordinate of the neighboring tile
                int ny = y + offset[i][1]; // Calculate the y-coordinate of the neighboring tile
                if (nx >= 0 && nx < width && ny >= 0 && ny < height){
                    if (grid[ny][nx] == -1){
                        return true;    // If any neighboring tile is unknown (-1), the current tile is a frontier
                    }
                }
            }
            return false;   // If no neighboring tile is unknown, the current tile is not a frontier
        }

        void updateFrontiersLocal(int center_x, int center_y, int radius = 4){
            int check_radius = radius + 1;
            int check_radius_squared = check_radius * check_radius;

            for (int dy = -check_radius; dy <= check_radius; ++dy){
                for (int dx = - check_radius; dx <= check_radius; ++dx){
                    if (dx * dx + dy * dy > check_radius_squared){
                        continue;   // Only check within circular radius
                    }
                    int check_x = center_x + dx;
                    int check_y = center_y + dy;
                    if (check_x >= 0 && check_x < width && check_y >= 0 && check_y < height){
                        Position pos = {check_x, check_y};
                        if (isFrontier(check_x, check_y)){
                            active_frontiers.insert(pos);
                        } else {
                            active_frontiers.erase(pos);
                        }
                    }
                }
            }
        }

        Position getNearestFrontier(Position start){
            Position best_target = {-1, -1};
            int min_cost = 99999;
            double dist_weight = 1.0;
            double info_gain_weight = 1.5;
            double hysteresis = 5.0;

            for (const auto& frontier : active_frontiers){
                int distance = std::abs(start.x - frontier.x) + std::abs(start.y - frontier.y);
                int info_gain = getInformationGain(frontier);
                double cost = distance * dist_weight - info_gain * info_gain_weight - hysteresis * (frontier == current_target ? 1.0 : 0.0); // Apply hysteresis bonus if the frontier is the current target
                if (cost < min_cost){
                    min_cost = cost;
                    best_target = frontier;
                }
            }
            return best_target;
        }

        int getInformationGain(Position frontier, int radius = 4){
            int unknown_count = 0;
            int radius_squared = radius * radius;

            for (int dy = -radius; dy <= radius; ++dy){
                for (int dx = -radius; dx <= radius; ++dx){
                    if (radius_squared < dx * dx + dy * dy){
                        continue;   // Only check within circular radius
                    }
                    int check_x = frontier.x + dx;
                    int check_y = frontier.y + dy;
                    if (check_x >= 0 && check_x < width && check_y >= 0 && check_y < height){
                        if (grid[check_y][check_x] == -1){
                            unknown_count += 1;   // Count the number of unknown tiles (-1) within the specified radius around the frontier
                        }
                    }
                }
            }
            return unknown_count;   // Return the total count of unknown tiles within the specified radius around the frontier
        }

        bool isMCTSBored(int x, int y, int threshold = 7){
            int threshold_sq = threshold * threshold;   // Calculate the square of the threshold radius for distance comparison
            for (const auto& frontier : active_frontiers){
                int dx = frontier.x - x;   // Calculate the x-distance from the drone's current position to the frontier
                int dy = frontier.y - y;   // Calculate the y-distance from the drone's current position to the frontier
                if (dx * dx + dy * dy <= threshold_sq){   // If the squared distance to the frontier is within the threshold radius, the MCTS is not considered bored
                    return false;   // Return false indicating that there are frontiers within the threshold radius
                }
            }
            return true;    // If no frontiers are found within the threshold radius, the MCTS is considered bored
        }
};

// Monte Carlo Tree Search (MCTS) Node structure
struct MCTSNode{
    double visits;  // Number of times this node has been visited during simulations
    double score;   // Cumulative score (reward) obtained from simulations passing through this node
    double heuristic_bonus; // Additional heuristic bonus to encourage exploration of certain nodes based on domain-specific knowledge or strategies
    Position state; // The position in the environment represented by this node

    MCTSNode* parent;   // Pointer to the parent node in the MCTS tree (nullptr for the root node)
    std::vector<std::unique_ptr<MCTSNode>> children;    // Vector of unique pointers to child nodes in the MCTS tree, representing possible future states from this node

    // Constructor to initialize an MCTSNode with the given position and optional parent node
    MCTSNode(Position current_pos, MCTSNode* parent_node = nullptr, double h_bonus = 0.0)
    : visits(0.0), score(0.0), state(current_pos), parent(parent_node), heuristic_bonus(h_bonus) {}

    // Calculate the Upper Confidence Bound for Trees (UCT) score for this node, which balances exploration and exploitation
    double getUCTScore(double Cp = 1.414){   // Cp is the exploration parameter that controls the balance between exploration and exploitation (higher = more exploration)
        if (visits < 0.0001){   // If the node has not been visited yet, return infinity to prioritize exploration
            return std::numeric_limits<double>::infinity();
        }

        double heuristic = 0.0;

        double exploitation = score / visits;   // Calculate the exploitation term as the average score (reward) obtained from this node

        double exploration = 2.0 * Cp * std::sqrt(std::log(parent->visits) / visits);   // Calculate the exploration term based on the number of visits to the parent node and this node, encouraging exploration of less-visited nodes

        return exploitation + exploration + heuristic_bonus;  // Return the combined UCT score, which is a balance between exploitation (average reward) and exploration (uncertainty based on visits)
    }

    // Backpropagate the simulation reward up the tree
    void backpropagate(double simulation_reward){
        MCTSNode* current = this;   // Start from the current node and traverse up the tree to update the visits and scores of all ancestor nodes

        while (current != nullptr){ // While there are still ancestor nodes to update (until reaching the root node)
            // decay the old history
            //current->visits = current->visits * gamma;
            //current->score = current->score * gamma;
        
            // add new simulation data
            current->visits += 1.0; // Increment the visit count for the current node to reflect that it has been visited during this simulation
            current->score += simulation_reward;    // Add the simulation reward to the cumulative score for the current node, updating its total score based on the outcome of the simulation

            current = current->parent;  // Move to the parent node to continue backpropagating the reward up the tree
        }
    }

    MCTSNode* getBestChild(){   // Get the child node with the highest UCT score, which represents the most promising move based on the simulations
        MCTSNode* best_node = nullptr;  // Initialize a pointer to keep track of the child node with the best UCT score

        double best_score = -999999.0;  // Initialize the best score to a very low value to ensure that any valid child node will have a higher score

        for (int i = 0; i < children.size(); i++){  // Iterate through all child nodes of the current node to evaluate their UCT scores
            double current_score = children[i]->getUCTScore();  // Calculate the UCT score for the current child node

            if (current_score > best_score){    // If the current child's UCT score is better than the best score found so far, update the best score and best node
                best_score = current_score; 
                best_node = children[i].get();  // Update the best_node pointer to point to the current child node, which has the highest UCT score among the evaluated children
            }
        } 
        return best_node;   // Return the child node with the highest UCT score, which is considered the best move based on the simulations performed in the MCTS algorithm
    }
    
    // Get the child node with the highest average score (reward) among the children that have been visited at least once
    MCTSNode* getBestRealChild(Environment& map, Position prev_pos){   // Get the child node with the highest average score (reward) among the children that have been visited at least once
        MCTSNode* best_node = nullptr; 
        double best_avg_score = -9999999.0;

        // Calculate the heading vector from the previous position to the current node's position
        int heading_x = state.x - prev_pos.x;   // Calculate the x-component of the heading vector from the previous position to the current node's position
        int heading_y = state.y - prev_pos.y;   // Calculate the y-component of the heading vector from the previous position to the current node's position

        // Iterate through all child nodes to find the one with the highest average score (score divided by visits)
        for (int i = 0; i < children.size(); ++i){
            if (children[i]->visits > 0){   // Only consider child nodes that have been visited at least once to avoid division by zero and ensure meaningful average scores
                double avg_score = children[i]->score / children[i]->visits;    // Calculate the average score for the current child node by dividing its cumulative score by the number of visits
                
                int current_x = children[i]->state.x;   // Get the x-coordinate of the current child node's position
                int current_y = children[i]->state.y;   // Get the y-coordinate of the current child node's position

                int child_heading_x = current_x - state.x;   // Calculate the x-component of the heading vector from the current node's position to the child node's position
                int child_heading_y = current_y - state.y;   // Calculate the y-component of the heading vector from the current node's position to the child node's position

                if (current_x == prev_pos.x && current_y == prev_pos.y){
                    avg_score -= 9999.0;   // Apply a penalty to the average score for moving back to the previous position, discouraging backtracking and promoting exploration of new areas
                }else if (map.visits[current_y][current_x] > 0){
                    avg_score -= 500.0;    // Apply a penalty to the average score for moving to a position that has already been visited, discouraging revisiting and promoting exploration of new areas
                }else if (heading_x == child_heading_x && heading_y == child_heading_y){ // If the heading direction from the previous position to the current node is the same as the heading direction from the current node to the child node
                    avg_score += 0.5;  // Apply a small bonus to the average score for moving in the same direction as the previous heading, encouraging the drone to continue exploring in a consistent direction
                }

                if (avg_score > best_avg_score){    // If the current child's average score is better than the best average score found so far, update the best average score and best node
                    best_avg_score = avg_score;
                    best_node = children[i].get();
                }
            }
        }
        return best_node;   // Return the child node with the highest average score, which represents the most promising move based on the actual outcomes of simulations that have visited this node
    }

    // Expand the current node by generating child nodes for all legal moves from the current position in the environment
    void expandNode(Environment& map){
        int current_x = state.x;
        int current_y = state.y;

        std::vector<Position> possible_moves = map.getLegalMoves(current_x, current_y, true);

        for (int i = 0; i < possible_moves.size(); i++){
            int distance_to_frontier = 99999;   // Initialize the distance to the nearest frontier (unknown tile) to a large value
            
            if (map.current_target.x != -1){
                distance_to_frontier = std::abs(possible_moves[i].x - map.current_target.x) + std::abs(possible_moves[i].y - map.current_target.y);   // Calculate the Manhattan distance from the possible move to the current target (nearest frontier) if a valid target exists
            }
            
            double h_bonus = 0.0;
            if (distance_to_frontier != 99999){
                h_bonus = 20 / (distance_to_frontier + 1);   // Calculate a heuristic bonus based on the distance to the nearest frontier (unknown tile) to encourage exploration of areas that are closer to unexplored regions
            }
            children.push_back(std::make_unique<MCTSNode>(possible_moves[i], this, h_bonus));   // Create a new child node for each legal move and add it to the children vector, passing the current node as the parent and the calculated heuristic bonus
        }
    }
};

// Simulate the drone's movement and exploration in the environment starting from the given node for a maximum number of steps
double simulateActivePerception(MCTSNode* start_node, Environment current_map, int max_steps){
    int sim_x = start_node->state.x;    // Initialize the simulated x-coordinate of the drone's position to the x-coordinate of the starting node's state
    int sim_y = start_node->state.y;    // Initialize the simulated y-coordinate of the drone's position to the y-coordinate of the starting node's state
    
    if (current_map.visits[sim_y][sim_x] > 0){ // If the drone revisits a tile, apply a penalty to the total reward to discourage revisiting
        return -500; // Return a negative reward to indicate that the simulation is not favorable due to revisiting a tile
    }

    double total_reward = 0.0;  // Initialize the total reward accumulated during the simulation to zero
    double step_reward = 0.0;   // Initialize the reward for the current step to zero
    
    int prev_dx = 0;    // Initialize the previous x-direction of movement to zero (no movement yet)
    int prev_dy = 0;    // Initialize the previous y-direction of movement to zero (no movement yet)

    std::random_device rd;  // Create a random device to seed the random number generator for stochastic behavior in the simulation
    std::mt19937 gen(rd()); // Initialize a Mersenne Twister random number generator with the seed from the random device

    // Simulate the drone's movement for a maximum number of steps
    for (int step = 0; step < max_steps; step++){
        std::vector<Position> legal_moves = current_map.getLegalMoves(sim_x, sim_y, true);    // Get legal moves from the current position

        if (legal_moves.empty()){   // If there are no legal moves available, break the simulation loop as the drone is stuck
            break;
        }

        std::vector<double> move_weights;   // Vector to store weights for each legal move, which will be used to bias the selection of moves based on exploration and direction

        // Calculate weights for each legal move
        for (const auto& move : legal_moves){
            double weight = 1.0;

            int move_dx = move.x - sim_x;   // Calculate the x-direction of the move relative to the current position
            int move_dy = move.y - sim_y;   // Calculate the y-direction of the move relative to the current position

            if (move_dx == prev_dx && move_dy == prev_dy){
                weight += 3.0;  // Increase weight for moves that continue in the same direction as the previous move, encouraging the drone to maintain its heading and explore in a consistent direction
            }

            // Encourage the drone to move towards unknown areas
            int unknown_count = 0;
            // Check the 8 neighboring cells of the move position
            for (int dx = -1; dx <= 1; ++dx){
                for (int dy = -1; dy <= 1; ++dy){
                    int neighbor_x = move.x + dx; // Calculate the neighbor's x-coordinate
                    int neighbor_y = move.y + dy; // Calculate the neighbor's y-coordinate
                    // Check if the neighbor is within bounds
                    if (neighbor_x >= 0 && neighbor_x < current_map.getWidth() && neighbor_y >= 0 && neighbor_y < current_map.getHeight()){
                        // Check if the neighbor cell is unknown
                        if (current_map.grid[neighbor_y][neighbor_x] == -1){ 
                            weight += 2.0; // Increase weight for moves that lead to unknown cells
                            //unknown_count++;    // Increment the count of unknown neighboring cells to encourage exploration of unknown areas
                        }
                    }
                }
            }

            int proximity_penalty = 0;
            for (int dy = -3; dy <= 3; ++dy){
                for (int dx = -3; dx <= 3; ++dx){
                    int check_x = move.x + dx;
                    int check_y = move.y + dy;
                    if (check_x >= 0 && check_x < current_map.getWidth() && check_y >= 0 && check_y < current_map.getHeight()){
                        if (current_map.visits[check_y][check_x] > 0){
                            proximity_penalty++; // Count the number of visited cells within a 3-cell radius to apply a penalty for moving near previously
                        }
                    }
                }
            }
            if (proximity_penalty > 0){
                weight /= (1.0 + (proximity_penalty * 10.0)); // Apply a penalty to the weight for moves that are near previously visited cells, discouraging revisiting and promoting exploration of new areas
            }
            
            //weight *= (1.0 + (unknown_count * 5.0)); // Increase weight for moves that lead to more unknown cells
            move_weights.push_back(weight);
        }
        // discrete distribution to select a move based on weights
        std::discrete_distribution<> distrib(move_weights.begin(), move_weights.end());
        int choice = distrib(gen);

        // Update previous direction
        prev_dx = legal_moves[choice].x - sim_x;
        prev_dy = legal_moves[choice].y - sim_y;

        // Move to the chosen position
        sim_x = legal_moves[choice].x;
        sim_y = legal_moves[choice].y;

        if (current_map.visits[sim_y][sim_x] > 0){ // If the drone revisits a tile, apply a penalty to the total reward to discourage revisiting
            total_reward -= 500.0;
            break;  // Break the simulation loop to avoid further penalties and encourage exploration of new areas
        }

        current_map.visits[sim_y][sim_x] += 1; // Increment the visit count for the new position to track how many times it has been visited during the simulation

        double step_tentative_penalty = 0.0; // Initialize a tentative penalty for the current step to zero
        for (int dy = -3; dy <= 3; dy++){
            for (int dx = -3; dx <= 3; dx++){
                int check_x = sim_x + dx;
                int check_y = sim_y + dy;
                if (check_x >= 0 && check_x < current_map.getWidth() && check_y >= 0 && check_y < current_map.getHeight()){
                    if (dx != 0 || dy != 0){ // Exclude the current position from the penalty check
                        if (current_map.visits[check_y][check_x] > 0){
                            step_tentative_penalty += 1.0;
                        }
                    }
                }
            }
        }

        // Explore the FOV and accumulate the reward
        double new_tiles = current_map.exploreFOV(sim_x, sim_y);

        current_map.updateFrontiersLocal(sim_x, sim_y);

        step_reward += (new_tiles * 20.0); // reward for discovering new tiles
        step_reward -= 1.0; // small penalty for each step to encourage shorter paths

        step_reward -= (step_tentative_penalty * 10.0); // penalty for being near previously visited tiles
    
        double gamma = 0.9; // discount factor for future rewards
        total_reward += (step_reward * std::pow(gamma, step));
    }
    return total_reward;
}

// Generate a true map with the specified width, height, and number of obstacles
std::vector<std::vector<int>> generateTrueMap (int width, int height, int num_obstacles){
    std::vector<std::vector<int>> true_map (height, std::vector<int> (width, 0));   // Initialize the true map with all cells set to 0 (free space)

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(0, width-1);  // Create a uniform integer distribution for generating random x-coordinates within the bounds of the map
    std::uniform_int_distribution<> dis_y(0, height-1); // Create a uniform integer distribution for generating random y-coordinates within the bounds of the map

    int placed = 0; // Count of obstacles placed in the true map

    while (placed < num_obstacles){ // Continue placing obstacles until the desired number is reached
        int obs_x = dis_x(gen); // Generate a random x-coordinate for the obstacle using the uniform distribution
        int obs_y = dis_y(gen); // Generate a random y-coordinate for the obstacle using the uniform distribution

        if (!(obs_x == 0 && obs_y == 0) && true_map[obs_y][obs_x] == 0){    // Ensure that the obstacle is not placed at the starting position (0, 0) and that the selected cell is currently free (0) before placing the obstacle
            true_map[obs_y][obs_x] = 1;
            placed++;
        }
    }

    return true_map;
}

MCTSNode* selectNode(MCTSNode* root){   // Select a node to expand based on the UCT score
    MCTSNode* current = root;   // Start from the root node
    while(!current->children.empty()){  // While the current node has children, continue traversing down the tree
        MCTSNode* next = current->getBestChild();   // Get the child node with the best UCT score
        if (next == nullptr){   // If there are no valid children, break the loop
            break;
        }
        current = next; // Move to the next node for further exploration
    }
    return current; // Return the selected node for expansion
}

int calculateHeuristic(Position start, Position goal){  // Calculate the Manhattan distance between the start and goal positions
    return std::abs(start.x - goal.x) + std::abs(start.y - goal.y); // Return the calculated Manhattan distance as the heuristic value
}

std::vector<Position> calculateAStar(Position start, Position goal, Environment& map){
    std::vector<std::vector<int>> g_score(map.getHeight(), std::vector<int>(map.getWidth(), 99999));    // Initialize g_score with a large value
    std::vector<std::vector<Position>> came_from(map.getHeight(), std::vector<Position>(map.getWidth(), {-1, -1})); // Initialize came_from with invalid positions
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNodes> open_set;  // Priority queue to store nodes to be evaluated

    open_set.push({start, calculateHeuristic(start, goal)});    // Push the start node into the open set with its f_score
    g_score[start.y][start.x] = 0;  // Set the g_score of the start node to 0

    while (!open_set.empty()){  // While there are nodes to evaluate
        AStarNode current_node = open_set.top();    // Get the node with the lowest f_score from the open set
        open_set.pop(); // Remove the current node from the open set

        int current_x = current_node.pos.x; // Get the x-coordinate of the current node
        int current_y = current_node.pos.y; // Get the y-coordinate of the current node

        if (current_x == goal.x && current_y == goal.y){    // If the goal is reached, reconstruct the path
            std::vector<Position> escape_path;   // Initialize a vector to store the final path
            Position current_trace = {current_x, current_y};    // Start tracing back from the goal to the start

            while (current_trace.x != start.x || current_trace.y != start.y){   // While the current trace is not the start position
                escape_path.push_back(current_trace);    // Add the current trace position to the final path
                current_trace = came_from[current_trace.y][current_trace.x];    // Move to the previous position in the path using came_from
            }
            std::reverse(escape_path.begin(), escape_path.end()); // Reverse the final path to get the correct order from start to goal
            return escape_path;  // Return the final path from start to goal
        }

        std::vector<Position> neighbors = map.getLegalMoves(current_x, current_y, true);  // Get the legal moves (neighbors) from the current position
        for (const auto& neighbor : neighbors){ // Iterate through each neighbor of the current node
            int neighbor_x = neighbor.x;    // Get the x-coordinate of the neighbor
            int neighbor_y = neighbor.y;    // Get the y-coordinate of the neighbor

            int proximity_penalty = 0;   // Initialize a penalty for moving to a neighbor that is close to obstacles
            for (int dy = -3; dy <= 3; ++dy){
                for (int dx = -3; dx <= 3; ++dx){
                    int check_x =neighbor_x + dx;   // Calculate the x-coordinate of the tile to check around the neighbor
                    int check_y = neighbor_y + dy;   // Calculate the y-coordinate of the tile to check around the neighbor
                    if (check_x >= 0 && check_x < map.getWidth() && check_y >= 0 && check_y < map.getHeight()){
                        if (map.visits[check_y][check_x] > 0){
                            proximity_penalty += 3;   // Increase the penalty for each visited tile within a 3-tile radius of the neighbor, discouraging paths that go near previously visited areas
                        }
                    }
                }
            }

            int tentative_g_score = g_score[current_y][current_x] + 1 + proximity_penalty;  // Calculate the tentative g_score for the neighbor (current g_score + cost to move to neighbor + proximity penalty)

            if (tentative_g_score < g_score[neighbor_y][neighbor_x]){   // If the tentative g_score is better than the previously recorded g_score for the neighbor
                came_from[neighbor_y][neighbor_x] = {current_x, current_y}; // Record the current node as the best path to reach the neighbor
                g_score[neighbor_y][neighbor_x] = tentative_g_score;    // Update the g_score for the neighbor with the better score

                int h_score = calculateHeuristic({neighbor_x, neighbor_y}, goal);   // Calculate the heuristic score (h_score) for the neighbor using the Manhattan distance to the goal
                int f_score = tentative_g_score + h_score;  // Calculate the f_score for the neighbor (g_score + h_score)

                open_set.push({neighbor, f_score}); // Push the neighbor node into the open set with its f_score for future evaluation
            } 
        }
    }
    return {};  // Return an empty path if no path is found
}

bool isInBounds(int x, int y, Environment& map){   // Check if the given coordinates (x, y) are within the bounds of the grid defined by width and height
    return (x >= 0 && x < map.getWidth() && y >= 0 && y < map.getHeight());   // Return true if the coordinates are within bounds, false otherwise
}

int main(){
    // initialisation
    int width, height, start_x, start_y;
    int threshold = 7; // Threshold for determining if the drone is stuck (number of steps without discovering new tiles)
    Position prev_pos = {-1, -1};

    if (!(std::cin >> width >> height >> start_x >> start_y) ) return 0;

    Environment true_map(width, height, 0);
    Environment known_map(width, height, -1);
    Position drone_pos ={start_x, start_y};

    int max_game_steps = 1000;

    for (int y = 0; y < height; ++y){
        for (int x = 0; x < width; ++x){
            int tile;
            std::cin >> tile;
            if (tile == 1){
                true_map.addObstacles(x, y);
            }
        }
    }

    std::vector<Position> escape_path;
    bool is_backtracking = false;

    std::cout << "MAP LOADED. STARTING LOOP...\n" << std::flush;

    // game loop
    for (int game_step = 0; game_step < max_game_steps; game_step++){
        known_map.updateFOV(drone_pos.x, drone_pos.y, true_map.grid);
        
        //known_map.fillFOVHoles(drone_pos.x, drone_pos.y);  // Fill Bresenham raycaster gaps immediately after FOV

        known_map.updateFrontiersLocal(drone_pos.x, drone_pos.y);

        if (known_map.isFullyExplored()){
            std::cout << "DONE\n" << std::flush;
            break;
        }

        known_map.updateTargetLock(drone_pos.x, drone_pos.y);

        if (is_backtracking){   // If the drone is currently backtracking to a previous position, continue following the escape path
            if (!escape_path.empty()){  // If there are still positions left in the escape path, move to the next position
                drone_pos = escape_path[0]; // Update the drone's position to the next position in the escape path
                escape_path.erase(escape_path.begin()); // Remove the first position from the escape path as it has been visited

                known_map.visits[drone_pos.y][drone_pos.x] += 1;    // Increment the visit count for the new position in the known map to track how many times the drone has visited this tile
                std::cout << "PATH " << drone_pos.x << " " << drone_pos.y << "\n" << std::flush;    // Output the current position of the drone to indicate the path it is following while backtracking
                std::this_thread::sleep_for(std::chrono::milliseconds(100));    // Pause for a short duration to visualize the drone's movement in the simulation
                std::cout << drone_pos.x << " " << drone_pos.y << "\n" << std::flush;   // Output the current position of the drone again to indicate its movement in the simulation
                
                continue;   // Continue to the next iteration of the game loop to keep backtracking until the escape path is empty (skip MCTS)
            }
            else{
                is_backtracking = false;   // If the escape path is empty, stop backtracking and proceed with MCTS for exploration
            }
        }

        auto root = std::make_unique<MCTSNode>(drone_pos);  // Create a new root node for the MCTS tree with the current drone position as the state

        // MCTS loop
        for (int i = 0; i < 1000; i++){    // Perform 10,000 iterations of the MCTS algorithm to explore possible future states and gather information for decision-making
            MCTSNode* leaf = selectNode(root.get());    // Select a leaf node from the MCTS tree to expand based on the UCT score
            leaf->expandNode(known_map);    // Expand the selected leaf node by generating child nodes for all legal moves from its current position in the known map

            MCTSNode* sim_node = leaf;  // Initialize the simulation node to the selected leaf node for running the simulation of the drone's movement and exploration
            if (!leaf->children.empty()){
                sim_node = leaf->children[0].get();
            }

            double reward = simulateActivePerception(sim_node, known_map, 20);
            
            sim_node->backpropagate(reward);
        }

        std::cout << "PATH ";
        MCTSNode* plan_node = root.get();
        for (int p = 0; p < 8; ++p){
            if (plan_node == nullptr || plan_node->children.empty()) break;

            plan_node = plan_node->getBestRealChild(known_map, prev_pos);
            if (plan_node != nullptr){ 
                std::cout << plan_node->state.x << " " << plan_node->state.y << " ";
            } else{
                break;
            }
        }
        std::cout << "\n" << std::flush;

        std::cout << "FRONTIERS ";
        for (const auto& frontier : known_map.active_frontiers){
            std::cout << frontier.x << " " << frontier.y << " ";
        }
        std::cout << "\n" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        MCTSNode* best_next_node = root->getBestRealChild(known_map, prev_pos);

        bool is_bored = known_map.isMCTSBored(drone_pos.x, drone_pos.y, 6);

        if (best_next_node != nullptr){ // If a valid next node is found, move the drone to that position
            int new_x = best_next_node->state.x; // Get the x-coordinate of the best next node's position
            int new_y = best_next_node->state.y; // Get the y-coordinate of the best next node's position

            if (known_map.visits[new_y][new_x] > 0 || is_bored){ // If the best next node has already been visited, it indicates that the drone is surrounded by previously visited tiles and may need to backtrack to find new frontiers
                std::vector<Position> best_escape_path; // Calculate the best escape path to the nearest frontier
                bool path_found = false;
                
                // Try local frontier set first
                while (best_escape_path.empty()){
                    Position target_frontier = known_map.getNearestFrontier(drone_pos); // Get the nearest frontier position from the current drone position to determine where to backtrack for exploration

                    if (target_frontier.x != -1){ // If a valid target frontier is found
                        best_escape_path = calculateAStar(drone_pos, target_frontier, known_map); // Calculate the A* path from the current drone position to the target frontier

                        if (target_frontier == drone_pos){
                            known_map.active_frontiers.erase(target_frontier); // If the target frontier is the same as the current drone position, remove it from the active frontiers set to avoid redundant exploration
                            continue; // Set path_found to true since the drone is already at the target frontier
                        }

                        if (!best_escape_path.empty()){ // If a valid escape path is found
                                path_found = true;
                                break;
                        } else{
                            known_map.active_frontiers.erase(target_frontier); // If no valid escape path is found, remove the target frontier from the active frontiers set and continue searching for another frontier
                        }
                    } else{ 
                        // No frontier found in local set - perform global scan to recover lost frontiers
                        known_map.scanGlobalFrontiers();
                        Position target_frontier_retry = known_map.getNearestFrontier(drone_pos);
                        
                        if (target_frontier_retry.x != -1){
                            // Found a frontier via global scan, retry A*
                            best_escape_path = calculateAStar(drone_pos, target_frontier_retry, known_map);
                            if (!best_escape_path.empty()){
                                path_found = true;
                                break;
                            }
                        }
                        // Truly stuck - no reachable frontiers exist
                        break;
                    }
                }

                if (path_found){
                    escape_path = best_escape_path; // Set the escape path to the calculated best escape path 
                    std::cout << "[C++] GOING TO FRONTIER " << escape_path.back().x << ", " << escape_path.back().y << " " << std::flush;
                    std::cout << "ASTAR_PATH " << drone_pos.x << " " << drone_pos.y << " " << std::flush;
                    for (const auto& pos : escape_path){
                        std::cout << pos.x << " " << pos.y << " ";
                    }
                    std::cout << "\n" << std::flush;

                    is_backtracking = true;
                    continue;   // Skip the rest of the loop and continue backtracking to the nearest frontier
                } else{
                    std::cout << "DONE\n" << std::flush;
                    break;
                }
            }

            prev_pos = drone_pos;
            drone_pos = best_next_node->state;
            known_map.visits[drone_pos.y][drone_pos.x] += 1;
            std::cout << drone_pos.x << " " << drone_pos.y << "\n" << std::flush;
        }
        else{
            std::cout << "STUCK\n" << std::flush;
            break;
        }
        // Pause slightly to visualize the simulation
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Check if the environment is fully explored
        if (known_map.isFullyExplored()){
            std::cout << "DONE\n" << std::flush;
            break;
        }

        // If the maximum number of game steps is reached and the environment is not fully explored, print "STUCK"
        if (game_step == max_game_steps - 1){
            std::cout << "STUCK\n" << std::flush;
        }
    }
    return 0;
}
