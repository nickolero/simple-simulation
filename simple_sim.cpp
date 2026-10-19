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
#include <iomanip>
#include <algorithm>

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

class TSDFMap{
    private: 
        int width;
        int height;
        double max_dist;

        std::vector<std::vector<double>> tsdf_grid; // 2D grid representing the TSDF values for each cell in the environment
        
        const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    public:
        TSDFMap(int w, int h, double max_truncation_dist = 5.0) : width(w), height(h), max_dist(max_truncation_dist){
            tsdf_grid = std::vector<std::vector<double>>(height, std::vector<double>(width, max_dist)); // Initialize the TSDF grid with the specified dimensions and set all values to max_dist
        }

        void updateTSDF(const std::vector<Position>& new_walls){
            if (new_walls.empty()) return; // If there are no new walls to update, return early

            using Element = std::pair<double, Position>; // Define a pair type to hold the distance and position for the priority queue
            std::priority_queue<Element, std::vector<Element>, std::greater<Element>> pq; // Min-heap priority queue to process cells based on their distance to the nearest wall

            for (const auto& wall : new_walls){
                if (wall.x >= 0 && wall.x < width && wall.y >= 0 && wall.y < height){
                    tsdf_grid[wall.y][wall.x] = 0.0; // Set the TSDF value of the wall cell to 0 (indicating a wall)
                    pq.push({0.0, wall}); // Add the wall cell to the priority queue with a distance of 0
                }
            }

            while (!pq.empty()){
                auto [dist, pos] = pq.top(); // Get the cell with the smallest distance from the priority queue
                pq.pop(); // Remove the cell from the priority queue

                if (dist > tsdf_grid[pos.y][pos.x]) continue; // If the current distance is greater than the stored TSDF value, skip processing this cell

                if (dist >= max_dist) continue; // If the distance exceeds the maximum truncation distance, skip processing this cell

                for (int i = 0; i < 8; ++i){
                    int nx = pos.x + dx[i]; // Calculate the x-coordinate of the neighboring cell
                    int ny = pos.y + dy[i]; // Calculate the y-coordinate of the neighboring cell
                    
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height){
                        double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0; // Calculate the step cost based on whether the movement is diagonal (1.414) or straight (1.0)
                        double new_dist = dist + step_cost; // Calculate the distance to the neighboring cell
                    
                        if (new_dist < tsdf_grid[ny][nx]){
                            tsdf_grid[ny][nx] = new_dist; // Update the TSDF value of the neighboring cell with the new distance
                            pq.push({new_dist, {nx, ny}}); // Add the neighboring cell to the priority queue for further processing
                        }
                    }
                }
            }
        }

        double getTSDFValue(int x, int y) const{
            if (x >= 0 && x < width && y >= 0 && y < height){
                return tsdf_grid[y][x];
            }
            return -1.0; // Return -1.0 if the position is out of bounds
        }

        std::pair<double, double> getSafetyGradient(int x, int y) const{
            if (x < 0 || x >= width || y < 0 || y >= height){
                return {0.0, 0.0};
            }

            auto sample = [&](int sample_x, int sample_y){
                sample_x = std::clamp(sample_x, 0, width - 1);
                sample_y = std::clamp(sample_y, 0, height - 1);
                return tsdf_grid[sample_y][sample_x];
            };

            double gradient_x = sample(x + 1, y) - sample(x - 1, y);
            double gradient_y = sample(x, y + 1) - sample(x, y - 1);
            double magnitude = std::hypot(gradient_x, gradient_y);

            if (magnitude <= 1e-6){
                return {0.0, 0.0};
            }
            return {gradient_x / magnitude, gradient_y / magnitude};
        }

        void TSDFToVisualiser(){
            std::cout << "TSDF ";
            for (int y = 0; y < height; ++y){
                for (int x = 0; x < width; ++x){
                    std::cout << std::fixed << std::setprecision(1) << tsdf_grid[y][x] << " ";
                }
            }
            std::cout << "\n" << std::flush;
        }

};

class Environment{  
    private:
        int width;
        int height;
        int fov_radius;
        int camera_fov_radius;

        // FOV
        // 8-directional movement (E, SE, S, SW, W, NW, N, NE)
        const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    public:
        int getWidth() const { return width; };
        int getHeight() const { return height; };
        std::vector<std::vector<int>>grid;
        std::vector<std::vector<int>>visits;
        std::vector<std::vector<int>> camera_observations;
        std::set<Position> active_frontiers;
        std::set<Position> lidar_frontiers;
        std::set<Position> camera_frontiers;
        std::set<Position> current_ndlf;
        std::set<Position> pending_ndlf_cells;
        Position current_ndlf_anchor = {-1, -1};
        int current_ndlf_id = 0;
        bool ndlf_initialized = false;
        int ndlf_remaining_count = 0;
        // Grid states are -1 unseen, 0 LIDAR-seen, 1 wall, and 2 camera-covered.
        Environment(int w, int h, int default_value, int fov, int camera_fov) : width(w), height(h), fov_radius(fov), camera_fov_radius(std::max(1, camera_fov)){ // Initialize the dual-sensor state.
            grid = std::vector<std::vector<int>>(height, std::vector<int>(width, default_value));
            visits = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));
            camera_observations = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));
        }

        int currentNDLFRemaining() const{
            return ndlf_remaining_count;
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
                if ((x0 != x1 || y0 != y1) && map_data[y0][x0] == 1){
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
                    if (map_data[prev_y][x0] == 1 || map_data[y0][prev_x] == 1){ // If the horizontal or vertical neighbors of the current point are obstacles (1)
                        return false; // Return false (no line of sight)
                    }
                }
            }
            return true; // If the loop completes without encountering an obstacle, return true (line of sight exists)
        }

        // Camera rays may pass through known free or camera-covered cells, but not through unseen fog.
        bool hasCameraLineOfSight(int x0, int y0, int x1, int y1) const{
            int delta_x = std::abs(x1 - x0);
            int delta_y = std::abs(y1 - y0);
            int step_x = (x0 < x1) ? 1 : -1;
            int step_y = (y0 < y1) ? 1 : -1;
            int error = delta_x - delta_y;

            while (true){
                if (x0 == x1 && y0 == y1){
                    return true;
                }
                if ((x0 != x1 || y0 != y1) && (grid[y0][x0] == -1 || grid[y0][x0] == 1)){
                    return false;
                }
                int doubled_error = 2 * error;
                int previous_x = x0;
                int previous_y = y0;
                if (doubled_error > -delta_y){
                    error -= delta_y;
                    x0 += step_x;
                }
                if (doubled_error < delta_x){
                    error += delta_x;
                    y0 += step_y;
                }
                if (previous_x != x0 && previous_y != y0 &&
                    (grid[previous_y][x0] == -1 || grid[y0][previous_x] == -1 || 
                     grid[previous_y][x0] == 1 || grid[y0][previous_x] == 1)){
                    return false;
                }
            }
        }

        // Update the field of view (FOV) of the drone based on its current position and the true map
        std::vector<Position> updateLidarFOV(int drone_x, int drone_y, std::vector<std::vector<int>>& true_map){  
            std::vector<Position> new_walls; // Vector to store the positions of newly discovered walls (obstacles) in the FOV
            if (grid[drone_y][drone_x] == -1){
                grid[drone_y][drone_x] = 0;
            }
            int radius = fov_radius; // Define the radius of the field of view (FOV) around the drone's position
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
                            if (grid[view_y][view_x] == -1 && true_map[view_y][view_x] == 1){ // If the cell in the grid is unknown (-1) and the corresponding cell in the true map is an obstacle (1)
                                new_walls.push_back({view_x, view_y}); // Add the position of the newly discovered wall (obstacle) to the new_walls vector
                            }
                            if (true_map[view_y][view_x] == 1){
                                grid[view_y][view_x] = 1;
                            } else if (grid[view_y][view_x] == -1){
                                grid[view_y][view_x] = 0;
                                if (ndlf_initialized){
                                    pending_ndlf_cells.insert({view_x, view_y});
                                }
                            }
                        }
                    }   
                }
            }
            //rebuildFrontiers();
            return new_walls;
        }

        // Keep the original method name as a compatibility wrapper for the LIDAR sensor.
        std::vector<Position> updateFOV(int drone_x, int drone_y, std::vector<std::vector<int>>& true_map){
            return updateLidarFOV(drone_x, drone_y, true_map);
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
        std::vector <Position> getLegalMoves(int current_x, int current_y, bool require_known, const TSDFMap& tsdf){
            std::vector <Position> legal_moves;
            for (int i = 0; i < 8; ++i){
                int next_x = current_x + dx[i];
                int next_y = current_y + dy[i];

                // Check if the next position is safe (within bounds and not an obstacle) and add it to the list of legal moves
                if (isSafe(next_x, next_y, require_known)){
                    //if (tsdf.getTSDFValue(next_x, next_y) > 0.0 && tsdf.getTSDFValue(next_x, next_y) < 1.0){
                    //    continue;   // Skip moves that lead to cells with TSDF values indicating proximity to obstacles
                    //}
                    if (!isSafe(next_x, current_y, require_known) && !isSafe(current_x, next_y, require_known)){
                        continue;   // Skip diagonal moves if both adjacent horizontal and vertical cells are obstacles
                    }
                    legal_moves.push_back({next_x, next_y});    
                }
            }
            return legal_moves;
        }

        // Store separate camera metrics so MCTS can reward discovery and punish overlap.
        struct CameraObservation {
            int newly_covered = 0;
            int overlap = 0;
            int obstacle_overlap = 0;
            int newly_covered_ndlf = 0;
        };


        CameraObservation estimateCameraFOV(int drone_x, int drone_y){
            CameraObservation observation;
            int radius = camera_fov_radius;
            int radius_squared = radius * radius; // Calculate the square of the radius for distance comparison

            // Check the FOV area around the drone's position to see if any unknown tiles (-1) are discovered
            for (int dy = -radius; dy <= radius; ++dy){
                for (int dx = -radius; dx <= radius; ++dx){
                    if (dx * dx + dy * dy > radius_squared){ // Skip cells that are outside the circular FOV radius
                        continue;
                    }

                    int view_x = drone_x + dx; 
                    int view_y = drone_y + dy;

                    // Check if the calculated coordinates are within the bounds of the grid.
                    if (view_x >= 0 && view_x < width && view_y >= 0 && view_y < height){
                        if (hasCameraLineOfSight(drone_x, drone_y, view_x, view_y)){
                            if (grid[view_y][view_x] == 1){
                                observation.obstacle_overlap++;
                            } else {
                                if (grid[view_y][view_x] == 0){
                                    observation.newly_covered++;
                                    if (isInsideCurrentNDLF({view_x, view_y})){
                                        observation.newly_covered_ndlf++;
                                    }
                                } else if (grid[view_y][view_x] == 2){
                                    observation.overlap++;
                                }
                            }
                        }
                    }
                }    
            }
            return observation;
        }

        // Observe with the smaller camera sensor and convert visible LIDAR cells from 0 to 2.
        CameraObservation observeCameraFOV(int drone_x, int drone_y){
            CameraObservation observation;
            int radius = camera_fov_radius;
            int radius_squared = radius * radius; // Calculate the square of the radius for distance comparison

            // Check the FOV area around the drone's position to see if any unknown tiles (-1) are discovered
            for (int dy = -radius; dy <= radius; ++dy){
                for (int dx = -radius; dx <= radius; ++dx){
                    if (dx * dx + dy * dy > radius_squared){ // Skip cells that are outside the circular FOV radius
                        continue;
                    }

                    int view_x = drone_x + dx; 
                    int view_y = drone_y + dy;

                    // Check if the calculated coordinates are within the bounds of the grid.
                    if (view_x >= 0 && view_x < width && view_y >= 0 && view_y < height){
                        if (hasCameraLineOfSight(drone_x, drone_y, view_x, view_y)){
                            if (grid[view_y][view_x] == 1){
                                observation.obstacle_overlap++;
                            } else {
                                camera_observations[view_y][view_x]++;
                                if (grid[view_y][view_x] == 0){
                                    observation.newly_covered++;
                                    grid[view_y][view_x] = 2;
                                    if (isInsideCurrentNDLF({view_x, view_y})){
                                        ndlf_remaining_count--;
                                    }
                                } else if (grid[view_y][view_x] == 2){
                                    observation.overlap++;
                                }
                            }
                        }
                    }
                }    
            }
            //rebuildFrontiers();
            return observation;
        }

        // Preserve the previous helper for callers that only need a new-cell count.
        int exploreFOV(int drone_x, int drone_y){
            return observeCameraFOV(drone_x, drone_y).newly_covered;
        }

        // Fill isolated -1 artifacts left by Bresenham raycaster gaps
        // Detects 1x1 -1 tiles completely surrounded by 0s and fills them
        void fillFOVHoles(int center_x, int center_y){
            int radius = fov_radius; // Define the radius of the field of view (FOV) around the drone's position
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

        // Return the connected component of a set of cells using 4-connected motion.
        std::set<Position> connectedComponent(const std::set<Position>& source, Position seed) const{
            std::set<Position> component;
            if (source.find(seed) == source.end()){
                return component;
            }
            std::queue<Position> pending;
            pending.push(seed);
            component.insert(seed);
            while (!pending.empty()){
                Position current = pending.front();
                pending.pop();
                for (int direction = 0; direction < 4; ++direction){
                    Position neighbor = {current.x + dx[direction * 2], current.y + dy[direction * 2]};
                    if (source.find(neighbor) != source.end() && component.insert(neighbor).second){
                        pending.push(neighbor);
                    }
                }
            }
            return component;
        }

        // Build the first NDLF from the initial LIDAR-visible free component.
        void initializeCurrentNDLF(Position seed){
            std::set<Position> lidar_cells;
            for (int y = 0; y < height; ++y){
                for (int x = 0; x < width; ++x){
                    if (grid[y][x] == 0){
                        lidar_cells.insert({x, y});
                    }
                }
            }
            current_ndlf = connectedComponent(lidar_cells, seed);
            ndlf_remaining_count = current_ndlf.size();
            current_ndlf_anchor = seed;
            current_ndlf_id = 1;
            ndlf_initialized = true;
            pending_ndlf_cells.clear();
            rebuildFrontiers();
        }

        // Check whether a position belongs to the current NDLF.
        bool isInsideCurrentNDLF(Position position) const{
            return current_ndlf.find(position) != current_ndlf.end();
        }

        // Activate the next NDLF only after the current NDLF has been completely camera-covered.
        bool activateNextNDLF(Position current_position){
            for (auto it = pending_ndlf_cells.begin(); it != pending_ndlf_cells.end();){
                if (grid[it->y][it->x] != 0){
                    it = pending_ndlf_cells.erase(it);
                } else {
                    ++it;
                }
            }
            
            if (currentNDLFRemaining() != 0 || pending_ndlf_cells.empty()){
                return false;
            }

            int closest_distance = 9999999;
            Position closest_ndlf = {-1, -1};
            for (const auto& pending_ndlf_cell : pending_ndlf_cells){
                int distance_to_ndlf = std::abs(pending_ndlf_cell.x - current_position.x) + std::abs(pending_ndlf_cell.y - current_position.y);
                if (distance_to_ndlf < closest_distance){
                    closest_distance = distance_to_ndlf;
                    closest_ndlf = pending_ndlf_cell;
                }
            }

            current_ndlf = connectedComponent(pending_ndlf_cells, closest_ndlf);
            ndlf_remaining_count = current_ndlf.size();
            
            for (const Position& cell : current_ndlf){
                pending_ndlf_cells.erase(cell);
            }
            
            current_ndlf_anchor = *current_ndlf.begin();
            current_ndlf_id++;
            rebuildFrontiers();
            
            return true;
        }

        // Recompute both frontier types from the monotonic grid state.
        void rebuildFrontiers(){
            lidar_frontiers.clear();
            camera_frontiers.clear();
            for (int y = 0; y < height; ++y){
                for (int x = 0; x < width; ++x){
                    if (grid[y][x] != 0){
                        continue;
                    }
                    bool next_to_unseen = false;
                    bool next_to_camera = false;
                    for (int direction = 0; direction < 4; ++direction){
                        int nx = x + dx[direction * 2];
                        int ny = y + dy[direction * 2];
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height){
                            continue;
                        }
                        next_to_unseen = next_to_unseen || grid[ny][nx] == -1;
                        next_to_camera = next_to_camera || grid[ny][nx] == 2;
                    }
                    if (next_to_unseen){
                        lidar_frontiers.insert({x, y});
                    }
                    if (next_to_camera){
                        camera_frontiers.insert({x, y});
                    }
                }
            }
            active_frontiers = camera_frontiers;
        }

        // Keep the old frontier-refresh entry point, now backed by both frontier sets.
        void scanGlobalFrontiers(){
            rebuildFrontiers();
        }

        // Check whether every reachable non-wall tile has been camera-covered.
        bool isFullyExplored(int start_x, int start_y, const TSDFMap& tsdf){
            std::queue<Position> covered_queue;
            std::vector<std::vector<bool>> covered_visited(height, std::vector<bool>(width, false));
            covered_queue.push({start_x, start_y});
            covered_visited[start_y][start_x] = true;

            while (!covered_queue.empty()){
                Position current = covered_queue.front();
                covered_queue.pop();
                for (const Position& move : getLegalMoves(current.x, current.y, false, tsdf)){
                    if (covered_visited[move.y][move.x]){
                        continue;
                    }
                    if (grid[move.y][move.x] == -1 || grid[move.y][move.x] == 0){
                        return false;
                    }
                    if (grid[move.y][move.x] == 2){
                        covered_visited[move.y][move.x] = true;
                        covered_queue.push(move);
                    }
                }
            }
            return grid[start_y][start_x] == 2;
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
        
            const int frontier_dx[4] = {1, -1, 0, 0};
            const int frontier_dy[4] = {0, 0, 1, -1};
            
            for (int i = 0; i < 4; ++i){
                int nx = x + frontier_dx[i]; // Calculate the x-coordinate of the neighboring tile
                int ny = y + frontier_dy[i]; // Calculate the y-coordinate of the neighboring tile
                if (nx >= 0 && nx < width && ny >= 0 && ny < height){
                    if (grid[ny][nx] == -1){
                        return true;    // If any neighboring tile is unknown (-1), the current tile is a frontier
                    }
                }
            }
            return false;   // If no neighboring tile is unknown, the current tile is not a frontier
        }

        void updateFrontiersLocal(int center_x, int center_y){
            int radius = fov_radius;
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
            double info_gain_weight = 0.9;
            double hysteresis = 15.0;

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

        int getInformationGain(Position frontier){
            int radius = fov_radius;
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

        bool isSurrounded(int drone_x, int drone_y, const TSDFMap& tsdf){
            std::vector<Position> moves = getLegalMoves(drone_x, drone_y, true, tsdf);   // Get the list of legal moves (positions) the drone can move to from its current position
            for (const auto& move : moves){
                // Any adjacent LIDAR-only cell is a valid local exploration opportunity.
                //if (grid[move.y][move.x] == 0){
                //    return false;
                //}
                // Revisited camera-covered cells do not prevent the A* fallback.
                //if (grid[move.y][move.x] == 2){
                //    continue;
                //}
                //if (visits[move.y][move.x] == 0){   // If any of the legal moves lead to an unvisited tile (visits count is 0), the drone is not considered surrounded
                //    return false;   // Return false indicating that there are unvisited tiles available for exploration
                //}
                if (estimateCameraFOV(move.x, move.y).newly_covered > 0){   // If any of the legal moves lead to a tile that would result in newly covered camera observations, the drone is not considered surrounded
                    return false;   // Return false indicating that there are opportunities for exploration through camera coverage
                }
            }
            return true;    // If all legal moves lead to visited tiles, the drone is considered surrounded
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

    // Add new root
    std::unique_ptr<MCTSNode> extractChild(Position next_pos){
        for (auto it = children.begin(); it != children.end(); ++it){ // Iterate through the child nodes of the current node to find a child with the specified next position
            if ((*it)->state == next_pos){ // If a child node with the specified next position is found, extract it as the new root of the MCTS tree
                std::unique_ptr<MCTSNode> new_root = std::move(*it); // Move the child node to a new unique pointer, effectively transferring ownership of the child node to the new root
                children.erase(it); // Remove the extracted child node from the current node's list of children, as it is now the new root of the MCTS tree
                new_root->parent = nullptr; // Set the parent of the new root node to nullptr, indicating that it is now the root of the MCTS tree
                return new_root;
            }
        }
        return nullptr;
    }

    // Calculate the Upper Confidence Bound for Trees (UCT) score for this node, which balances exploration and exploitation
    double getUCTScore(double Cp = 1.414){   // Cp is the exploration parameter that controls the balance between exploration and exploitation (higher = more exploration)
        if (visits < 0.0001){   // If the node has not been visited yet, return infinity to prioritize exploration
            return std::numeric_limits<double>::infinity();
        }

        double heuristic = 0.0;

        double exploitation = score / visits;   // Calculate the exploitation term as the average score (reward) obtained from this node

        double exploration = 2.0 * Cp * std::sqrt(std::log(parent->visits) / visits);   // Calculate the exploration term based on the number of visits to the parent node and this node, encouraging exploration of less-visited nodes

        double decaying_heuristic = heuristic_bonus / (1.0 + visits);   // Calculate a decaying heuristic bonus that decreases as the node is visited more often, encouraging exploration of nodes with higher heuristic values
        return exploitation + exploration + decaying_heuristic;  // Return the combined UCT score, which is a balance between exploitation (average reward) and exploration (uncertainty based on visits)
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

    MCTSNode* getBestChild(Environment& map){   // Get the child node with the highest UCT score, which represents the most promising move based on the simulations
        MCTSNode* best_node = nullptr;  // Initialize a pointer to keep track of the child node with the best UCT score
        double best_score = -std::numeric_limits<double>::infinity();  // Initialize the best score to negative infinity to ensure that any valid child node will have a higher score

        for (const auto& child : children){  // Iterate through all child nodes of the current node to evaluate their UCT scores
            const Position& position = child->state;  // Get the position represented by the current child node

            if (map.grid[position.y][position.x] == 1){
                continue;   // Skip walls; revisits remain legal and are penalized by the rollout.
            }
            
            double score = child->getUCTScore();  // Calculate the UCT score for the current child node

            if (score > best_score){    // If the current child's UCT score is better than the best score found so far, update the best score and best node
                best_score = score; 
                best_node = child.get();  // Update the best_node pointer to point to the current child node, which has the highest UCT score among the evaluated children
            }
        } 
        return best_node;   // Return the child node with the highest UCT score, which is considered the best move based on the simulations performed in the MCTS algorithm
    }
    
    // Get the child node with the highest average score (reward) among the children that have been visited at least once
    MCTSNode* getBestRealChild(Environment& map, const TSDFMap& tsdf, Position prev_pos, const std::vector<Position>& planned_path = {}){   // Select the highest-value child while avoiding low-clearance cells
        MCTSNode* best_node = nullptr; 
        double best_selection_score = -std::numeric_limits<double>::infinity();

        Position nearest_ndlf_zero = {-1, -1};
        int min_ndlf_dist = 999999;
        if (map.currentNDLFRemaining() > 0) {
            for (const auto& cell : map.current_ndlf) {
                if (map.grid[cell.y][cell.x] == 0) {
                    int d = std::abs(state.x - cell.x) + std::abs(state.y - cell.y);
                    if (d < min_ndlf_dist) {
                        min_ndlf_dist = d;
                        nearest_ndlf_zero = cell;
                    }
                }
            }
        }

        for (const auto& child : children){
            const Position& position = child->state;

            if (map.grid[position.y][position.x] == 1){
                continue;   // Skip walls and immediate backtracking; useful revisits remain legal.
            }

            bool already_planned = false;   // Flag to check if the child node's position is already part of the planned path
            for (const auto& planned : planned_path){
                if (planned == position){
                    already_planned = true;   // If the child node's position is found in the planned path, set the flag to true
                    break;
                }
            }

            if (already_planned){
                continue;   // Skip child nodes that are already part of the planned path or are the previous position, as they are not considered for selection
            }

            double avg_score = (child->visits > 0.0) ? child->score / child->visits : 0.0;
            double clearance = tsdf.getTSDFValue(position.x, position.y);
            double clearance_penalty = 0.0;
            constexpr double preferred_clearance = 2.5;

            if (clearance >= 0.0 && clearance < preferred_clearance){
                double deficit = preferred_clearance - clearance;
                clearance_penalty = 800.0 * deficit * deficit;
                if (clearance < 1.45){
                    clearance_penalty += 1500.0;
                }
            }

            double revisit_penalty = 150.0 * map.visits[position.y][position.x] * map.visits[position.y][position.x];

            double target_pull = 0.0;
            if (nearest_ndlf_zero.x != -1) {
                double dist_now = std::hypot(state.x - nearest_ndlf_zero.x, state.y - nearest_ndlf_zero.y);
                double dist_next = std::hypot(position.x - nearest_ndlf_zero.x, position.y - nearest_ndlf_zero.y);
                target_pull = 120.0 * (dist_now - dist_next);
            }

            double selection_score = avg_score + target_pull - clearance_penalty - revisit_penalty;
            // Apply the same active-NDLF lock at final action selection, not only in rollouts.
            // Preserve useful return paths while discouraging repeated ping-ponging.
            double exp_visits = 25.0 * (std::exp(0.8 * map.visits[position.y][position.x]) - 1.0); 
            
            if (position == prev_pos){
                exp_visits += 150.0;   // Apply a penalty for backtracking to the previous position, discouraging immediate reversals in movement
            }

            selection_score -= std::min(exp_visits, 600.0);   // Penalize child nodes that have been visited more frequently, discouraging repeated visits to the same position

            if (best_node == nullptr || selection_score > best_selection_score){
                best_node = child.get();
                best_selection_score = selection_score;
            }
        }
        return best_node;   // Return the child node with the highest average score, which represents the most promising move based on the actual outcomes of simulations that have visited this node
    }

    // Expand the current node by generating child nodes for all legal moves from the current position in the environment
    void expandNode(Environment& map, const TSDFMap& tsdf){
        int current_x = state.x;
        int current_y = state.y;

        std::vector<Position> possible_moves = map.getLegalMoves(current_x, current_y, true, tsdf);   // Get the list of legal moves (positions) the drone can move to from its current position, considering known free space and TSDF values

        for (int i = 0; i < possible_moves.size(); i++){
            
            // If the possible move leads to a tile that has already been visited, skip it
                // Keep revisited cells available so the drone can return and finish an NDLF.

            int distance_to_frontier = 99999;   // Initialize the distance to the nearest frontier (unknown tile) to a large value
            
            if (map.current_target.x != -1){
                distance_to_frontier = std::abs(possible_moves[i].x - map.current_target.x) + std::abs(possible_moves[i].y - map.current_target.y);   // Calculate the Manhattan distance from the possible move to the current target (nearest frontier) if a valid target exists
            }
            
                // Do not use a strong frontier heuristic here; the active NDLF reward controls locality.
                double h_bonus = 0.0;
            children.push_back(std::make_unique<MCTSNode>(possible_moves[i], this, h_bonus));   // Create a new child node for each legal move and add it to the children vector, passing the current node as the parent and the calculated heuristic bonus
        }
    }
};


double simulateActivePerception(MCTSNode* start_node, Environment current_map, int max_steps, const TSDFMap& tsdf, std::mt19937& gen, int incoming_dx, int incoming_dy){
    if (start_node == nullptr){
       return -99999.0;
    }

    int sim_x = start_node->state.x;
    int sim_y = start_node->state.y;
    int sim_prev_x = (start_node->parent != nullptr) ? start_node->parent->state.x : -1;
    int sim_prev_y = (start_node->parent != nullptr) ? start_node->parent->state.y : -1;
    double total_reward = 0.0;
    int prev_dx = incoming_dx;
    int prev_dy = incoming_dy;
    constexpr double gamma = 0.9;
    constexpr double preferred_clearance = 2.5;

    for (int step = 0; step < max_steps; step++){
        // Step 0 evaluates start_node->state itself! Subsequent steps pick a neighbor first.
        if (step > 0) {
            std::vector<Position> legal_moves = current_map.getLegalMoves(sim_x, sim_y, true, tsdf);
            if (legal_moves.empty()){
                break;
            }

            std::vector<double> move_weights;
            for (const auto& move : legal_moves){
                int prior_visits = current_map.visits[move.y][move.x];
                Environment::CameraObservation observation = current_map.estimateCameraFOV(move.x, move.y);
                int ndlf_prog = observation.newly_covered_ndlf;

                double clearance = tsdf.getTSDFValue(move.x, move.y);
                double clearance_pen = 0.0;
                if (clearance >= 0.0 && clearance < preferred_clearance){
                    double deficit = preferred_clearance - clearance;
                    clearance_pen = 600.0 * deficit * deficit;
                    if (clearance < 1.45) {
                        clearance_pen += 1000.0;
                    }
                }

                double revisit_pen = 120.0 * (prior_visits + 1) * (prior_visits + 1);

                //double exp_vis = 25.0 * (std::exp(0.8 * (prior_visits + 1)) - 1.0);
                
                double candidate_score = 400.0 * observation.newly_covered;
                candidate_score += 800.0 * ndlf_prog;
                candidate_score -= 20.0 * observation.overlap;
                candidate_score -= 15.0 * observation.obstacle_overlap;
                candidate_score -= clearance_pen;
                candidate_score -= revisit_pen;
                //candidate_score -= std::min(exp_vis, 600.0);

                // Smooth momentum bonus to prevent zig-zagging in corridors
                int move_dx = move.x - sim_x;
                int move_dy = move.y - sim_y;
                if (move_dx == prev_dx && move_dy == prev_dy) {
                    candidate_score += 60.0;
                }

                move_weights.push_back(std::exp(std::clamp(candidate_score / 250.0, -20.0, 20.0)));
            }

            std::discrete_distribution<> distrib(move_weights.begin(), move_weights.end());
            int choice = distrib(gen);

            sim_prev_x = sim_x;
            sim_prev_y = sim_y;
            prev_dx = legal_moves[choice].x - sim_x;
            prev_dy = legal_moves[choice].y - sim_y;
            sim_x = legal_moves[choice].x;
            sim_y = legal_moves[choice].y;
        }

        double step_reward = 0.0;
        int prior_visits = current_map.visits[sim_y][sim_x];
        current_map.visits[sim_y][sim_x]++;

        double tsdf_val = tsdf.getTSDFValue(sim_x, sim_y);
        if (tsdf_val <= 0.01){
            step_reward -= 99999.0;
            total_reward += (step_reward * std::pow(gamma, step));
            break;
        }

        double clearance_penalty = 0.0;
        if (tsdf_val < preferred_clearance){
            double deficit = preferred_clearance - tsdf_val;
            clearance_penalty = 600.0 * deficit * deficit;
            if (tsdf_val < 1.45){
                clearance_penalty += 1000.0;
            }
        }

        int ndlf_before = current_map.currentNDLFRemaining();
        Environment::CameraObservation observation = current_map.observeCameraFOV(sim_x, sim_y);
        int ndlf_after = current_map.currentNDLFRemaining();
        double ndlf_progress = ndlf_before - ndlf_after;
        double revisit_pen = 120.0 * prior_visits * prior_visits;

        step_reward += 400.0 * observation.newly_covered;
        step_reward += 800.0 * ndlf_progress;
        step_reward -= 20.0 * observation.overlap;
        step_reward -= 15.0 * observation.obstacle_overlap;
        step_reward -= clearance_penalty;
        step_reward -= revisit_pen;
        step_reward -= 25.0; // Base step tax

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

MCTSNode* selectNode(MCTSNode* root, Environment& map){   // Select a node to expand based on the UCT score
    MCTSNode* current = root;   // Start from the root node
    
    while(!current->children.empty()){  // While the current node has children, continue traversing down the tree
        MCTSNode* next = current->getBestChild(map);   // Get the child node with the best UCT score
        if (next == nullptr){   // If there are no valid children, break the loop
            break;
        }
        current = next; // Move to the next node for further exploration
    }
    return current; // Return the selected node for expansion
}

int calculateHeuristic(Position start, Position goal){  // Calculate the Manhattan distance between the start and goal positions
    int dx = std::abs(start.x - goal.x);   // Calculate the absolute difference in the x-coordinates of the start and goal positions
    int dy = std::abs(start.y - goal.y);   // Calculate the absolute difference in the y-coordinates of the start and goal positions
    return 10 * std::max(dx, dy) + 4 * std::min(dx, dy); // Return the calculated Manhattan distance as the heuristic value
}

std::vector<Position> calculateAStar(Position start, Position goal, Environment& map, const TSDFMap& tsdf){
    std::vector<std::vector<int>> g_score(map.getHeight(), std::vector<int>(map.getWidth(), 9999999));    // Initialize g_score with a large value
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

        std::vector<Position> neighbors = map.getLegalMoves(current_x, current_y, true, tsdf);  // Get the legal moves (neighbors) from the current position
        for (const auto& neighbor : neighbors){ // Iterate through each neighbor of the current node
            int neighbor_x = neighbor.x;    // Get the x-coordinate of the neighbor
            int neighbor_y = neighbor.y;    // Get the y-coordinate of the neighbor

            int proximity_penalty = 0;   // Initialize a penalty for moving to a neighbor that is close to obstacles
            
            double tsdf_value = tsdf.getTSDFValue(neighbor_x, neighbor_y);   // Get the TSDF value for the neighbor position to assess proximity to obstacles

            if (tsdf_value == -1) continue; // If the TSDF value is -1, skip this neighbor as it is considered invalid or out of bounds

            if (tsdf_value < 3.0){
                proximity_penalty = static_cast<int>(std::round((3.0 - tsdf_value) * 20.0)); // Apply a penalty based on how close the neighbor is to obstacles (closer = higher penalty)
            }

            auto [gradient_x, gradient_y] = tsdf.getSafetyGradient(current_x, current_y);
            double move_length = std::hypot(neighbor_x - current_x, neighbor_y - current_y);
            double gradient_alignment = 0.0;
            if (move_length > 1e-6){
                gradient_alignment = ((neighbor_x - current_x) * gradient_x +
                                      (neighbor_y - current_y) * gradient_y) / move_length;
            }
            int gradient_penalty = static_cast<int>(std::round((1.0 - gradient_alignment) * 3.0));

            int move_cost = (neighbor_x != current_x && neighbor_y != current_y) ? 14 : 10; // Determine the cost to move to the neighbor (14 for diagonal moves, 10 for straight moves)

            int tentative_g_score = g_score[current_y][current_x] + proximity_penalty +
                                    gradient_penalty + move_cost;  // Calculate the tentative g_score for the neighbor (current g_score + movement, clearance, and safety-direction costs)

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

void runRootActionRollouts(MCTSNode& root, Environment& map, const TSDFMap& tsdf, std::mt19937& gen, int incoming_dx, int incoming_dy, int rollout_count = 1000){
    if (root.children.empty()){
        root.expandNode(map, tsdf);
    }

    for (int rollout = 0; rollout < rollout_count; ++rollout){
        MCTSNode* leaf = &root;

        // Selection: follow the existing tree until a leaf is reached.
        while (!leaf->children.empty()){
            MCTSNode* next = leaf->getBestChild(map);
            if (next == nullptr){
                break;
            }
            leaf = next;
        }

        // Expansion: create the leaf's possible successors once.
        if (leaf->children.empty()){
            leaf->expandNode(map, tsdf);
        }

        MCTSNode* simulation_node = leaf;
        if (!leaf->children.empty()){
            simulation_node = leaf->getBestChild(map);
        }

        if (simulation_node == nullptr){
            continue;
        }

        int action_dx = simulation_node->state.x - leaf->state.x;
        int action_dy = simulation_node->state.y - leaf->state.y;

        // Simulation: evaluate a hypothetical continuation from the expanded node.
        double reward = simulateActivePerception(
            simulation_node, map, 20, tsdf, gen, action_dx, action_dy);

        // Backpropagation: update every node from simulation_node to the root.
        simulation_node->backpropagate(reward);
    } 
}

int main(){
    // initialisation
    int width, height, start_x, start_y, ui_fov, camera_fov;
    int threshold = 7; 
    Position prev_pos = {-1, -1};

    if (!(std::cin >> width >> height >> start_x >> start_y >> ui_fov >> camera_fov) ) return 0;

    Environment true_map(width, height, 0, ui_fov, camera_fov);
    Environment known_map(width, height, -1, ui_fov, camera_fov);
    Position drone_pos ={start_x, start_y};
    TSDFMap tsdf(width, height, 5.0);

    int max_game_steps = 1000;

    // Load the true map from input
    for (int y = 0; y < height; ++y){
        for (int x = 0; x < width; ++x){
            int tile;
            std::cin >> tile;
            if (tile == 1){
                true_map.addObstacles(x, y);
            }
        }
    }

    std::cout << "MAP LOADED. STARTING LOOP...\n" << std::flush;

    // Initialize the very first FOV and frontiers before the loop starts
    // Bootstrap the first NDLF from the initial LIDAR observation.
    std::vector<Position> init_walls = known_map.updateLidarFOV(drone_pos.x, drone_pos.y, true_map.grid);
    tsdf.updateTSDF(init_walls);
    known_map.initializeCurrentNDLF(drone_pos);
    known_map.observeCameraFOV(drone_pos.x, drone_pos.y);
    tsdf.TSDFToVisualiser();
    known_map.rebuildFrontiers();
    // Stream the initial NDLF status before the first planning cycle.
    std::cout << "NDLF " << known_map.current_ndlf_id << " "
              << known_map.currentNDLFRemaining() << " ";
    for (const Position& cell : known_map.current_ndlf){
        std::cout << cell.x << " " << cell.y << " ";
    }
    std::cout << "\n" << std::flush;

    std::deque<Position> current_plan; // The multi-step execution queue
    std::vector<Position> escape_path; // Holds the remaining A* steps between planning phases
    std::unique_ptr<MCTSNode> mcts_root = nullptr; // Initialize the MCTS root node to nullptr
    bool mcts_failed = false;

    std::cout << drone_pos.x << " " << drone_pos.y << "\n" << std::flush;
    // game loop
    for (int game_step = 0; game_step < max_game_steps; game_step++){
        
        // Check if the environment is fully explored
        if (known_map.isFullyExplored(drone_pos.x, drone_pos.y, tsdf)){
            std::cout << "DONE\n" << std::flush;
            break;
        }

        // --- PHASE 1: PLANNING ---
        // Only run CPU-heavy tasks if the drone has no moves in its queue
        if (current_plan.empty()) {
            known_map.updateTargetLock(drone_pos.x, drone_pos.y);

            if (!escape_path.empty()){
                int dest_x = escape_path.back().x;
                int dest_y = escape_path.back().y;
                if (known_map.grid[dest_y][dest_x] == 1 || known_map.grid[dest_y][dest_x] == 2){
                    escape_path.clear(); // Clear the escape path if the destination is now blocked or already explored
                    std::cout << "ASTAR_PATH \n" << std::flush; // Erase finished leash in visualizer
                }
            }

            bool needs_transit = false;

            if (escape_path.empty() && known_map.currentNDLFRemaining() > 0){
                int closest_ndlf = 999999;
                for (const auto& cell : known_map.current_ndlf){
                    if (known_map.grid[cell.y][cell.x] == 0){
                        int distance = std::abs(drone_pos.x - cell.x) + std::abs(drone_pos.y - cell.y);
                        if (distance < closest_ndlf){
                            closest_ndlf = distance;
                        }
                    }
                }
                if (closest_ndlf > 5){
                    needs_transit = true; // The drone is too far from the current NDLF, so it needs to transit to it
                }
            }
            
            // Check if we need to trigger A* (surrounded, boredom, or continuing an existing escape path)
            if (known_map.currentNDLFRemaining() == 0 || needs_transit ||!escape_path.empty() || mcts_failed) {
                mcts_failed = false;
                // If we don't already have an active escape path, calculate one
                if (escape_path.empty()) {
                    bool path_found = false;

                    if (known_map.currentNDLFRemaining() > 0){
                        Position best_ndlf_target = {-1, -1};
                        int best_ndlf_distance = 99999;
                        for (const auto& cell : known_map.current_ndlf){
                            if (known_map.grid[cell.y][cell.x] == 0){
                                // This cell is in the current NDLF and has not been covered
                                int distance = std::abs(drone_pos.x - cell.x) + std::abs(drone_pos.y - cell.y);
                                if (distance < best_ndlf_distance){
                                    best_ndlf_distance = distance;
                                    best_ndlf_target = cell;
                                }
                            }
                        }
                        if (best_ndlf_target.x != -1){
                            escape_path = calculateAStar(drone_pos, best_ndlf_target, known_map, tsdf);
                            if (!escape_path.empty()){
                                path_found = true;
                            }
                        }
                    }

                    // Attempt to find a path to the nearest frontier using A*
                    if (!path_found) {
                        while (!known_map.active_frontiers.empty()) {
                            Position target_frontier_retry = known_map.getNearestFrontier(drone_pos);
                            if (target_frontier_retry.x == -1) break; // No valid frontiers found, exit the loop
                            
                            if (target_frontier_retry == drone_pos){
                                known_map.active_frontiers.erase(target_frontier_retry);
                                continue; // Skip to the next iteration if the target frontier is the same as the drone's current position
                            }
                            escape_path = calculateAStar(drone_pos, target_frontier_retry, known_map, tsdf);
                        
                            if (!escape_path.empty()){
                                path_found = true;
                                break; // Exit the loop if a valid escape path is found
                            } else {
                                known_map.active_frontiers.erase(target_frontier_retry);
                            }
                        }
                    }
                        
                    // Failsafe global scan if local frontiers fail
                    if (!path_found) {
                        known_map.scanGlobalFrontiers();
                        while (!known_map.active_frontiers.empty()) {
                            Position target_frontier_retry = known_map.getNearestFrontier(drone_pos);
                            if (target_frontier_retry.x == -1) break; // No valid frontiers found, exit the loop
                            
                            if (target_frontier_retry == drone_pos){
                                known_map.active_frontiers.erase(target_frontier_retry);
                                continue; // Skip to the next iteration if the target frontier is the same as the drone's current position
                            }
                            escape_path = calculateAStar(drone_pos, target_frontier_retry, known_map, tsdf);
                        
                            if (!escape_path.empty()){
                                path_found = true;
                                break; // Exit the loop if a valid escape path is found
                            } else {
                                known_map.active_frontiers.erase(target_frontier_retry);
                            }
                        }
                    }

                    if (!path_found || escape_path.empty()) {
                        std::cout << "STUCK\n" << std::flush;
                        break; // Exit the main loop if no escape path can be found
                    }
                    
                    std::cout << "[C++] GOING TO FRONTIER " << escape_path.back().x << ", " << escape_path.back().y << " \n" << std::flush;
                }

                current_plan.push_back(escape_path[0]);
                escape_path.erase(escape_path.begin());
                

                std::cout << "ASTAR_PATH ";
                for (const auto& pos : escape_path) {
                    std::cout << pos.x << " " << pos.y << " ";
                }
                std::cout << "\n" << std::flush;
                
            } else {
                // RUN PERSISTENT MCTS 
                mcts_root = std::make_unique<MCTSNode>(drone_pos);
                mcts_root->expandNode(known_map, tsdf);

                std::mt19937 sim_gen(std::random_device{}()); // Initialize a Mersenne Twister random number
                int curr_heading_x = (prev_pos.x != -1) ? (drone_pos.x - prev_pos.x) : 0; // Calculate the current heading in the x-direction based on the previous position
                int curr_heading_y = (prev_pos.y != -1) ? (drone_pos.y - prev_pos.y) : 0; // Calculate the current heading in the y-direction based
                runRootActionRollouts(*mcts_root, known_map, tsdf, sim_gen, curr_heading_x, curr_heading_y, 1000); // Run rollouts from the MCTS root node to evaluate potential moves and update the tree with rewards

                MCTSNode* next_node = mcts_root->getBestRealChild(known_map, tsdf, prev_pos);

                if (next_node != nullptr){ // && (known_map.visits[next_node->state.y][next_node->state.x] == 0)){
                    current_plan.push_back(next_node->state);   
                }


                // Visualizer Output: Print the projected MCTS path
                std::cout << "PATH ";
                for (const auto& step : current_plan){
                    std::cout << step.x << " " << step.y << " ";
                }
                std::cout << "\n" << std::flush;

                // Visualizer Output: Print active frontiers
                std::cout << "FRONTIERS ";
                for (const auto& frontier : known_map.active_frontiers){
                    std::cout << frontier.x << " " << frontier.y << " ";
                }
                std::cout << "\n" << std::flush;
            }
        }
        // --- PHASE 2: EXECUTION ---
        if (!current_plan.empty()) {
            Position next_step = current_plan.front();

            const auto legal_moves = known_map.getLegalMoves(drone_pos.x, drone_pos.y, true, tsdf);
            bool move_is_legal = std::find(legal_moves.begin(), legal_moves.end(), next_step) != legal_moves.end();
            if (!move_is_legal) {
                current_plan.clear(); // Trash the dangerous plan
                escape_path.clear();  // Also trash the remaining A* path if we were backtracking
                mcts_root.reset(); // Reset the MCTS root to force a new search in the next planning phase
                mcts_failed = true; // Set the flag to indicate that MCTS failed to find a valid move
                continue;             // Immediately loop back to Phase 1 and recalculate
            }

            current_plan.pop_front();
            
            prev_pos = drone_pos;
            drone_pos = next_step;

            mcts_root.reset(); // Reset the MCTS root if we are following an A* escape path to avoid conflicts            
            
            known_map.visits[drone_pos.y][drone_pos.x] += 1;
            
            // IMPORTANT: Update sensors IMMEDIATELY after taking the physical step
            // Update LIDAR first, then camera coverage, then advance the NDLF sequence if complete.
            std::vector<Position> new_walls = known_map.updateLidarFOV(drone_pos.x, drone_pos.y, true_map.grid);
            tsdf.updateTSDF(new_walls);
            known_map.observeCameraFOV(drone_pos.x, drone_pos.y);
            known_map.activateNextNDLF(drone_pos);
            // Stream the active NDLF so the visualizer can highlight the current pocket.
            std::cout << "NDLF " << known_map.current_ndlf_id << " "
                      << known_map.currentNDLFRemaining() << " ";
            for (const Position& cell : known_map.current_ndlf){
                std::cout << cell.x << " " << cell.y << " ";
            }
            std::cout << "\n" << std::flush;
            tsdf.TSDFToVisualiser();
            known_map.rebuildFrontiers();
            
            std::cout << drone_pos.x << " " << drone_pos.y << "\n" << std::flush;

            std::cout << "PATH ";
            for (const auto& step : current_plan){
                std::cout << step.x << " " << step.y << " ";
            }
            std::cout << "\n" << std::flush;

        } else {
            // If the queue is STILL empty after planning, we are truly stuck
            mcts_failed = true; // Set the flag to indicate that MCTS failed to find a valid move
            mcts_root.reset(); // Reset the MCTS root to force a new search in the next planning phase
            continue; // Skip the execution phase and loop back to planning in the next iteration
        }

        // Pause slightly to visualize the simulation (adjust timing as needed)
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // If the maximum number of game steps is reached
        if (game_step == max_game_steps - 1){
            std::cout << "STUCK\n" << std::flush;
        }
    }
    return 0;
}
