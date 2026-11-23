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
#include <deque>
#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <sstream>

// A simple 2D position struct
struct Position {
    int x;
    int y;

    bool operator<(const Position& other) const { // this is used for std::set ordering
        if (x == other.x) return y < other.y; // if x is equal, compare y, otherwise compare x, 
        return x < other.x;
    }
    bool operator==(const Position& other) const { // this is used for equality comparison
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position& other) const { // this is used for inequality comparison
        return !(*this == other);
    }
};

struct AStarNode {
    Position pos;
    int f_score;
};

struct CompareNodes {
    bool operator()(const AStarNode& a, const AStarNode& b) const {
        return a.f_score > b.f_score;
    }
};

class TSDFMap {
private:
    int width;
    int height;
    double max_dist;
    std::vector<double> tsdf_flat;

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

public:
    TSDFMap(int w, int h, double max_truncation_dist = 5.0)
        : width(w), height(h), max_dist(max_truncation_dist),
          tsdf_flat(w * h, max_truncation_dist) {
            using Element = std::pair<double, Position>;
            std::priority_queue<Element, std::vector<Element>, std::greater<Element>> pq;
            for (int x = 0; x < width; ++x) {
                tsdf_flat[0 * width + x] = 1.0;
                tsdf_flat[(height - 1) * width + x] = 1.0;
                pq.push({1.0, {x, 0}});
                pq.push({1.0, {x, height - 1}});
            }
            for (int y = 0; y < height; ++y) {
                tsdf_flat[y * width + 0] = 1.0;
                tsdf_flat[y * width + (width - 1)] = 1.0;
                pq.push({1.0, {0, y}});
                pq.push({1.0, {width - 1, y}});
            }
            while (!pq.empty()) {
                auto [dist, pos] = pq.top();
                pq.pop();
                int idx = pos.y * width + pos.x;
                if (dist > tsdf_flat[idx] || dist >= max_dist) continue;
                for (int i = 0; i < 8; ++i) {
                    int nx = pos.x + dx[i];
                    int ny = pos.y + dy[i];
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
                        double new_dist = dist + step_cost;
                        int n_idx = ny * width + nx;
                        if (new_dist < tsdf_flat[n_idx]) {
                            tsdf_flat[n_idx] = new_dist;
                            pq.push({new_dist, {nx, ny}});
                        }
                    }
                }
            }
          }

    inline double getTSDFValue(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return tsdf_flat[y * width + x];
        }
        return -1.0;
    }

    void updateTSDF(const std::vector<Position>& new_walls) {
        if (new_walls.empty()) return;

        using Element = std::pair<double, Position>;
        std::priority_queue<Element, std::vector<Element>, std::greater<Element>> pq;

        for (const auto& wall : new_walls) {
            if (wall.x >= 0 && wall.x < width && wall.y >= 0 && wall.y < height) {
                tsdf_flat[wall.y * width + wall.x] = 0.0;
                pq.push({0.0, wall});
            }
        }

        while (!pq.empty()) {
            auto [dist, pos] = pq.top();
            pq.pop();

            int idx = pos.y * width + pos.x;
            if (dist > tsdf_flat[idx]) continue;
            if (dist >= max_dist) continue;

            for (int i = 0; i < 8; ++i) {
                int nx = pos.x + dx[i];
                int ny = pos.y + dy[i];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
                    double new_dist = dist + step_cost;
                    int n_idx = ny * width + nx;

                    if (new_dist < tsdf_flat[n_idx]) {
                        tsdf_flat[n_idx] = new_dist;
                        pq.push({new_dist, {nx, ny}});
                    }
                }
            }
        }
    }

    inline double getGradientScore(int curr_x, int curr_y, int next_x, int next_y, double safe_clearance, double weight) const {
        double curr_tsdf = getTSDFValue(curr_x, curr_y);
        double next_tsdf = getTSDFValue(next_x, next_y);
        double min_tsdf = std::min(curr_tsdf, next_tsdf);

        if (min_tsdf < 0.0 || min_tsdf >= safe_clearance) {
            return 0.0;
        }

        double move_length = (curr_x != next_x && curr_y != next_y) ? 1.41421356 : 1.0;
        double tsdf_delta = (next_tsdf - curr_tsdf) / move_length;
        double urgency = safe_clearance - min_tsdf;
        return weight * urgency * tsdf_delta;
    }

    // Multiplicative discovery damping near walls: prevents edge-peeking from overpowering wall safety
    inline double getClearanceDiscoveryFactor(int x, int y, double safe_clearance) const {
        double c = getTSDFValue(x, y);
        if (c < 0.0 || c >= safe_clearance) return 1.0;
        double ratio = std::clamp(c / safe_clearance, 0.0, 1.0);
        return 0.25 + 0.75 * (ratio * ratio);
    }

    // Fixed Lateral Corridor Centering: uses exact 1-tile perpendicular offsets around both cardinal and diagonal moves
    inline double getLateralCenteringPenalty(int curr_x, int curr_y, int next_x, int next_y, double safe_clearance, double weight) const {
        int mdx = next_x - curr_x;
        int mdy = next_y - curr_y;
        if (mdx == 0 && mdy == 0) return 0.0;

        double next_t = getTSDFValue(next_x, next_y);
        if (next_t <= 0.0) return 0.0;

        double t_left_curr = 0.0, t_right_curr = 0.0;
        double t_left_next = 0.0, t_right_next = 0.0;

        if (mdx == 0 || mdy == 0) {
            // Cardinal move: perpendiculars are exact 1-tile steps (-mdy, mdx) and (mdy, -mdx)
            t_left_curr  = std::max(0.0, getTSDFValue(curr_x - mdy, curr_y + mdx));
            t_right_curr = std::max(0.0, getTSDFValue(curr_x + mdy, curr_y - mdx));
            t_left_next  = std::max(0.0, getTSDFValue(next_x - mdy, next_y + mdx));
            t_right_next = std::max(0.0, getTSDFValue(next_x + mdy, next_y - mdx));
        } else {
            // Diagonal move (mdx = +-1, mdy = +-1): the two 1-hop flank tiles shared by curr and next
            // are (curr_x + mdx, curr_y) and (curr_x, curr_y + mdy), plus 1-hop diagonal flanks from curr
            t_left_curr  = std::max(0.0, getTSDFValue(curr_x + mdx, curr_y));
            t_right_curr = std::max(0.0, getTSDFValue(curr_x, curr_y + mdy));
            t_left_next  = std::max(0.0, getTSDFValue(curr_x - mdx, curr_y + mdy));
            t_right_next = std::max(0.0, getTSDFValue(curr_x + mdx, curr_y - mdy));
        }

        double lateral_ridge = std::min(safe_clearance + 0.5, std::max({t_left_curr, t_right_curr, t_left_next, t_right_next}));
        double deadband_threshold = std::max(0.0, lateral_ridge - 0.20);
        if (next_t < deadband_threshold) {
            double off_center = deadband_threshold - next_t;
            return weight * off_center * off_center;
        }
        return 0.0;
    }

    void TSDFToVisualiser() const {
        std::cout << "TSDF ";
        for (int i = 0; i < width * height; ++i) {
            std::cout << std::fixed << std::setprecision(1) << tsdf_flat[i] << " ";
        }
        std::cout << "\n" << std::flush;
    }
};

// Kinematic Turn Smoothness Function:
// Evaluates both 1-step turn angle and 2-step rolling heading to prevent sawtooth zig-zagging and sharp turns
inline double computeKinematicSmoothness(
    int prev2_dx, int prev2_dy,
    int prev_dx, int prev_dy,
    int move_dx, int move_dy) {

    if (prev_dx == 0 && prev_dy == 0) return 0.0;

    double len1 = (prev_dx != 0 && prev_dy != 0) ? 1.41421356 : 1.0;
    double len2 = (move_dx != 0 && move_dy != 0) ? 1.41421356 : 1.0;
    double cos_sim = (prev_dx * move_dx + prev_dy * move_dy) / (len1 * len2);

    double score = 0.0;
    if (cos_sim > 0.95) {
        score += 220.0; // 0 deg: Pure straight flight
    } else if (cos_sim > 0.50) {
        score += 40.0;  // 45 deg: Smooth banking turn
    } else if (cos_sim > -0.10) {
        score -= 420.0; // 90 deg: Hard right-angle turn (heavily discouraged if 0/45 deg is available)
    } else if (cos_sim > -0.80) {
        score -= 850.0; // 135 deg: Sharp hairpin turn
    } else {
        score -= 1200.0; // 180 deg: Immediate reversal
    }

    // 2-step rolling heading check: stops Left-45 -> Right-45 -> Left-45 sawtooth wiggling
    // and blocks two consecutive 90-degree turns (U-turns)
    if (prev2_dx != 0 || prev2_dy != 0) {
        double len0 = (prev2_dx != 0 && prev2_dy != 0) ? 1.41421356 : 1.0;
        double cos_02 = (prev2_dx * move_dx + prev2_dy * move_dy) / (len0 * len2);

        int roll_x = prev2_dx + prev_dx;
        int roll_y = prev2_dy + prev_dy;
        double r_len = std::hypot(roll_x, roll_y);
        if (r_len > 1e-5) {
            double roll_cos = (roll_x * move_dx + roll_y * move_dy) / (r_len * len2);
            if (roll_cos > 0.90) {
                score += 120.0; // Consistently aligned with 2-step momentum
            } else if (roll_cos < 0.25) {
                score -= 320.0; // Penalize S-curve zig-zag or consecutive sharp turns
            }
        }
        // If step t-2 and step t point in opposite directions (< -0.5), penalize 2-step U-turn
        if (cos_02 < -0.50) {
            score -= 450.0;
        }
    }

    return score;
}

inline std::vector<Position> filterKinematicConeMoves(
    int curr_x, int curr_y,
    int prev_dx, int prev_dy,
    const std::vector<Position>& candidate_moves,
    const TSDFMap& tsdf,
    double safe_clearance) {

    if ((prev_dx == 0 && prev_dy == 0) || candidate_moves.size() <= 1) {
        return candidate_moves;
    }

    double len1 = (prev_dx != 0 && prev_dy != 0) ? 1.41421356 : 1.0;
    double curr_tsdf = tsdf.getTSDFValue(curr_x, curr_y);
    std::vector<Position> cone_45; // 0 deg and +-45 deg smooth flight cone
    std::vector<Position> cone_90; // <= 90 deg cornering cone
    bool has_viable_forward_move = false;

    for (const auto& m : candidate_moves) {
        int mdx = m.x - curr_x; // Move delta x
        int mdy = m.y - curr_y; // Move delta y
        double len2 = (mdx != 0 && mdy != 0) ? 1.41421356 : 1.0; // Move length
        double cos_sim = (prev_dx * mdx + prev_dy * mdy) / (len1 * len2); // Cosine similarity between previous move and candidate move, normalized by their lengths, ranges from -1 (opposite) to 1 (same direction), with 0 indicating perpendicular movement

        if (cos_sim > 0.50) { // Within +-45 deg of previous move
            cone_45.push_back(m); // Add to 45-degree cone
            double next_tsdf = tsdf.getTSDFValue(m.x, m.y);
            if (next_tsdf >= safe_clearance ||
                (next_tsdf > 0.8 && next_tsdf >= curr_tsdf - 0.05)) {
                has_viable_forward_move = true;
            }
        }
        if (cos_sim >= -0.05) { // Within +-90 deg of previous move
            cone_90.push_back(m); // Add to 90-degree cone
        }
    }

    // While cruising with open space ahead, strictly enforce +-45 deg maximum turn rate!
    if (has_viable_forward_move && !cone_45.empty()) {
        return cone_45;
    }
    // When forced to corner against a wall, allow up to 90 deg turn
    if (!cone_90.empty()) {
        return cone_90;
    }
    return candidate_moves;
}

class Environment {
private:
    int width;
    int height;
    int fov_radius;
    int camera_fov_radius;

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

public:
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getLidarFovRadius() const { return fov_radius; }
    int getCameraFovRadius() const { return camera_fov_radius; }

    std::vector<std::vector<int>> grid;
    std::vector<std::vector<int>> visits;
    std::vector<std::vector<int>> camera_observations;

    std::vector<int8_t> flat_grid;
    std::vector<int> flat_visits;
    std::vector<uint8_t> flat_known_cp;

    mutable std::vector<uint8_t> cell_cache_valid;
    mutable std::vector<std::vector<int>> lidar_vis_cache;
    mutable std::vector<int> lidar_obs_count_cache;
    mutable std::vector<std::vector<int>> cam_vis_cache;
    mutable std::vector<int> cam_obs_count_cache;
    mutable std::vector<std::vector<Position>> legal_moves_cache;

    std::set<Position> active_frontiers;
    std::set<Position> lidar_frontiers;
    std::set<Position> camera_frontiers;

    bool sar_enabled = false;
    bool sar_rtl = false;
    bool cp_enabled = false;
    bool cp_coords_known = false;
    bool cp_rtl = false;

    int total_sar_objects = 0;
    int total_cp_objects = 0;
    std::set<Position> discovered_sar;
    std::set<Position> discovered_cp;
    std::set<Position> known_uncollected_cp;

    std::vector<double> frontier_dist_flat; // Stores the distance to the nearest frontier for each cell in a flattened grid representation
    std::vector<double> checkpoint_dist_flat; // Stores the distance to the nearest checkpoint for each cell in a flattened grid representation

    Environment(int w, int h, int default_value, int fov, int camera_fov)
        : width(w), height(h), fov_radius(fov), camera_fov_radius(std::max(1, camera_fov)) {
        grid = std::vector<std::vector<int>>(height, std::vector<int>(width, default_value));
        visits = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));
        camera_observations = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));

        int N = w * h;
        flat_grid.assign(N, static_cast<int8_t>(default_value));
        flat_visits.assign(N, 0);
        flat_known_cp.assign(N, 0);
        cell_cache_valid.assign(N, 0);
        lidar_vis_cache.resize(N);
        lidar_obs_count_cache.assign(N, 0);
        cam_vis_cache.resize(N);
        cam_obs_count_cache.assign(N, 0);
        legal_moves_cache.resize(N);
        frontier_dist_flat.assign(N, 999999.0);
        checkpoint_dist_flat.assign(N, 999999.0);
    }

    static inline bool isOpaqueTile(int val) {
        return val == 1 || val == 3;
    }

    void setTile(int x, int y, int val) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            grid[y][x] = val;
        }
    }

    bool hasLineOfSight(int x0, int y0, int x1, int y1, const std::vector<std::vector<int>>& map_data) const {
        int d_x = std::abs(x1 - x0);
        int d_y = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = d_x - d_y;

        while (true) {
            if (x0 == x1 && y0 == y1) return true;
            if ((x0 != x1 || y0 != y1) && isOpaqueTile(map_data[y0][x0])) return false;

            int e2 = 2 * err;
            int prev_x = x0;
            int prev_y = y0;

            if (e2 > -d_y) { err -= d_y; x0 += sx; }
            if (e2 < d_x) { err += d_x; y0 += sy; }
            if (prev_x != x0 && prev_y != y0) {
                if (isOpaqueTile(map_data[prev_y][x0]) || isOpaqueTile(map_data[y0][prev_x])) {
                    return false;
                }
            }
        }
        return true;
    }

    bool hasCameraLineOfSight(int x0, int y0, int x1, int y1) const {
        int delta_x = std::abs(x1 - x0);
        int delta_y = std::abs(y1 - y0);
        int step_x = (x0 < x1) ? 1 : -1;
        int step_y = (y0 < y1) ? 1 : -1;
        int error = delta_x - delta_y;

        while (true) {
            if (x0 == x1 && y0 == y1) return true;
            if ((x0 != x1 || y0 != y1) && (grid[y0][x0] == -1 || isOpaqueTile(grid[y0][x0]))) return false;
            int doubled_error = 2 * error;
            int previous_x = x0;
            int previous_y = y0;
            if (doubled_error > -delta_y) { error -= delta_y; x0 += step_x; }
            if (doubled_error < delta_x) { error += delta_x; y0 += step_y; }
            if (previous_x != x0 && previous_y != y0 &&
                (grid[previous_y][x0] == -1 || grid[y0][previous_x] == -1 ||
                 isOpaqueTile(grid[previous_y][x0]) || isOpaqueTile(grid[y0][previous_x]))) {
                return false;
            }
        }
    }

    std::vector<Position> updateLidarFOV(int drone_x, int drone_y, const std::vector<std::vector<int>>& true_map) {
        std::vector<Position> new_walls;
        if (grid[drone_y][drone_x] == -1) {
            grid[drone_y][drone_x] = 0;
            cell_cache_valid[drone_y * width + drone_x] = 0;
        }
        int radius = fov_radius;
        int radius_squared = radius * radius;

        for (int dy_off = -radius; dy_off <= radius; ++dy_off) {
            for (int dx_off = -radius; dx_off <= radius; ++dx_off) {
                if (dx_off * dx_off + dy_off * dy_off > radius_squared) continue;

                int view_x = drone_x + dx_off;
                int view_y = drone_y + dy_off;

                if (view_x >= 0 && view_x < width && view_y >= 0 && view_y < height) {
                    if (hasLineOfSight(drone_x, drone_y, view_x, view_y, true_map)) {
                        int true_tile = true_map[view_y][view_x];
                        if (isOpaqueTile(true_tile)) {
                            if (grid[view_y][view_x] == -1) {
                                new_walls.push_back({view_x, view_y});
                            }
                            grid[view_y][view_x] = 1;
                        } else if (grid[view_y][view_x] == -1) {
                            grid[view_y][view_x] = 0;
                            for (int i = 0; i < 8; ++i) {
                                int nx = view_x + dx[i], ny = view_y + dy[i];
                                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                    cell_cache_valid[ny * width + nx] = 0;
                                }
                            }
                        }
                    }
                }
            }
        }
        return new_walls;
    }

    inline bool isSafe(int x, int y, bool require_known = false) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        if (isOpaqueTile(grid[y][x])) return false;
        if (require_known && grid[y][x] == -1) return false;
        return true;
    }

    inline void ensureCellCached(int x, int y, const TSDFMap& tsdf) const {
        int idx = y * width + x;
        if (cell_cache_valid[idx]) return;

        legal_moves_cache[idx].clear();
        lidar_vis_cache[idx].clear();
        lidar_obs_count_cache[idx] = 0;
        cam_vis_cache[idx].clear();
        cam_obs_count_cache[idx] = 0;

        if (!isSafe(x, y, true)) {
            cell_cache_valid[idx] = 1;
            return;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (!isSafe(nx, ny, true)) continue;
            double nb_t = tsdf.getTSDFValue(nx, ny);
            if (nb_t <= 0.05) continue;
            if (dx[i] != 0 && dy[i] != 0) {
                if (isOpaqueTile(grid[y][nx]) || isOpaqueTile(grid[ny][x])) continue;
                if (!isSafe(nx, y, true) || !isSafe(x, ny, true)) continue;
            }
            legal_moves_cache[idx].push_back({nx, ny});
        }

        int l_rad = fov_radius;
        int l_rad_sq = l_rad * l_rad;
        for (int dy_off = -l_rad; dy_off <= l_rad; ++dy_off) {
            for (int dx_off = -l_rad; dx_off <= l_rad; ++dx_off) {
                if (dx_off * dx_off + dy_off * dy_off > l_rad_sq) continue;
                int vx = x + dx_off;
                int vy = y + dy_off;
                if (vx >= 0 && vx < width && vy >= 0 && vy < height) {
                    if (hasLineOfSight(x, y, vx, vy, grid)) {
                        if (isOpaqueTile(grid[vy][vx])) {
                            lidar_obs_count_cache[idx]++;
                        } else {
                            lidar_vis_cache[idx].push_back(vy * width + vx);
                        }
                    }
                }
            }
        }

        int c_rad = camera_fov_radius;
        int c_rad_sq = c_rad * c_rad;
        for (int dy_off = -c_rad; dy_off <= c_rad; ++dy_off) {
            for (int dx_off = -c_rad; dx_off <= c_rad; ++dx_off) {
                if (dx_off * dx_off + dy_off * dy_off > c_rad_sq) continue;
                int vx = x + dx_off;
                int vy = y + dy_off;
                if (vx >= 0 && vx < width && vy >= 0 && vy < height) {
                    if (hasLineOfSight(x, y, vx, vy, grid)) {
                        if (isOpaqueTile(grid[vy][vx])) {
                            cam_obs_count_cache[idx]++;
                        } else {
                            cam_vis_cache[idx].push_back(vy * width + vx);
                        }
                    }
                }
            }
        }

        cell_cache_valid[idx] = 1;
    }

    void invalidateAllCellCaches() {
        std::fill(cell_cache_valid.begin(), cell_cache_valid.end(), 0);
    }

    std::vector<Position> getLegalMoves(int current_x, int current_y, bool require_known, const TSDFMap& tsdf) const {
        if (require_known && current_x >= 0 && current_x < width && current_y >= 0 && current_y < height) {
            ensureCellCached(current_x, current_y, tsdf);
            return legal_moves_cache[current_y * width + current_x];
        }

        std::vector<Position> legal_moves;
        for (int i = 0; i < 8; ++i) {
            int next_x = current_x + dx[i];
            int next_y = current_y + dy[i];

            if (!isSafe(next_x, next_y, require_known)) continue;
            if (tsdf.getTSDFValue(next_x, next_y) <= 0.05) continue;

            if (dx[i] != 0 && dy[i] != 0) {
                if (isOpaqueTile(grid[current_y][next_x]) || isOpaqueTile(grid[next_y][current_x])) continue;
                if (require_known && (!isSafe(next_x, current_y, true) || !isSafe(current_x, next_y, true))) continue;
            }
            legal_moves.push_back({next_x, next_y});
        }
        return legal_moves;
    }

    struct SensorObservation {
        int newly_covered = 0;
        int overlap = 0;
        int obstacle_overlap = 0;
        int known_cp_covered = 0;
    };

    SensorObservation estimateLidarFOV(int drone_x, int drone_y, const TSDFMap& tsdf) const {
        SensorObservation obs;
        ensureCellCached(drone_x, drone_y, tsdf);
        int cell = drone_y * width + drone_x;
        obs.obstacle_overlap = lidar_obs_count_cache[cell];
        for (int v_idx : lidar_vis_cache[cell]) {
            int8_t g = flat_grid[v_idx];
            if (g == -1) obs.newly_covered++;
            else obs.overlap++;
        }
        return obs;
    }

    SensorObservation estimateCameraFOV(int drone_x, int drone_y, const TSDFMap& tsdf) const {
        SensorObservation obs;
        ensureCellCached(drone_x, drone_y, tsdf);
        int cell = drone_y * width + drone_x;
        obs.obstacle_overlap = cam_obs_count_cache[cell];
        for (int v_idx : cam_vis_cache[cell]) {
            int8_t g = flat_grid[v_idx];
            if (g == 0) obs.newly_covered++;
            else if (g == 2) obs.overlap++;
            if (cp_coords_known && flat_known_cp[v_idx]) obs.known_cp_covered++;
        }
        return obs;
    }

    void observePhysicalCameraFOV(int drone_x, int drone_y, const std::vector<std::vector<int>>& true_map) {
        int radius = camera_fov_radius;
        int radius_squared = radius * radius;

        for (int dy_off = -radius; dy_off <= radius; ++dy_off) {
            for (int dx_off = -radius; dx_off <= radius; ++dx_off) {
                if (dx_off * dx_off + dy_off * dy_off > radius_squared) continue;

                int view_x = drone_x + dx_off;
                int view_y = drone_y + dy_off;

                if (view_x >= 0 && view_x < width && view_y >= 0 && view_y < height) {
                    if (hasCameraLineOfSight(drone_x, drone_y, view_x, view_y)) {
                        int true_tile = true_map[view_y][view_x];
                        if (true_tile == 3 && sar_enabled) {
                            if (discovered_sar.insert({view_x, view_y}).second) {
                                std::cout << "FOUND_SAR " << view_x << " " << view_y << "\n" << std::flush;
                            }
                        } else if (true_tile == 4 && cp_enabled) {
                            if (discovered_cp.insert({view_x, view_y}).second) {
                                known_uncollected_cp.erase({view_x, view_y});
                                std::cout << "FOUND_CP " << view_x << " " << view_y << "\n" << std::flush;
                            }
                        }

                        if (!isOpaqueTile(grid[view_y][view_x])) {
                            camera_observations[view_y][view_x]++;
                            if (grid[view_y][view_x] == 0) {
                                grid[view_y][view_x] = 2;
                            }
                        }
                    }
                }
            }
        }
    }

    void syncFlatArrays() {
        int N = width * height;
        flat_known_cp.assign(N, 0);
        for (const auto& cp : known_uncollected_cp) {
            if (cp.x >= 0 && cp.x < width && cp.y >= 0 && cp.y < height) {
                flat_known_cp[cp.y * width + cp.x] = 1;
            }
        }
        for (int y = 0; y < height; ++y) {
            int row_off = y * width;
            for (int x = 0; x < width; ++x) {
                flat_grid[row_off + x] = static_cast<int8_t>(grid[y][x]);
                flat_visits[row_off + x] = visits[y][x];
            }
        }
    }

    void rebuildFrontiers(const TSDFMap& tsdf, const std::vector<std::vector<int>>* true_map_ptr = nullptr) {
        lidar_frontiers.clear();
        camera_frontiers.clear();

        int N = width * height;
        frontier_dist_flat.assign(N, 999999.0);
        using QElem = std::pair<double, int>;
        std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> pq;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                if (grid[y][x] == 0) {
                    frontier_dist_flat[idx] = 0.0;
                    pq.push({0.0, idx});

                    bool next_to_unseen = false;
                    bool next_to_camera = false;
                    for (int direction = 0; direction < 4; ++direction) {
                        int nx = x + dx[direction * 2];
                        int ny = y + dy[direction * 2];
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                        next_to_unseen = next_to_unseen || grid[ny][nx] == -1;
                        next_to_camera = next_to_camera || grid[ny][nx] == 2;
                    }
                    if (next_to_unseen) lidar_frontiers.insert({x, y});
                    if (next_to_camera) camera_frontiers.insert({x, y});
                } else if (grid[y][x] == 2) {
                    for (int direction = 0; direction < 4; ++direction) {
                        int nx = x + dx[direction * 2];
                        int ny = y + dy[direction * 2];
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
                        bool unseen_neighbor = (grid[ny][nx] == -1);
                        bool uninspected_sar_neighbor = (sar_enabled && true_map_ptr != nullptr &&
                                                         (*true_map_ptr)[ny][nx] == 3 &&
                                                         grid[ny][nx] == 1 &&
                                                         discovered_sar.count({nx, ny}) == 0);
                        if (unseen_neighbor || uninspected_sar_neighbor) {
                            lidar_frontiers.insert({x, y});
                            frontier_dist_flat[idx] = 0.0;
                            pq.push({0.0, idx});
                            break;
                        }
                    }
                }
            }
        }
        active_frontiers = camera_frontiers;
        for (const auto& lf : lidar_frontiers) {
            active_frontiers.insert(lf);
        }

        double safe_clearance = std::max(1.0, static_cast<double>(camera_fov_radius) - 1.0);

        while (!pq.empty()) {
            auto [d, idx] = pq.top();
            pq.pop();
            if (d > frontier_dist_flat[idx]) continue;

            int px = idx % width;
            int py = idx / width;

            for (int i = 0; i < 8; ++i) {
                int nx = px + dx[i];
                int ny = py + dy[i];
                if (!isSafe(nx, ny, true)) continue;
                if (dx[i] != 0 && dy[i] != 0) {
                    if (isOpaqueTile(grid[py][nx]) || isOpaqueTile(grid[ny][px])) continue;
                }
                double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
                double clearance = tsdf.getTSDFValue(nx, ny);
                if (clearance > 0.0 && clearance < safe_clearance) {
                    // Only penalize off-center tiles in wide passages; do not inflate 1-tile bottlenecks!
                    double best_local_c = clearance;
                    for (int k = 0; k < 8; ++k) {
                        int tx = px + dx[k], ty = py + dy[k];
                        if (isSafe(tx, ty, true)) {
                            best_local_c = std::max(best_local_c, tsdf.getTSDFValue(tx, ty));
                        }
                    }
                    double ridge_c = std::min(safe_clearance, best_local_c);
                    if (clearance < ridge_c) {
                        step_cost += 2.2 * (ridge_c - clearance);
                    }
                }
                int n_idx = ny * width + nx;
                if (d + step_cost < frontier_dist_flat[n_idx]) {
                    frontier_dist_flat[n_idx] = d + step_cost;
                    pq.push({d + step_cost, n_idx});
                }
            }
        }

        checkpoint_dist_flat.assign(N, 999999.0);
        if (cp_enabled && cp_coords_known && !known_uncollected_cp.empty()) {
            std::priority_queue<QElem, std::vector<QElem>, std::greater<QElem>> cp_pq;
            for (const auto& cp : known_uncollected_cp) {
                int c_idx = cp.y * width + cp.x;
                checkpoint_dist_flat[c_idx] = 0.0;
                cp_pq.push({0.0, c_idx});
            }

            while (!cp_pq.empty()) {
                auto [d, idx] = cp_pq.top();
                cp_pq.pop();
                if (d > checkpoint_dist_flat[idx]) continue;

                int px = idx % width;
                int py = idx / width;

                for (int i = 0; i < 8; ++i) {
                    int nx = px + dx[i];
                    int ny = py + dy[i];
                    if (!isSafe(nx, ny, true)) continue;
                    if (dx[i] != 0 && dy[i] != 0) {
                        if (isOpaqueTile(grid[py][nx]) || isOpaqueTile(grid[ny][px])) continue;
                    }
                    double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
                    double clearance = tsdf.getTSDFValue(nx, ny);
                    if (clearance > 0.0 && clearance < safe_clearance) {
                        step_cost += 2.2 * (safe_clearance - clearance);
                    }
                    int n_idx = ny * width + nx;
                    if (d + step_cost < checkpoint_dist_flat[n_idx]) {
                        checkpoint_dist_flat[n_idx] = d + step_cost;
                        cp_pq.push({d + step_cost, n_idx});
                    }
                }
            }
        }

        syncFlatArrays();
    }

    inline double getDistanceToNearestUncovered(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return frontier_dist_flat[y * width + x];
        }
        return 999999.0;
    }

    inline double getDistanceToNearestCheckpoint(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return checkpoint_dist_flat[y * width + x];
        }
        return 999999.0;
    }

    inline double bias(int curr_x, int curr_y, int next_x, int next_y, double weight = 1400.0) const {
        if (!cp_enabled || !cp_coords_known || known_uncollected_cp.empty()) {
            return 0.0;
        }
        double d_curr = getDistanceToNearestCheckpoint(curr_x, curr_y);
        double d_next = getDistanceToNearestCheckpoint(next_x, next_y);
        if (d_curr >= 999990.0 || d_next >= 999990.0) {
            return 0.0;
        }
        return weight * (d_curr - d_next);
    }

    bool shouldTriggerRTL(const std::vector<Position>& drone_positions, const TSDFMap& /*tsdf*/) const {
        bool any_rtl_objective_active = (sar_enabled && sar_rtl && total_sar_objects > 0) ||
                                        (cp_enabled && cp_rtl && total_cp_objects > 0);

        if (any_rtl_objective_active) {
            bool sar_done = !(sar_enabled && sar_rtl && total_sar_objects > 0) ||
                            (static_cast<int>(discovered_sar.size()) >= total_sar_objects);
            bool cp_done = !(cp_enabled && cp_rtl && total_cp_objects > 0) ||
                           (static_cast<int>(discovered_cp.size()) >= total_cp_objects);

            if (sar_done && cp_done) {
                return true;
            }
        }

        if (!active_frontiers.empty()) {
            return false;
        }

        for (const auto& dp : drone_positions) {
            if (getDistanceToNearestUncovered(dp.x, dp.y) < 999990.0) {
                return false;
            }
        }
        return true;
    }
};

// ============================================================================
// MODULE 1: Search Tree Representation & D-UCT Policy
// ============================================================================
struct MCTSNode {
    double visits;
    double score;
    double heuristic_bonus;
    Position state;

    MCTSNode* parent;
    std::vector<std::unique_ptr<MCTSNode>> children;

    std::vector<Position> best_rollout_path;
    double best_rollout_score = -std::numeric_limits<double>::infinity();

    MCTSNode(Position current_pos, MCTSNode* parent_node = nullptr, double h_bonus = 0.0)
        : visits(0.0), score(0.0), heuristic_bonus(h_bonus), state(current_pos), parent(parent_node) {}

    void trimRolloutPrefix(Position new_root_pos) {
        if (!best_rollout_path.empty() && best_rollout_path.front() == new_root_pos) {
            best_rollout_path.erase(best_rollout_path.begin());
        }
        best_rollout_score = -std::numeric_limits<double>::infinity();
        for (auto& ch : children) {
            ch->trimRolloutPrefix(new_root_pos);
        }
    }

    std::unique_ptr<MCTSNode> extractChild(Position next_pos) {
        for (auto it = children.begin(); it != children.end(); ++it) {
            if ((*it)->state == next_pos) {
                std::unique_ptr<MCTSNode> new_root = std::move(*it);
                children.erase(it);
                new_root->parent = nullptr;
                new_root->trimRolloutPrefix(next_pos);
                return new_root;
            }
        }
        return nullptr;
    }

    void pruneInvalidNodes(const Environment& map, const TSDFMap& tsdf) {
        std::vector<Position> valid_moves = map.getLegalMoves(state.x, state.y, true, tsdf);

        for (auto it = children.begin(); it != children.end();) {
            Position child_pos = (*it)->state;
            bool is_still_legal = std::find(valid_moves.begin(), valid_moves.end(), child_pos) != valid_moves.end();
            double child_tsdf = tsdf.getTSDFValue(child_pos.x, child_pos.y);

            if (!is_still_legal || map.grid[child_pos.y][child_pos.x] == 1 || child_tsdf <= 0.05) {
                it = children.erase(it);
            } else {
                (*it)->pruneInvalidNodes(map, tsdf);
                ++it;
            }
        }
    }

    inline double getDiscountedAverage() const {
        if (visits <= 1e-6) return -std::numeric_limits<double>::infinity();
        return score / visits;
    }

    inline double getDUCTScore(double Cp = 650.0) const {
        if (visits < 0.0001) {
            return std::numeric_limits<double>::infinity();
        }
        double t_id_gamma = (parent != nullptr) ? std::max(1.0, parent->visits) : std::max(1.0, visits);
        double f_bar = score / visits;
        double c_explore = 2.0 * Cp * std::sqrt(std::log(t_id_gamma + 1.0) / visits);
        double decaying_heuristic = heuristic_bonus / (1.0 + 0.5 * visits);
        return f_bar + c_explore + decaying_heuristic;
    }

    // Balanced D-UCT discount factor gamma_duct = 0.997 so 600 rollouts/turn don't wipe out tree memory
    void backpropagateDUCT(double F_t, const std::vector<Position>& full_rollout_seq, double gamma_duct = 0.997) {
        if (F_t > best_rollout_score || best_rollout_path.size() < full_rollout_seq.size()) {
            best_rollout_score = F_t;
            best_rollout_path = full_rollout_seq;
        }

        MCTSNode* current = this;
        while (current != nullptr) {
            if (current->parent != nullptr) {
                for (auto& sibling : current->parent->children) {
                    sibling->visits *= gamma_duct;
                    sibling->score *= gamma_duct;
                }
                current->visits += 1.0;
                current->score += F_t;

                if (F_t > current->best_rollout_score || current->best_rollout_path.size() < full_rollout_seq.size()) {
                    current->best_rollout_score = F_t;
                    current->best_rollout_path = full_rollout_seq;
                }
            } else {
                double sum_v = 0.0;
                double sum_s = 0.0;
                for (const auto& child : current->children) {
                    sum_v += child->visits;
                    sum_s += child->score;
                }
                current->visits = sum_v;
                current->score = sum_s;
            }
            current = current->parent;
        }
    }

    MCTSNode* getBestChildDUCT(const Environment& map, const TSDFMap& tsdf) {
        MCTSNode* best_node = nullptr;
        double best_score = -std::numeric_limits<double>::infinity();

        for (const auto& child : children) {
            const Position& position = child->state;
            if (map.grid[position.y][position.x] == 1 || tsdf.getTSDFValue(position.x, position.y) <= 0.05) {
                continue;
            }
            double s = child->getDUCTScore();
            if (s > best_score) {
                best_score = s;
                best_node = child.get();
            }
        }
        return best_node;
    }

    double computeChildHeuristicBonus(
        Position move,
        const Environment& map,
        const TSDFMap& tsdf,
        int in2_dx, int in2_dy,
        int in_dx, int in_dy) const {
        double parent_frontier_dist = map.getDistanceToNearestUncovered(state.x, state.y);
        double safe_clearance = std::max(1.0, static_cast<double>(map.getCameraFovRadius()) - 1.0);
        int mdx = move.x - state.x;
        int mdy = move.y - state.y;
        double move_frontier_dist = map.getDistanceToNearestUncovered(move.x, move.y);
        double turn_smoothness = computeKinematicSmoothness(in2_dx, in2_dy, in_dx, in_dy, mdx, mdy);
        double lat_pen = tsdf.getLateralCenteringPenalty(state.x, state.y, move.x, move.y, safe_clearance, 450.0);
        double move_tsdf = tsdf.getTSDFValue(move.x, move.y);
        double tsdf_pen = 0.0;

        if (move_tsdf < safe_clearance) {
            double deficit = safe_clearance - move_tsdf;
            double narrow_scale = (lat_pen <= 1e-3) ? 0.6 : 1.0;
            tsdf_pen = 1250.0 * narrow_scale * deficit * deficit;
        }
        if (turn_smoothness > 0.0 && move_tsdf < safe_clearance) {
            turn_smoothness *= (move_tsdf / safe_clearance);
        }

        return 350.0 * (parent_frontier_dist - move_frontier_dist)
             + map.bias(state.x, state.y, move.x, move.y, 450.0)
             + tsdf.getGradientScore(state.x, state.y, move.x, move.y, safe_clearance, 300.0)
             + 1.5 * turn_smoothness
             - lat_pen
             - tsdf_pen;
    }

    // Tree expansion passes both root_prev_pos and root_prev2_pos and avoids backtracking into prev/prev2
    void expandNode(const Environment& map, const TSDFMap& tsdf, Position root_prev_pos = {-1, -1}, Position root_prev2_pos = {-1, -1}) {
        std::vector<Position> all_moves = map.getLegalMoves(state.x, state.y, true, tsdf);
        if (all_moves.empty()) return;

        Position immediate_back = (parent != nullptr) ? parent->state : root_prev_pos;
        Position two_step_back = (parent != nullptr)
            ? ((parent->parent != nullptr) ? parent->parent->state : root_prev_pos)
            : root_prev2_pos;

        std::vector<Position> filtered_moves;
        for (const auto& m : all_moves) {
            if (m != immediate_back && m != two_step_back) {
                filtered_moves.push_back(m);
            }
        }
        if (filtered_moves.empty()) {
            for (const auto& m : all_moves) {
                if (m != immediate_back) filtered_moves.push_back(m);
            }
        }
        if (filtered_moves.empty()) {
            return;
            //filtered_moves = all_moves;
        }

        double safe_clearance = std::max(1.0, static_cast<double>(map.getCameraFovRadius()) - 1.0);

        int in_dx = 0, in_dy = 0;
        int in2_dx = 0, in2_dy = 0;
        if (parent != nullptr) {
            in_dx = state.x - parent->state.x;
            in_dy = state.y - parent->state.y;
            if (parent->parent != nullptr) {
                in2_dx = parent->state.x - parent->parent->state.x;
                in2_dy = parent->state.y - parent->parent->state.y;
            } else if (root_prev_pos.x != -1) {
                in2_dx = parent->state.x - root_prev_pos.x;
                in2_dy = parent->state.y - root_prev_pos.y;
            }
        } else if (root_prev_pos.x != -1) {
            in_dx = state.x - root_prev_pos.x;
            in_dy = state.y - root_prev_pos.y;
            if (root_prev2_pos.x != -1) {
                in2_dx = root_prev_pos.x - root_prev2_pos.x;
                in2_dy = root_prev_pos.y - root_prev2_pos.y;
            }
        }

        filtered_moves = filterKinematicConeMoves(state.x, state.y, in_dx, in_dy, filtered_moves, tsdf, safe_clearance);

        for (const auto& move : filtered_moves) {
            double h_bonus = computeChildHeuristicBonus(move, map, tsdf, in2_dx, in2_dy, in_dx, in_dy);
            children.push_back(std::make_unique<MCTSNode>(move, this, h_bonus));
        }
    }

    void refreshRootForCurrentHeading(
        const Environment& map,
        const TSDFMap& tsdf,
        Position root_prev_pos,
        Position root_prev2_pos) {
        std::vector<Position> all_moves = map.getLegalMoves(state.x, state.y, true, tsdf);
        if (all_moves.empty()) return;

        int in_dx = (root_prev_pos.x != -1) ? state.x - root_prev_pos.x : 0;
        int in_dy = (root_prev_pos.y != -1) ? state.y - root_prev_pos.y : 0;
        int in2_dx = (root_prev2_pos.x != -1 && root_prev_pos.x != -1)
            ? root_prev_pos.x - root_prev2_pos.x : 0;
        int in2_dy = (root_prev2_pos.y != -1 && root_prev_pos.y != -1)
            ? root_prev_pos.y - root_prev2_pos.y : 0;
        double len1 = (in_dx != 0 && in_dy != 0) ? 1.41421356 : 1.0;

        bool has_non_reversal = false;
        for (const auto& move : all_moves) {
            if (move == root_prev_pos || move == root_prev2_pos) continue;
            if (in_dx == 0 && in_dy == 0) {
                has_non_reversal = true;
                break;
            }
            int mdx = move.x - state.x;
            int mdy = move.y - state.y;
            double len2 = (mdx != 0 && mdy != 0) ? 1.41421356 : 1.0;
            if ((in_dx * mdx + in_dy * mdy) / (len1 * len2) >= -0.10) {
                has_non_reversal = true;
                break;
            }
        }

        if (has_non_reversal) {
            for (auto it = children.begin(); it != children.end();) {
                Position child_pos = (*it)->state;
                int mdx = child_pos.x - state.x;
                int mdy = child_pos.y - state.y;
                double len2 = (mdx != 0 && mdy != 0) ? 1.41421356 : 1.0;
                double cos_sim = (in_dx == 0 && in_dy == 0)
                    ? 1.0
                    : (in_dx * mdx + in_dy * mdy) / (len1 * len2);
                if (child_pos == root_prev_pos || child_pos == root_prev2_pos || cos_sim < -0.10) {
                    it = children.erase(it);
                } else {
                    ++it;
                }
            }
        }

        std::vector<Position> candidates;
        for (const auto& move : all_moves) {
            if (!has_non_reversal || (move != root_prev_pos && move != root_prev2_pos)) {
                candidates.push_back(move);
            }
        }
        double safe_clearance = std::max(1.0, static_cast<double>(map.getCameraFovRadius()) - 1.0);
        std::vector<Position> cone_moves = filterKinematicConeMoves(
            state.x, state.y, in_dx, in_dy, candidates, tsdf, safe_clearance);

        for (auto& child : children) {
            child->heuristic_bonus = computeChildHeuristicBonus(
                child->state, map, tsdf, in2_dx, in2_dy, in_dx, in_dy);
        }
        for (const auto& move : cone_moves) {
            auto existing = std::find_if(children.begin(), children.end(),
                [&](const std::unique_ptr<MCTSNode>& child) { return child->state == move; });
            if (existing == children.end()) {
                children.push_back(std::make_unique<MCTSNode>(
                    move, this, computeChildHeuristicBonus(move, map, tsdf, in2_dx, in2_dy, in_dx, in_dy)));
            }
        }
    }
};

struct CandidateSequence {
    std::vector<Position> path;
    double f_bar = 0.0;
    double prob = 0.0;
    double cond_utility = 0.0;

    std::vector<int> lidar_footprint;
    std::vector<int> cam_footprint;

    void precomputeFootprints(const Environment& map, const TSDFMap& tsdf) {
        int W = map.getWidth();
        lidar_footprint.clear();
        cam_footprint.clear();
        for (const auto& p : path) {
            map.ensureCellCached(p.x, p.y, tsdf);
            int cell = p.y * W + p.x;
            for (int v : map.lidar_vis_cache[cell]) lidar_footprint.push_back(v);
            for (int v : map.cam_vis_cache[cell]) cam_footprint.push_back(v);
        }
    }
};

struct DroneAgent {
        int id;
        Position home_pos;
        Position drone_pos;
        Position prev_pos = {-1, -1};
        Position prev2_pos = {-1, -1};

        std::vector<int> flat_own_visits;
        std::unique_ptr<MCTSNode> mcts_root = nullptr;
        std::deque<Position> current_plan;
        std::vector<Position> best_full_plan;
        std::vector<Position> rtl_path;
        bool rtl_active = false;
        bool pending_hover_at_path_end = false;

        std::vector<CandidateSequence> sparse_domain;
        CandidateSequence committed_sequence;
        double beta = 400.0;
        double winning_expected_utility = 0.0;

        DroneAgent(int drone_id, Position start, int w, int h)
                : id(drone_id), home_pos(start), drone_pos(start),
                    flat_own_visits(w * h, 0) {
                flat_own_visits[start.y * w + start.x] = 1;
        }
};

struct TeammateMask {
    std::vector<int8_t> masked_grid;
    std::vector<uint8_t> masked_known_cp;
    std::vector<int> teammate_traj_visits;
    std::vector<std::vector<Position>> teammate_positions;
    std::vector<Position> teammate_current_roots;

    TeammateMask(const Environment& base_map, int max_steps)
        : masked_grid(base_map.flat_grid),
          masked_known_cp(base_map.flat_known_cp),
          teammate_traj_visits(base_map.getWidth() * base_map.getHeight(), 0),
          teammate_positions(max_steps) {}
};

TeammateMask buildTeammateMaskFast(
    const Environment& base_map,
    const std::vector<const CandidateSequence*>& other_seqs,
    const std::vector<Position>& other_roots,
    int max_steps) {

    TeammateMask mask(base_map, max_steps);
    mask.teammate_current_roots = other_roots;
    int W = base_map.getWidth();
    int H = base_map.getHeight();

    for (size_t idx = 0; idx < other_seqs.size(); ++idx) {
        const CandidateSequence* seq = other_seqs[idx];
        Position last_pos = (idx < other_roots.size()) ? other_roots[idx] : Position{-1, -1};

        if (seq != nullptr) {
            for (int v : seq->lidar_footprint) {
                if (mask.masked_grid[v] == -1) mask.masked_grid[v] = 0;
            }
            for (int v : seq->cam_footprint) {
                if (mask.masked_grid[v] == 0) mask.masked_grid[v] = 2;
                mask.masked_known_cp[v] = 0;
            }
        }

        for (int s = 0; s < max_steps; ++s) {
            Position p = last_pos;
            if (seq != nullptr && s < static_cast<int>(seq->path.size())) {
                p = seq->path[s];
                last_pos = p;
            }
            if (p.x >= 0 && p.x < W && p.y >= 0 && p.y < H) {
                mask.teammate_positions[s].push_back(p);
                mask.teammate_traj_visits[p.y * W + p.x]++;
            }
        }
    }
    return mask;
}

double evaluateLocalMarginalUtilityFast(
    const DroneAgent& agent,
    const std::vector<Position>& x_r,
    const Environment& base_map,
    const TeammateMask& team_mask,
    const TSDFMap& tsdf) {

    if (x_r.empty()) return -99999.0;

    int W = base_map.getWidth();
    std::vector<int8_t> sim_grid = team_mask.masked_grid;
    std::vector<uint8_t> sim_cp = team_mask.masked_known_cp;

    int sim_x = agent.drone_pos.x;
    int sim_y = agent.drone_pos.y;
    int sim_prev_x = agent.prev_pos.x;
    int sim_prev_y = agent.prev_pos.y;

    int prev_dx = (agent.prev_pos.x != -1) ? (agent.drone_pos.x - agent.prev_pos.x) : 0;
    int prev_dy = (agent.prev_pos.y != -1) ? (agent.drone_pos.y - agent.prev_pos.y) : 0;
    int prev2_dx = (agent.prev2_pos.x != -1 && agent.prev_pos.x != -1) ? (agent.prev_pos.x - agent.prev2_pos.x) : 0;
    int prev2_dy = (agent.prev2_pos.y != -1 && agent.prev_pos.y != -1) ? (agent.prev_pos.y - agent.prev2_pos.y) : 0;

    constexpr double gamma = 0.9;
    const double safe_clearance = std::max(1.0, static_cast<double>(base_map.getCameraFovRadius() - 1.0));

    constexpr double W_NEW_LIDAR = 300.0;
    constexpr double W_NEW_CAMERA = 350.0;
    constexpr double W_KNOWN_CP_CAMERA = 2500.0;
    constexpr double W_OVERLAP_LIDAR = 5.0;
    constexpr double W_OVERLAP_CAMERA = 8.0;
    constexpr double W_OVERLAP_CAMERA_OBS = 5.0;
    constexpr double W_TSDF_PENALTY = 1250.0;
    constexpr double W_LATERAL_CENTER = 600.0;
    constexpr double W_GRADIENT = 520.0;
    constexpr double W_FRONTIER_PULL = 1200.0;
    constexpr double W_CP_BIAS = 1600.0;
    constexpr double W_OWN_REVISIT_PENALTY = 320.0;
    constexpr double W_TEAM_REVISIT_PENALTY = 110.0;
    constexpr double W_COLLISION_PENALTY = 50000.0;
    constexpr double STEP_COST = 25.0;

    double total_marginal_utility = 0.0;
    double discount = 1.0;
    double best_frontier_dist_seen = base_map.getDistanceToNearestUncovered(sim_x, sim_y);

    if (x_r[0] == agent.prev_pos) {
        total_marginal_utility -= 800.0;
    } else if (x_r[0] == agent.prev2_pos) {
        total_marginal_utility -= 500.0;
    }

    for (size_t step = 0; step < x_r.size(); ++step) {
        Position next_pos = x_r[step];
        int move_dx = next_pos.x - sim_x;
        int move_dy = next_pos.y - sim_y;

        double step_reward = computeKinematicSmoothness(prev2_dx, prev2_dy, prev_dx, prev_dy, move_dx, move_dy);
        double lat_pen = tsdf.getLateralCenteringPenalty(sim_x, sim_y, next_pos.x, next_pos.y, safe_clearance, W_LATERAL_CENTER);

        prev2_dx = prev_dx;
        prev2_dy = prev_dy;
        prev_dx = move_dx;
        prev_dy = move_dy;

        sim_prev_x = sim_x;
        sim_prev_y = sim_y;
        sim_x = next_pos.x;
        sim_y = next_pos.y;
        int cell = sim_y * W + sim_x;

        base_map.ensureCellCached(sim_x, sim_y, tsdf);

        if (step < team_mask.teammate_positions.size()) {
            for (size_t m = 0; m < team_mask.teammate_positions[step].size(); ++m) {
                Position mate_now = team_mask.teammate_positions[step][m];
                if (mate_now.x == sim_x && mate_now.y == sim_y) {
                    step_reward -= W_COLLISION_PENALTY;
                }
                Position mate_prev = (step == 0)
                    ? (m < team_mask.teammate_current_roots.size() ? team_mask.teammate_current_roots[m] : Position{-1, -1})
                    : (m < team_mask.teammate_positions[step - 1].size() ? team_mask.teammate_positions[step - 1][m] : Position{-1, -1});
                if (mate_prev.x == sim_x && mate_prev.y == sim_y && mate_now.x == sim_prev_x && mate_now.y == sim_prev_y) {
                    step_reward -= W_COLLISION_PENALTY;
                }
            }
        }

        double tsdf_val = tsdf.getTSDFValue(sim_x, sim_y);
        if (sim_grid[cell] == 1 || tsdf_val <= 0.05) {
            step_reward -= 99999.0;
            total_marginal_utility += step_reward * discount;
            break;
        }

        int own_v = agent.flat_own_visits[cell];
        int total_v = base_map.flat_visits[cell];
        int team_v = std::max(0, total_v - own_v) + team_mask.teammate_traj_visits[cell];

        double dist_before = (sim_prev_x != -1) ? base_map.getDistanceToNearestUncovered(sim_prev_x, sim_prev_y)
                                                : base_map.getDistanceToNearestUncovered(sim_x, sim_y);
        double dist_now = base_map.getDistanceToNearestUncovered(sim_x, sim_y);
        bool is_advancing_to_frontier = best_frontier_dist_seen > 0.5 &&
                        dist_now < best_frontier_dist_seen;

        double frontier_progress_reward = 0.0;
        if (best_frontier_dist_seen > 0.5) {
            if (dist_now < best_frontier_dist_seen) {
                frontier_progress_reward = W_FRONTIER_PULL * (best_frontier_dist_seen - dist_now);
                best_frontier_dist_seen = dist_now;
            } else if (dist_now > dist_before) {
                frontier_progress_reward = -0.6 * W_FRONTIER_PULL * (dist_now - dist_before);
            }
        }

        double cp_bias_reward = (sim_prev_x != -1)
            ? base_map.bias(sim_prev_x, sim_prev_y, sim_x, sim_y, W_CP_BIAS)
            : 0.0;

        int l_new = 0, l_ov = 0;
        for (int v_idx : base_map.lidar_vis_cache[cell]) {
            if (sim_grid[v_idx] == -1) {
                l_new++;
                sim_grid[v_idx] = 0;
            } else {
                l_ov++;
            }
        }

        int c_new = 0, c_ov = 0, cp_cov = 0;
        int c_obs = base_map.cam_obs_count_cache[cell];
        for (int v_idx : base_map.cam_vis_cache[cell]) {
            int8_t g = sim_grid[v_idx];
            if (g == 0) {
                c_new++;
                sim_grid[v_idx] = 2;
            } else if (g == 2) {
                c_ov++;
            }
            if (base_map.cp_coords_known && sim_cp[v_idx]) {
                cp_cov++;
                sim_cp[v_idx] = 0;
            }
        }

        int effective_own_v = is_advancing_to_frontier ? std::min(own_v, 3) : own_v;
        double revisit_scale = is_advancing_to_frontier ? 0.25 : 1.0;
        double clearance_penalty = is_advancing_to_frontier ? 0.35 * lat_pen : lat_pen;
        if (tsdf_val < safe_clearance) {
            double deficit = safe_clearance - tsdf_val;
            double narrow_scale = (lat_pen <= 1e-3)
                ? 0.25
                : (is_advancing_to_frontier ? 0.40 : 1.0);
            clearance_penalty += narrow_scale * W_TSDF_PENALTY * deficit * deficit;
        }

        double disc_factor = tsdf.getClearanceDiscoveryFactor(sim_x, sim_y, safe_clearance);

        double own_revisit_pen = revisit_scale * W_OWN_REVISIT_PENALTY * effective_own_v * effective_own_v;
        double team_revisit_pen = revisit_scale * W_TEAM_REVISIT_PENALTY * team_v * team_v;
        double step_grad_score = (sim_prev_x != -1)
            ? tsdf.getGradientScore(sim_prev_x, sim_prev_y, sim_x, sim_y, safe_clearance, W_GRADIENT)
            : 0.0;

        step_reward += disc_factor * (W_NEW_LIDAR * l_new + W_NEW_CAMERA * c_new);
        step_reward += W_KNOWN_CP_CAMERA * cp_cov;
        step_reward += frontier_progress_reward;
        step_reward += cp_bias_reward;
        step_reward += step_grad_score;
        step_reward -= W_OVERLAP_LIDAR * l_ov;
        step_reward -= W_OVERLAP_CAMERA * c_ov;
        step_reward -= W_OVERLAP_CAMERA_OBS * c_obs;
        step_reward -= clearance_penalty;
        step_reward -= (own_revisit_pen + team_revisit_pen);
        step_reward -= STEP_COST;

        total_marginal_utility += step_reward * discount;
        discount *= gamma;
    }

    return total_marginal_utility;
}

double performRolloutAndEvaluateFast(
    const DroneAgent& agent,
    const std::vector<MCTSNode*>& tree_path,
    const Environment& base_map,
    const TeammateMask& team_mask,
    int max_steps,
    const TSDFMap& tsdf,
    std::mt19937& gen,
    std::vector<Position>& out_full_sequence) {

    out_full_sequence.clear();
    if (tree_path.empty()) return -99999.0;

    int W = base_map.getWidth();
    std::vector<int8_t> sim_grid = team_mask.masked_grid;
    std::vector<uint8_t> sim_cp = team_mask.masked_known_cp;

    int sim_x = agent.drone_pos.x;
    int sim_y = agent.drone_pos.y;
    int sim_prev_x = agent.prev_pos.x;
    int sim_prev_y = agent.prev_pos.y;
    int sim_prev2_x = agent.prev2_pos.x;
    int sim_prev2_y = agent.prev2_pos.y;

    int prev_dx = (agent.prev_pos.x != -1) ? (agent.drone_pos.x - agent.prev_pos.x) : 0;
    int prev_dy = (agent.prev_pos.y != -1) ? (agent.drone_pos.y - agent.prev_pos.y) : 0;
    int prev2_dx = (agent.prev2_pos.x != -1 && agent.prev_pos.x != -1) ? (agent.prev_pos.x - agent.prev2_pos.x) : 0;
    int prev2_dy = (agent.prev2_pos.y != -1 && agent.prev_pos.y != -1) ? (agent.prev_pos.y - agent.prev2_pos.y) : 0;

    constexpr double gamma = 0.9;
    const double safe_clearance = std::max(1.0, static_cast<double>(base_map.getCameraFovRadius() - 1.0));

    constexpr double W_NEW_LIDAR = 300.0;
    constexpr double W_NEW_CAMERA = 350.0;
    constexpr double W_KNOWN_CP_CAMERA = 2500.0;
    constexpr double W_OVERLAP_LIDAR = 5.0;
    constexpr double W_OVERLAP_CAMERA = 8.0;
    constexpr double W_OVERLAP_CAMERA_OBS = 5.0;
    constexpr double W_TSDF_PENALTY = 1250.0;
    constexpr double W_LATERAL_CENTER = 600.0;
    constexpr double W_GRADIENT = 520.0;
    constexpr double W_FRONTIER_PULL = 1200.0;
    constexpr double W_CP_BIAS = 1600.0;
    constexpr double W_OWN_REVISIT_PENALTY = 320.0;
    constexpr double W_TEAM_REVISIT_PENALTY = 110.0;
    constexpr double W_COLLISION_PENALTY = 50000.0;
    constexpr double STEP_COST = 25.0;

    double total_marginal_utility = 0.0;
    double discount = 1.0;
    double best_frontier_dist_seen = base_map.getDistanceToNearestUncovered(sim_x, sim_y);

    for (int step = 0; step < max_steps; ++step) {
        Position chosen_move;
        base_map.ensureCellCached(sim_x, sim_y, tsdf);

        if (step < static_cast<int>(tree_path.size())) {
            chosen_move = tree_path[step]->state;
        } else {
            const auto& all_moves = base_map.legal_moves_cache[sim_y * W + sim_x];
            if (all_moves.empty()) break;

            // Filter out immediate 1-step back (sim_prev), 2-step back (sim_prev2), AND any tile already in out_full_sequence
            std::vector<Position> legal_moves;
            legal_moves.reserve(all_moves.size());
            for (const auto& m : all_moves) {
                if ((m.x == sim_prev_x && m.y == sim_prev_y) ||
                    (m.x == sim_prev2_x && m.y == sim_prev2_y) ||
                    (m == agent.drone_pos)) {
                    continue;
                }
                if (std::find(out_full_sequence.begin(), out_full_sequence.end(), m) != out_full_sequence.end()) {
                    continue;
                }
                legal_moves.push_back(m);
            }
            if (legal_moves.empty()) {
                for (const auto& m : all_moves) {
                    if (!(m.x == sim_prev_x && m.y == sim_prev_y)) {
                        legal_moves.push_back(m);
                    }
                }
            }
            if (legal_moves.empty()) {
                break;
                //legal_moves = all_moves;
            }

            legal_moves = filterKinematicConeMoves(sim_x, sim_y, prev_dx, prev_dy, legal_moves, tsdf, safe_clearance);

            double curr_frontier_dist = base_map.getDistanceToNearestUncovered(sim_x, sim_y);
            std::vector<double> move_weights;
            move_weights.reserve(legal_moves.size());

            for (const auto& move : legal_moves) {
                base_map.ensureCellCached(move.x, move.y, tsdf);
                int m_cell = move.y * W + move.x;
                int own_v = agent.flat_own_visits[m_cell];
                int total_v = base_map.flat_visits[m_cell];
                int team_v = std::max(0, total_v - own_v) + team_mask.teammate_traj_visits[m_cell];

                int l_new = 0, l_ov = 0;
                for (int v_idx : base_map.lidar_vis_cache[m_cell]) {
                    if (sim_grid[v_idx] == -1) l_new++;
                    else l_ov++;
                }

                int c_new = 0, c_ov = 0, cp_cov = 0;
                int c_obs = base_map.cam_obs_count_cache[m_cell];
                for (int v_idx : base_map.cam_vis_cache[m_cell]) {
                    int8_t g = sim_grid[v_idx];
                    if (g == 0) c_new++;
                    else if (g == 2) c_ov++;
                    if (base_map.cp_coords_known && sim_cp[v_idx]) cp_cov++;
                }

                double lat_pen = tsdf.getLateralCenteringPenalty(sim_x, sim_y, move.x, move.y, safe_clearance, W_LATERAL_CENTER);
                double clearance = tsdf.getTSDFValue(move.x, move.y);
                double next_frontier_dist = base_map.getDistanceToNearestUncovered(move.x, move.y);
                bool is_advancing_to_frontier = best_frontier_dist_seen > 0.5 &&
                                                next_frontier_dist < best_frontier_dist_seen;
                int effective_own_v = is_advancing_to_frontier ? std::min(own_v + 1, 3) : own_v + 1;
                double revisit_scale = is_advancing_to_frontier ? 0.25 : 1.0;
                double clearance_pen = is_advancing_to_frontier ? 0.35 * lat_pen : lat_pen;
                if (clearance < safe_clearance) {
                    double deficit = safe_clearance - clearance;
                    double narrow_scale = (lat_pen <= 1e-3)
                        ? 0.25
                        : (is_advancing_to_frontier ? 0.40 : 1.0);
                    clearance_pen += narrow_scale * W_TSDF_PENALTY * deficit * deficit;
                }

                double disc_factor = tsdf.getClearanceDiscoveryFactor(move.x, move.y, safe_clearance);

                double collision_pen = 0.0;
                if (step < static_cast<int>(team_mask.teammate_positions.size())) {
                    for (size_t m = 0; m < team_mask.teammate_positions[step].size(); ++m) {
                        Position mate_now = team_mask.teammate_positions[step][m];
                        if (mate_now == move) collision_pen += W_COLLISION_PENALTY;
                        Position mate_prev = (step == 0)
                            ? (m < team_mask.teammate_current_roots.size() ? team_mask.teammate_current_roots[m] : Position{-1, -1})
                            : (m < team_mask.teammate_positions[step - 1].size() ? team_mask.teammate_positions[step - 1][m] : Position{-1, -1});
                        if (mate_prev == move && mate_now.x == sim_x && mate_now.y == sim_y) {
                            collision_pen += W_COLLISION_PENALTY;
                        }
                    }
                }

                double gradient_score = tsdf.getGradientScore(sim_x, sim_y, move.x, move.y, safe_clearance, W_GRADIENT);
                double revisit_pen = revisit_scale * W_OWN_REVISIT_PENALTY * effective_own_v * effective_own_v
                                   + revisit_scale * W_TEAM_REVISIT_PENALTY * team_v * team_v;

                // Match frontier progress formula between move_weights and step_reward!
                double frontier_pull = 0.0;
                if (best_frontier_dist_seen > 0.5) {
                    if (next_frontier_dist < best_frontier_dist_seen) {
                        frontier_pull = W_FRONTIER_PULL * (best_frontier_dist_seen - next_frontier_dist);
                    } else if (next_frontier_dist > curr_frontier_dist) {
                        frontier_pull = -0.6 * W_FRONTIER_PULL * (next_frontier_dist - curr_frontier_dist);
                    }
                }
                double cp_bias_pull = base_map.bias(sim_x, sim_y, move.x, move.y, W_CP_BIAS);

                int move_dx = move.x - sim_x;
                int move_dy = move.y - sim_y;
                double heading_score = computeKinematicSmoothness(prev2_dx, prev2_dy, prev_dx, prev_dy, move_dx, move_dy);

                double candidate_score = disc_factor * (W_NEW_LIDAR * l_new + W_NEW_CAMERA * c_new)
                                       + W_KNOWN_CP_CAMERA * cp_cov
                                       + frontier_pull
                                       + cp_bias_pull
                                       + gradient_score
                                       + heading_score
                                       - W_OVERLAP_LIDAR * l_ov
                                       - W_OVERLAP_CAMERA * c_ov
                                       - W_OVERLAP_CAMERA_OBS * c_obs
                                       - clearance_pen
                                       - revisit_pen
                                       - collision_pen;

                move_weights.push_back(std::exp(std::clamp(candidate_score / 400.0, -20.0, 20.0)));
            }

            std::discrete_distribution<> distrib(move_weights.begin(), move_weights.end());
            int choice = distrib(gen);
            chosen_move = legal_moves[choice];
        }

        if (step == 0) {
            if (chosen_move == agent.prev_pos) {
                total_marginal_utility -= 800.0;
            } else if (chosen_move == agent.prev2_pos) {
                total_marginal_utility -= 500.0;
            }
        }

        out_full_sequence.push_back(chosen_move);
        int move_dx = chosen_move.x - sim_x;
        int move_dy = chosen_move.y - sim_y;
        double step_reward = computeKinematicSmoothness(prev2_dx, prev2_dy, prev_dx, prev_dy, move_dx, move_dy);
        double lat_pen = tsdf.getLateralCenteringPenalty(sim_x, sim_y, chosen_move.x, chosen_move.y, safe_clearance, W_LATERAL_CENTER);

        prev2_dx = prev_dx;
        prev2_dy = prev_dy;
        prev_dx = move_dx;
        prev_dy = move_dy;
        sim_prev2_x = sim_prev_x;
        sim_prev2_y = sim_prev_y;
        sim_prev_x = sim_x;
        sim_prev_y = sim_y;
        sim_x = chosen_move.x;
        sim_y = chosen_move.y;
        int cell = sim_y * W + sim_x;

        base_map.ensureCellCached(sim_x, sim_y, tsdf);

        if (step < static_cast<int>(team_mask.teammate_positions.size())) {
            for (size_t m = 0; m < team_mask.teammate_positions[step].size(); ++m) {
                Position mate_now = team_mask.teammate_positions[step][m];
                if (mate_now.x == sim_x && mate_now.y == sim_y) step_reward -= W_COLLISION_PENALTY;
                Position mate_prev = (step == 0)
                    ? (m < team_mask.teammate_current_roots.size() ? team_mask.teammate_current_roots[m] : Position{-1, -1})
                    : (m < team_mask.teammate_positions[step - 1].size() ? team_mask.teammate_positions[step - 1][m] : Position{-1, -1});
                if (mate_prev.x == sim_x && mate_prev.y == sim_y && mate_now.x == sim_prev_x && mate_now.y == sim_prev_y) {
                    step_reward -= W_COLLISION_PENALTY;
                }
            }
        }

        double tsdf_val = tsdf.getTSDFValue(sim_x, sim_y);
        if (sim_grid[cell] == 1 || tsdf_val <= 0.05) {
            step_reward -= 99999.0;
            total_marginal_utility += step_reward * discount;
            break;
        }

        int own_v = agent.flat_own_visits[cell];
        int total_v = base_map.flat_visits[cell];
        int team_v = std::max(0, total_v - own_v) + team_mask.teammate_traj_visits[cell];

        double dist_before = (sim_prev_x != -1) ? base_map.getDistanceToNearestUncovered(sim_prev_x, sim_prev_y)
                                                : base_map.getDistanceToNearestUncovered(sim_x, sim_y);
        double dist_now = base_map.getDistanceToNearestUncovered(sim_x, sim_y);

        double frontier_progress_reward = 0.0;
        if (best_frontier_dist_seen > 0.5) {
            if (dist_now < best_frontier_dist_seen) {
                frontier_progress_reward = W_FRONTIER_PULL * (best_frontier_dist_seen - dist_now);
                best_frontier_dist_seen = dist_now;
            } else if (dist_now > dist_before) {
                frontier_progress_reward = -0.6 * W_FRONTIER_PULL * (dist_now - dist_before);
            }
        }

        double cp_bias_reward = (sim_prev_x != -1)
            ? base_map.bias(sim_prev_x, sim_prev_y, sim_x, sim_y, W_CP_BIAS)
            : 0.0;

        int l_new = 0, l_ov = 0;
        for (int v_idx : base_map.lidar_vis_cache[cell]) {
            if (sim_grid[v_idx] == -1) {
                l_new++;
                sim_grid[v_idx] = 0;
            } else {
                l_ov++;
            }
        }

        int c_new = 0, c_ov = 0, cp_cov = 0;
        int c_obs = base_map.cam_obs_count_cache[cell];
        for (int v_idx : base_map.cam_vis_cache[cell]) {
            int8_t g = sim_grid[v_idx];
            if (g == 0) {
                c_new++;
                sim_grid[v_idx] = 2;
            } else if (g == 2) {
                c_ov++;
            }
            if (base_map.cp_coords_known && sim_cp[v_idx]) {
                cp_cov++;
                sim_cp[v_idx] = 0;
            }
        }

        bool is_advancing_to_frontier = best_frontier_dist_seen > 0.5 &&
                                        dist_now < best_frontier_dist_seen;
        int effective_own_v = is_advancing_to_frontier ? std::min(own_v, 3) : own_v;
        double revisit_scale = is_advancing_to_frontier ? 0.25 : 1.0;
        double clearance_penalty = is_advancing_to_frontier ? 0.35 * lat_pen : lat_pen;
        if (tsdf_val < safe_clearance) {
            double deficit = safe_clearance - tsdf_val;
            double narrow_scale = (lat_pen <= 1e-3)
                ? 0.25
                : (is_advancing_to_frontier ? 0.40 : 1.0);
            clearance_penalty += narrow_scale * W_TSDF_PENALTY * deficit * deficit;
        }

        double disc_factor = tsdf.getClearanceDiscoveryFactor(sim_x, sim_y, safe_clearance);

        double own_revisit_pen = revisit_scale * W_OWN_REVISIT_PENALTY * effective_own_v * effective_own_v;
        double team_revisit_pen = revisit_scale * W_TEAM_REVISIT_PENALTY * team_v * team_v;
        double step_grad_score = (sim_prev_x != -1)
            ? tsdf.getGradientScore(sim_prev_x, sim_prev_y, sim_x, sim_y, safe_clearance, W_GRADIENT)
            : 0.0;

        step_reward += disc_factor * (W_NEW_LIDAR * l_new + W_NEW_CAMERA * c_new);
        step_reward += W_KNOWN_CP_CAMERA * cp_cov;
        step_reward += frontier_progress_reward;
        step_reward += cp_bias_reward;
        step_reward += step_grad_score;
        step_reward -= W_OVERLAP_LIDAR * l_ov;
        step_reward -= W_OVERLAP_CAMERA * c_ov;
        step_reward -= W_OVERLAP_CAMERA_OBS * c_obs;
        step_reward -= clearance_penalty;
        step_reward -= (own_revisit_pen + team_revisit_pen);
        step_reward -= STEP_COST;

        total_marginal_utility += step_reward * discount;
        discount *= gamma;
    }

    return total_marginal_utility;
}

void sampleTeammateSequencesFast(
    int self_idx,
    const std::vector<DroneAgent>& agents,
    std::mt19937& gen,
    std::vector<const CandidateSequence*>& out_other_seqs,
    std::vector<Position>& out_other_roots) {

    out_other_seqs.clear();
    out_other_roots.clear();

    for (size_t r = 0; r < agents.size(); ++r) {
        if (static_cast<int>(r) == self_idx) continue;

        out_other_roots.push_back(agents[r].drone_pos);

        if (!agents[r].current_plan.empty()) {
            out_other_seqs.push_back(&agents[r].committed_sequence);
        } else if (!agents[r].sparse_domain.empty()) {
            std::vector<double> probs;
            probs.reserve(agents[r].sparse_domain.size());
            for (const auto& cand : agents[r].sparse_domain) {
                probs.push_back(std::max(1e-6, cand.prob));
            }
            std::discrete_distribution<> dist(probs.begin(), probs.end());
            int pick = dist(gen);
            out_other_seqs.push_back(&agents[r].sparse_domain[pick]);
        } else {
            out_other_seqs.push_back(nullptr);
        }
    }
}

std::vector<Position> sanitizeRolloutPath(
    const std::vector<Position>& raw_path,
    Position root_pos,
    Position expected_first_step) {

    auto it = std::find(raw_path.begin(), raw_path.end(), expected_first_step);
    if (it == raw_path.end()) return {};

    std::vector<Position> clean;
    Position prev = root_pos;
    std::set<Position> visited_in_path = {root_pos};

    for (; it != raw_path.end(); ++it) {
        int dx = std::abs(it->x - prev.x);
        int dy = std::abs(it->y - prev.y);
        if (dx > 1 || dy > 1 || (dx == 0 && dy == 0)) break;
        if (visited_in_path.count(*it)) break;
        visited_in_path.insert(*it);
        clean.push_back(*it);
        prev = *it;
    }
    return clean;
}

// Consistent SelectSetOfSequences: uses pure D-UCT empirical average without double-subtracting penalties
void selectSetOfSequences(DroneAgent& agent, const Environment& known_map, const TSDFMap& tsdf, size_t K_s = 8) {
    if (!agent.mcts_root) return;

    struct NodeEntry {
        std::vector<Position> path;
        double f_bar;
        Position first_step;
    };

    std::vector<NodeEntry> root_entries;
    std::vector<NodeEntry> deeper_entries;

    for (const auto& root_child : agent.mcts_root->children) {
        if (root_child->visits <= 0.0 || root_child->best_rollout_path.empty()) continue;

        Position fs = root_child->state;
        std::vector<Position> clean_root_path = sanitizeRolloutPath(
            root_child->best_rollout_path, agent.drone_pos, fs);
        if (clean_root_path.empty()) continue;

        double reliable_score = root_child->getDiscountedAverage();
        root_entries.push_back({std::move(clean_root_path), reliable_score, fs});

        for (const auto& grand_child : root_child->children) {
            if (grand_child->visits >= 2.0 && !grand_child->best_rollout_path.empty()) {
                std::vector<Position> clean_gc_path = sanitizeRolloutPath(
                    grand_child->best_rollout_path, agent.drone_pos, fs);
                if (clean_gc_path.size() >= 3) {
                    double gc_score = grand_child->getDiscountedAverage();
                    deeper_entries.push_back({std::move(clean_gc_path), gc_score, fs});
                }
            }
        }
    }

    std::sort(root_entries.begin(), root_entries.end(),
              [](const NodeEntry& a, const NodeEntry& b) { return a.f_bar > b.f_bar; });
    std::sort(deeper_entries.begin(), deeper_entries.end(),
              [](const NodeEntry& a, const NodeEntry& b) { return a.f_bar > b.f_bar; });

    std::vector<CandidateSequence> next_domain;
    std::set<std::vector<Position>> unique_paths;

    for (const auto& entry : root_entries) {
        if (next_domain.size() >= K_s) break;
        if (unique_paths.count(entry.path) == 0) {
            unique_paths.insert(entry.path);
            CandidateSequence cs{entry.path, entry.f_bar, 0.0, entry.f_bar, {}, {}};
            cs.precomputeFootprints(known_map, tsdf);
            next_domain.push_back(std::move(cs));
        }
    }

    for (const auto& entry : deeper_entries) {
        if (next_domain.size() >= K_s) break;
        if (unique_paths.count(entry.path) == 0) {
            unique_paths.insert(entry.path);
            CandidateSequence cs{entry.path, entry.f_bar, 0.0, entry.f_bar, {}, {}};
            cs.precomputeFootprints(known_map, tsdf);
            next_domain.push_back(std::move(cs));
        }
    }

    if (next_domain.empty()) return;

    double unif_p = 1.0 / static_cast<double>(next_domain.size());
    for (auto& c : next_domain) {
        for (const auto& prev_c : agent.sparse_domain) {
            if (prev_c.path == c.path) {
                c.prob = prev_c.prob;
                break;
            }
        }
        if (c.prob <= 0.0) c.prob = unif_p;
    }

    double sum_p = 0.0;
    for (const auto& c : next_domain) sum_p += c.prob;
    for (auto& c : next_domain) c.prob = (sum_p > 0.0) ? (c.prob / sum_p) : unif_p;

    agent.sparse_domain = std::move(next_domain);
}

void updateDistributionPC(
    int self_idx,
    std::vector<DroneAgent>& agents,
    const Environment& known_map,
    const TSDFMap& tsdf,
    std::mt19937& gen,
    int max_steps = 10,
    int mc_samples = 25) {

    DroneAgent& self = agents[self_idx];
    size_t K = self.sparse_domain.size();
    if (K == 0) return;

    std::vector<int> peer_indices;
    for (size_t r = 0; r < agents.size(); ++r) {
        if (static_cast<int>(r) != self_idx) {
            peer_indices.push_back(static_cast<int>(r));
        }
    }

    if (peer_indices.empty()) {
        TeammateMask empty_mask(known_map, max_steps);
        for (size_t k = 0; k < K; ++k) {
            self.sparse_domain[k].cond_utility = evaluateLocalMarginalUtilityFast(
                self, self.sparse_domain[k].path, known_map, empty_mask, tsdf);
        }
    } else if (peer_indices.size() == 1 && agents[peer_indices[0]].current_plan.empty() &&
               !agents[peer_indices[0]].sparse_domain.empty()) {
        const DroneAgent& peer = agents[peer_indices[0]];
        std::vector<double> cond_u(K, 0.0);

        for (const auto& peer_cand : peer.sparse_domain) {
            if (peer_cand.prob <= 1e-6) continue;
            std::vector<const CandidateSequence*> other_seqs = {&peer_cand};
            std::vector<Position> other_roots = {peer.drone_pos};
            TeammateMask mask = buildTeammateMaskFast(known_map, other_seqs, other_roots, max_steps);

            for (size_t k = 0; k < K; ++k) {
                double f_val = evaluateLocalMarginalUtilityFast(self, self.sparse_domain[k].path, known_map, mask, tsdf);
                cond_u[k] += peer_cand.prob * f_val;
            }
        }
        for (size_t k = 0; k < K; ++k) {
            self.sparse_domain[k].cond_utility = cond_u[k];
        }
    } else {
        std::vector<double> cond_u(K, 0.0);
        for (int m = 0; m < mc_samples; ++m) {
            std::vector<const CandidateSequence*> other_seqs;
            std::vector<Position> other_roots;
            sampleTeammateSequencesFast(self_idx, agents, gen, other_seqs, other_roots);
            TeammateMask mask = buildTeammateMaskFast(known_map, other_seqs, other_roots, max_steps);

            for (size_t k = 0; k < K; ++k) {
                double f_val = evaluateLocalMarginalUtilityFast(self, self.sparse_domain[k].path, known_map, mask, tsdf);
                cond_u[k] += f_val / static_cast<double>(mc_samples);
            }
        }
        for (size_t k = 0; k < K; ++k) {
            self.sparse_domain[k].cond_utility = cond_u[k];
        }
    }

    double E_f = 0.0;
    double H_q = 0.0;
    for (size_t k = 0; k < K; ++k) {
        double p = std::max(1e-6, self.sparse_domain[k].prob);
        E_f += p * self.sparse_domain[k].cond_utility;
        H_q -= p * std::log(p);
    }

    constexpr double alpha = 0.08;
    double beta = std::max(10.0, self.beta);

    double sum_new_p = 0.0;
    for (size_t k = 0; k < K; ++k) {
        double p = std::max(1e-6, self.sparse_domain[k].prob);
        double advantage_term = (E_f - self.sparse_domain[k].cond_utility) / beta;
        double entropy_term = H_q + std::log(p);
        double bracket = std::clamp(advantage_term + entropy_term, -8.0, 8.0);

        double updated_p = p - alpha * p * bracket;
        updated_p = std::max(1e-4, updated_p);
        self.sparse_domain[k].prob = updated_p;
        sum_new_p += updated_p;
    }

    for (size_t k = 0; k < K; ++k) {
        self.sparse_domain[k].prob /= sum_new_p;
    }
}

void runDecMCTSForSwarm(
    const std::vector<int>& planning_drones,
    std::vector<DroneAgent>& agents,
    const Environment& known_map,
    const TSDFMap& tsdf,
    std::mt19937& gen,
    int N_outer = 10,
    int tau_n = 60,
    size_t K_s = 8,
    int rollout_horizon = 10,
    int k_chain_steps = 3) {

    if (planning_drones.empty()) return;

    for (auto& ag : agents) {
        if (!ag.current_plan.empty()) {
            ag.committed_sequence.path.assign(ag.current_plan.begin(), ag.current_plan.end());
            ag.committed_sequence.precomputeFootprints(known_map, tsdf);
        }
    }

    for (int id : planning_drones) {
        DroneAgent& ag = agents[id];
        if (!ag.mcts_root || ag.mcts_root->state != ag.drone_pos) {
            ag.mcts_root = std::make_unique<MCTSNode>(ag.drone_pos);
        }
        ag.mcts_root->pruneInvalidNodes(known_map, tsdf);
        ag.mcts_root->refreshRootForCurrentHeading(known_map, tsdf, ag.prev_pos, ag.prev2_pos);
        if (ag.mcts_root->children.empty()) {
            ag.mcts_root->expandNode(known_map, tsdf, ag.prev_pos, ag.prev2_pos);
        }
        ag.beta = 400.0;
    }

    for (int n = 0; n < N_outer; ++n) {
        for (int id : planning_drones) {
            DroneAgent& ag = agents[id];

            for (int sample = 0; sample < tau_n; ++sample) {
                MCTSNode* leaf = ag.mcts_root.get();
                std::vector<MCTSNode*> tree_path;

                while (!leaf->children.empty() && static_cast<int>(tree_path.size()) < rollout_horizon) {
                    MCTSNode* next = leaf->getBestChildDUCT(known_map, tsdf);
                    if (next == nullptr) break;
                    leaf = next;
                    tree_path.push_back(leaf);
                }

                if (leaf->children.empty()) {
                    leaf->expandNode(known_map, tsdf, ag.prev_pos, ag.prev2_pos);
                }

                MCTSNode* simulation_node = leaf;
                if (!leaf->children.empty()) {
                    MCTSNode* child = leaf->getBestChildDUCT(known_map, tsdf);
                    if (child != nullptr) {
                        simulation_node = child;
                        tree_path.push_back(simulation_node);
                    }
                }

                if (simulation_node == nullptr || tree_path.empty()) continue;

                std::vector<const CandidateSequence*> other_seqs;
                std::vector<Position> other_roots;
                sampleTeammateSequencesFast(id, agents, gen, other_seqs, other_roots);
                TeammateMask mask = buildTeammateMaskFast(known_map, other_seqs, other_roots, rollout_horizon);

                std::vector<Position> full_rollout_seq;
                double F_t = performRolloutAndEvaluateFast(
                    ag, tree_path, known_map, mask, rollout_horizon, tsdf, gen, full_rollout_seq);

                simulation_node->backpropagateDUCT(F_t, full_rollout_seq, 0.997);
            }

            selectSetOfSequences(ag, known_map, tsdf, K_s);
        }

        for (int id : planning_drones) {
            updateDistributionPC(id, agents, known_map, tsdf, gen, rollout_horizon, 25);
            agents[id].beta *= 0.88;
        }
    }

    for (int id : planning_drones) {
        DroneAgent& ag = agents[id];
        ag.current_plan.clear();
        ag.best_full_plan.clear();

        if (!ag.sparse_domain.empty()) {
            const CandidateSequence* best_seq = &ag.sparse_domain[0];
            for (const auto& cand : ag.sparse_domain) {
                if (cand.prob > best_seq->prob ||
                    (std::abs(cand.prob - best_seq->prob) < 1e-4 && cand.cond_utility > best_seq->cond_utility)) {
                    best_seq = &cand;
                }
            }

            ag.winning_expected_utility = best_seq->cond_utility;
            ag.best_full_plan = best_seq->path;

            int steps_to_extract = std::min(k_chain_steps, static_cast<int>(best_seq->path.size()));
            std::set<Position> seen_in_chain = {ag.drone_pos};

            for (int k = 0; k < steps_to_extract; ++k) {
                Position next_p = best_seq->path[k];
                if (seen_in_chain.count(next_p)) break;
                seen_in_chain.insert(next_p);
                ag.current_plan.push_back(next_p);
            }
        }
    }
}

int calculateHeuristic(Position start, Position goal) {
    int dx = std::abs(start.x - goal.x);
    int dy = std::abs(start.y - goal.y);
    return 10 * std::max(dx, dy) + 4 * std::min(dx, dy);
}

std::vector<Position> calculateAStar(
    Position start,
    Position goal,
    const Environment& map,
    const TSDFMap& tsdf,
    const std::set<Position>& dynamic_obstacles = {}) {

    if (start == goal) return {};

    std::vector<std::vector<int>> g_score(map.getHeight(), std::vector<int>(map.getWidth(), 9999999));
    std::vector<std::vector<Position>> came_from(map.getHeight(), std::vector<Position>(map.getWidth(), {-1, -1}));
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNodes> open_set;

    open_set.push({start, calculateHeuristic(start, goal)});
    g_score[start.y][start.x] = 0;

    const double safe_clearance = std::max(1.0, static_cast<double>(map.getCameraFovRadius() - 1.0));

    while (!open_set.empty()) {
        AStarNode current_node = open_set.top();
        open_set.pop();

        int current_x = current_node.pos.x;
        int current_y = current_node.pos.y;

        if (current_x == goal.x && current_y == goal.y) {
            std::vector<Position> path;
            Position current_trace = {current_x, current_y};

            while (current_trace.x != start.x || current_trace.y != start.y) {
                path.push_back(current_trace);
                current_trace = came_from[current_trace.y][current_trace.x];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<Position> neighbors = map.getLegalMoves(current_x, current_y, true, tsdf);
        for (const auto& neighbor : neighbors) {
            if (current_x == start.x && current_y == start.y && dynamic_obstacles.count(neighbor)) {
                continue;
            }
            int neighbor_x = neighbor.x;
            int neighbor_y = neighbor.y;

            double tsdf_value = tsdf.getTSDFValue(neighbor_x, neighbor_y);
            if (tsdf_value <= 0.05) continue;

            int proximity_penalty = 0;
            if (tsdf_value < safe_clearance) {
                double deficit = safe_clearance - tsdf_value;
                proximity_penalty = static_cast<int>(std::round(deficit * deficit * 60.0));
            }
            if (dynamic_obstacles.count(neighbor)) {
                proximity_penalty += 150;
            }

            int move_cost = (neighbor_x != current_x && neighbor_y != current_y) ? 14 : 10;
            int tentative_g_score = g_score[current_y][current_x] + proximity_penalty + move_cost;

            if (tentative_g_score < g_score[neighbor_y][neighbor_x]) {
                came_from[neighbor_y][neighbor_x] = {current_x, current_y};
                g_score[neighbor_y][neighbor_x] = tentative_g_score;

                int h_score = calculateHeuristic({neighbor_x, neighbor_y}, goal);
                open_set.push({neighbor, tentative_g_score + h_score});
            }
        }
    }
    return {};
}

struct RunStats {
    int total_reachable_free_tiles = 0;
    int total_moves = 0;
    int total_hovers = 0;
    int turns_0_deg = 0;
    int turns_45_deg = 0;
    int turns_90_deg = 0;
    int turns_135_deg = 0;
    int turns_180_deg = 0;
    double total_swarm_distance = 0.0;
    double tsdf_sum = 0.0;
    double min_tsdf = std::numeric_limits<double>::infinity();
    int tsdf_violation_steps = 0;
    double total_planning_ms = 0.0;
    int planning_calls = 0;
    int exploration_steps = -1;
    bool exploration_snapshot_taken = false;
    std::vector<std::vector<int>> exploration_visits;
    std::vector<std::vector<int>> exploration_own_visits;
    int unique_visited_tiles = 0;
    int total_tile_visits = 0;
    std::vector<std::vector<uint8_t>> prev_lidar_fov;
    std::vector<std::vector<uint8_t>> prev_cam_fov;
    std::vector<uint8_t> ever_lidar_seen;
    std::vector<uint8_t> ever_cam_seen;
    int real_lidar_overlap = 0;
    int real_camera_overlap = 0;
    std::vector<int> sar_discovery_steps;
    std::vector<int> cp_discovery_steps;
};

void recordRealOverlapForDrone(int drone_id, Position pos, const Environment& known_map,
                               const std::vector<std::vector<int>>& true_map, RunStats& stats) {
    int width = known_map.getWidth();
    int height = known_map.getHeight();
    std::vector<uint8_t> curr_lidar(width * height, 0);
    std::vector<uint8_t> curr_cam(width * height, 0);
    int lidar_radius = known_map.getLidarFovRadius();
    int camera_radius = known_map.getCameraFovRadius();

    for (int dy = -lidar_radius; dy <= lidar_radius; ++dy) {
        for (int dx = -lidar_radius; dx <= lidar_radius; ++dx) {
            if (dx * dx + dy * dy > lidar_radius * lidar_radius) continue;
            int x = pos.x + dx, y = pos.y + dy;
            if (x < 0 || x >= width || y < 0 || y >= height) continue;
            if (Environment::isOpaqueTile(true_map[y][x])) continue;
            if (known_map.hasLineOfSight(pos.x, pos.y, x, y, true_map)) {
                curr_lidar[y * width + x] = 1;
            }
        }
    }
    for (int dy = -camera_radius; dy <= camera_radius; ++dy) {
        for (int dx = -camera_radius; dx <= camera_radius; ++dx) {
            if (dx * dx + dy * dy > camera_radius * camera_radius) continue;
            int x = pos.x + dx, y = pos.y + dy;
            if (x < 0 || x >= width || y < 0 || y >= height) continue;
            if (Environment::isOpaqueTile(known_map.grid[y][x])) continue;
            if (known_map.hasCameraLineOfSight(pos.x, pos.y, x, y)) {
                curr_cam[y * width + x] = 1;
            }
        }
    }

    for (int idx = 0; idx < width * height; ++idx) {
        if (curr_lidar[idx] && !stats.prev_lidar_fov[drone_id][idx]) {
            if (stats.ever_lidar_seen[idx]) ++stats.real_lidar_overlap;
            stats.ever_lidar_seen[idx] = 1;
        }
        if (curr_cam[idx] && !stats.prev_cam_fov[drone_id][idx]) {
            if (stats.ever_cam_seen[idx]) ++stats.real_camera_overlap;
            stats.ever_cam_seen[idx] = 1;
        }
    }
    stats.prev_lidar_fov[drone_id] = std::move(curr_lidar);
    stats.prev_cam_fov[drone_id] = std::move(curr_cam);
}

int countReachableFreeTiles(const Environment& true_map, const std::vector<DroneAgent>& agents) {
    const int width = true_map.getWidth();
    const int height = true_map.getHeight();
    std::vector<uint8_t> visited(width * height, 0);
    std::queue<Position> pending;
    for (const auto& agent : agents) {
        Position start = agent.home_pos;
        if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height ||
            Environment::isOpaqueTile(true_map.grid[start.y][start.x])) {
            continue;
        }
        int index = start.y * width + start.x;
        if (!visited[index]) {
            visited[index] = 1;
            pending.push(start);
        }
    }

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    int reachable = 0;
    while (!pending.empty()) {
        Position current = pending.front();
        pending.pop();
        ++reachable;
        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
                Environment::isOpaqueTile(true_map.grid[ny][nx])) {
                continue;
            }
            if (dx[i] != 0 && dy[i] != 0 &&
                (Environment::isOpaqueTile(true_map.grid[current.y][nx]) ||
                 Environment::isOpaqueTile(true_map.grid[ny][current.x]))) {
                continue;
            }
            int index = ny * width + nx;
            if (!visited[index]) {
                visited[index] = 1;
                pending.push({nx, ny});
            }
        }
    }
    return reachable;
}

int countCameraCoveredTiles(const Environment& map) {
    int count = 0;
    for (const auto& row : map.grid) {
        count += static_cast<int>(std::count(row.begin(), row.end(), 2));
    }
    return count;
}

int countLidarCoveredTiles(const Environment& map) {
    int count = 0;
    for (const auto& row : map.grid) {
        for (int value : row) count += (value == 0 || value == 2) ? 1 : 0;
    }
    return count;
}

void emitStats(const std::string& status, int total_steps, const Environment& known_map,
               const std::vector<DroneAgent>& agents, const TSDFMap&, const RunStats& stats) {
    int camera_covered_tiles = countCameraCoveredTiles(known_map);
    int lidar_covered_tiles = countLidarCoveredTiles(known_map);
    const auto& visit_grid = stats.exploration_snapshot_taken ? stats.exploration_visits : known_map.visits;
    int unique_visited_tiles = 0;
    int total_tile_visits = 0;
    int own_revisit_steps = 0;
    int inter_drone_overlap_tiles = 0;
    for (int y = 0; y < known_map.getHeight(); ++y) {
        for (int x = 0; x < known_map.getWidth(); ++x) {
            if (visit_grid[y][x] > 0) ++unique_visited_tiles;
            total_tile_visits += visit_grid[y][x];
            int distinct_visitors = 0;
            int index = y * known_map.getWidth() + x;
            for (size_t agent_index = 0; agent_index < agents.size(); ++agent_index) {
                const auto& own_visits = stats.exploration_snapshot_taken
                    ? stats.exploration_own_visits[agent_index]
                    : agents[agent_index].flat_own_visits;
                if (own_visits[index] > 0) ++distinct_visitors;
                own_revisit_steps += std::max(0, own_visits[index] - 1);
            }
            if (distinct_visitors >= 2) ++inter_drone_overlap_tiles;
        }
    }
    auto firstOrMinusOne = [](const std::vector<int>& values, bool all_found) {
        if (values.empty()) return -1;
        return all_found ? values.back() : values.front();
    };
    bool sar_all = stats.sar_discovery_steps.size() >= static_cast<size_t>(known_map.total_sar_objects);
    bool cp_all = stats.cp_discovery_steps.size() >= static_cast<size_t>(known_map.total_cp_objects);
    double avg_tsdf = stats.total_moves > 0 ? stats.tsdf_sum / stats.total_moves : 0.0;
    double lidar_coverage_pct = 100.0 * lidar_covered_tiles / std::max(1, stats.total_reachable_free_tiles);
    double camera_coverage_pct = 100.0 * camera_covered_tiles / std::max(1, stats.total_reachable_free_tiles);
    double lidar_overlap_ratio = static_cast<double>(stats.real_lidar_overlap) / std::max(1, lidar_covered_tiles);
    double camera_overlap_ratio = static_cast<double>(stats.real_camera_overlap) / std::max(1, camera_covered_tiles);
    double avg_planning_ms = stats.planning_calls > 0 ? stats.total_planning_ms / stats.planning_calls : 0.0;
    int first_sar = stats.sar_discovery_steps.empty() ? -1 : stats.sar_discovery_steps.front();
    int all_sar = firstOrMinusOne(stats.sar_discovery_steps, sar_all);
    int first_cp = stats.cp_discovery_steps.empty() ? -1 : stats.cp_discovery_steps.front();
    int all_cp = firstOrMinusOne(stats.cp_discovery_steps, cp_all);

    std::cout << "HEATMAP";
    for (const auto& row : visit_grid) {
        for (int value : row) std::cout << " " << value;
    }
    std::cout << "\n";

    std::cout << std::setprecision(10)
              << "STATS {\"status\":\"" << status
              << "\",\"total_steps\":" << total_steps
              << ",\"exploration_steps\":" << stats.exploration_steps
              << ",\"total_reachable_tiles\":" << stats.total_reachable_free_tiles
              << ",\"lidar_covered_tiles\":" << lidar_covered_tiles
              << ",\"lidar_coverage_pct\":" << lidar_coverage_pct
              << ",\"camera_covered_tiles\":" << camera_covered_tiles
              << ",\"camera_coverage_pct\":" << camera_coverage_pct
              << ",\"coverage_pct\":" << camera_coverage_pct
              << ",\"real_lidar_overlap\":" << stats.real_lidar_overlap
              << ",\"lidar_overlap_ratio\":" << lidar_overlap_ratio
              << ",\"real_camera_overlap\":" << stats.real_camera_overlap
              << ",\"camera_overlap_ratio\":" << camera_overlap_ratio
              << ",\"unique_visited_tiles\":" << unique_visited_tiles
              << ",\"total_tile_visits\":" << total_tile_visits
              << ",\"revisited_steps\":" << total_tile_visits - unique_visited_tiles
              << ",\"own_revisit_steps\":" << own_revisit_steps
              << ",\"inter_drone_overlap_tiles\":" << inter_drone_overlap_tiles
              << ",\"total_swarm_distance\":" << stats.total_swarm_distance
              << ",\"total_hovers\":" << stats.total_hovers
              << ",\"turns_0_deg\":" << stats.turns_0_deg
              << ",\"turns_45_deg\":" << stats.turns_45_deg
              << ",\"turns_90_deg\":" << stats.turns_90_deg
              << ",\"turns_135_deg\":" << stats.turns_135_deg
              << ",\"turns_180_deg\":" << stats.turns_180_deg
              << ",\"avg_tsdf_clearance\":" << avg_tsdf
              << ",\"min_tsdf_clearance\":" << (std::isfinite(stats.min_tsdf) ? stats.min_tsdf : 0.0)
              << ",\"tsdf_exposure_pct\":" << (100.0 * stats.tsdf_violation_steps / std::max(1, stats.total_moves))
              << ",\"total_planning_ms\":" << stats.total_planning_ms
              << ",\"avg_planning_ms\":" << avg_planning_ms
              << ",\"planning_calls\":" << stats.planning_calls
              << ",\"sar_found\":" << known_map.discovered_sar.size()
              << ",\"sar_total\":" << known_map.total_sar_objects
              << ",\"time_to_first_sar\":" << first_sar
              << ",\"time_to_all_sar\":" << all_sar
              << ",\"cp_found\":" << known_map.discovered_cp.size()
              << ",\"cp_total\":" << known_map.total_cp_objects
              << ",\"time_to_first_cp\":" << first_cp
              << ",\"time_to_all_cp\":" << all_cp << "}\n" << std::flush;
}

int main() {
    int width, height, num_drones, ui_fov, camera_fov;
    int sar_en, sar_rtl_flag, cp_en, cp_coords_flag, cp_rtl_flag;
    if (!(std::cin >> width >> height >> num_drones >> ui_fov >> camera_fov
                   >> sar_en >> sar_rtl_flag >> cp_en >> cp_coords_flag >> cp_rtl_flag)) {
        return 0;
    }

    num_drones = std::max(1, num_drones);
    std::vector<DroneAgent> agents;
    for (int i = 0; i < num_drones; ++i) {
        int sx, sy;
        std::cin >> sx >> sy;
        agents.emplace_back(i, Position{sx, sy}, width, height);
    }

    Environment true_map(width, height, 0, ui_fov, camera_fov);
    Environment known_map(width, height, -1, ui_fov, camera_fov);
    TSDFMap tsdf(width, height, 5.0);

    known_map.sar_enabled = (sar_en != 0);
    known_map.sar_rtl = (sar_rtl_flag != 0);
    known_map.cp_enabled = (cp_en != 0);
    known_map.cp_coords_known = (cp_coords_flag != 0);
    known_map.cp_rtl = (cp_rtl_flag != 0);

    int max_game_steps = 3000;
    constexpr int K_CHAIN_STEPS = 3;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int tile;
            std::cin >> tile;
            true_map.setTile(x, y, tile);
            if (tile == 3 && known_map.sar_enabled) {
                known_map.total_sar_objects++;
            } else if (tile == 4 && known_map.cp_enabled) {
                known_map.total_cp_objects++;
                if (known_map.cp_coords_known) {
                    known_map.known_uncollected_cp.insert({x, y});
                }
            }
        }
    }

    RunStats run_stats;
    run_stats.total_reachable_free_tiles = countReachableFreeTiles(true_map, agents);
    int total_cells = width * height;
    run_stats.prev_lidar_fov.assign(num_drones, std::vector<uint8_t>(total_cells, 0));
    run_stats.prev_cam_fov.assign(num_drones, std::vector<uint8_t>(total_cells, 0));
    run_stats.ever_lidar_seen.assign(total_cells, 0);
    run_stats.ever_cam_seen.assign(total_cells, 0);

    std::cout << "MAP LOADED. STARTING OPTIMIZED DEC-MCTS LOOP WITH " << num_drones << " DRONES...\n" << std::flush;

    for (auto& ag : agents) {
        std::vector<Position> init_walls = known_map.updateLidarFOV(ag.drone_pos.x, ag.drone_pos.y, true_map.grid);
        tsdf.updateTSDF(init_walls);
        known_map.observePhysicalCameraFOV(ag.drone_pos.x, ag.drone_pos.y, true_map.grid);
        recordRealOverlapForDrone(ag.id, ag.drone_pos, known_map, true_map.grid, run_stats);
        int& initial_visits = known_map.visits[ag.drone_pos.y][ag.drone_pos.x];
        if (initial_visits == 0) ++run_stats.unique_visited_tiles;
        ++initial_visits;
        ++run_stats.total_tile_visits;
        std::cout << "MOVE " << ag.id << " " << ag.drone_pos.x << " " << ag.drone_pos.y << "\n" << std::flush;
    }
    known_map.invalidateAllCellCaches();
    tsdf.TSDFToVisualiser();
    known_map.rebuildFrontiers(tsdf, &true_map.grid);
    size_t previous_sar_count = known_map.discovered_sar.size();
    size_t previous_cp_count = known_map.discovered_cp.size();

    bool global_rtl_active = false;
    std::mt19937 sim_gen(std::random_device{}());
    const double safe_clearance = std::max(1.0, static_cast<double>(camera_fov - 1.0));

    for (int game_step = 0; game_step < max_game_steps; ++game_step) {

        std::vector<Position> current_positions;
        for (const auto& ag : agents) current_positions.push_back(ag.drone_pos);

        if (!global_rtl_active && known_map.shouldTriggerRTL(current_positions, tsdf)) {
            global_rtl_active = true;
            if (run_stats.exploration_steps < 0) {
                run_stats.exploration_steps = game_step;
                run_stats.exploration_visits = known_map.visits;
                run_stats.exploration_own_visits.clear();
                for (const auto& agent : agents) {
                    run_stats.exploration_own_visits.push_back(agent.flat_own_visits);
                }
                run_stats.exploration_snapshot_taken = true;
            }
            std::cout << "FRONTIERS \n" << std::flush;
            std::cout << "MISSION COMPLETE. ALL UAVS RETURNING TO LAUNCH.\n" << std::flush;

            for (auto& ag : agents) {
                ag.rtl_active = true;
                ag.current_plan.clear();
                ag.best_full_plan.clear();
                ag.sparse_domain.clear();
                ag.mcts_root.reset();
                ag.rtl_path = calculateAStar(ag.drone_pos, ag.home_pos, known_map, tsdf);
            }
        }

        bool all_home = global_rtl_active;
        for (const auto& ag : agents) {
            if (ag.drone_pos != ag.home_pos) {
                all_home = false;
                break;
            }
        }
        if (all_home) {
            for (const auto& ag : agents) {
                std::cout << "ASTAR_PATH " << ag.id << " \n" << std::flush;
            }
            emitStats("DONE", game_step, known_map, agents, tsdf, run_stats);
            std::cout << "DONE\n" << std::flush;
            break;
        }

        // --- PHASE 1: PLANNING ---
        if (global_rtl_active) {
            for (auto& ag : agents) {
                if (ag.drone_pos == ag.home_pos) continue;
                if (ag.current_plan.empty()) {
                    if (ag.rtl_path.empty()) {
                        ag.rtl_path = calculateAStar(ag.drone_pos, ag.home_pos, known_map, tsdf);
                    }
                    if (!ag.rtl_path.empty()) {
                        ag.current_plan.push_back(ag.rtl_path.front());
                        ag.rtl_path.erase(ag.rtl_path.begin());
                    }
                }
                std::cout << "ASTAR_PATH " << ag.id << " ";
                for (const auto& pos : ag.rtl_path) {
                    std::cout << pos.x << " " << pos.y << " ";
                }
                std::cout << "\n" << std::flush;
            }
        } else {
            std::vector<int> drones_needing_plan;
            for (const auto& ag : agents) {
                if (ag.current_plan.empty()) {
                    drones_needing_plan.push_back(ag.id);
                }
            }

            if (!drones_needing_plan.empty()) {
                auto planning_start = std::chrono::steady_clock::now();
                runDecMCTSForSwarm(drones_needing_plan, agents, known_map, tsdf, sim_gen, 10, 60, 8, 10, K_CHAIN_STEPS);
                auto planning_end = std::chrono::steady_clock::now();
                run_stats.total_planning_ms += std::chrono::duration<double, std::milli>(planning_end - planning_start).count();
                ++run_stats.planning_calls;

                std::cout << "FRONTIERS ";
                for (const auto& frontier : known_map.active_frontiers) {
                    std::cout << frontier.x << " " << frontier.y << " ";
                }
                std::cout << "\n" << std::flush;
            }
        }

        // --- PHASE 2: EXECUTION & COLLISION-FREE SYMMETRIC DECONFLICTION ---
        std::vector<int> exec_order(num_drones);
        for (int i = 0; i < num_drones; ++i) exec_order[i] = i;
        std::sort(exec_order.begin(), exec_order.end(), [&](int a, int b) {
            return agents[a].winning_expected_utility > agents[b].winning_expected_utility;
        });

        std::set<Position> reserved_next_tiles;
        std::vector<std::pair<Position, Position>> reserved_transitions;

        for (const auto& ag : agents) {
            if (global_rtl_active && ag.drone_pos == ag.home_pos) {
                reserved_next_tiles.insert(ag.drone_pos);
            }
        }

// Replace from: for (size_t order_idx = 0; order_idx < exec_order.size(); ++order_idx)
        // Down to:      for (const auto& ag : agents) { if (!global_rtl_active) ...
        for (size_t order_idx = 0; order_idx < exec_order.size(); ++order_idx) {
            int id = exec_order[order_idx];
            DroneAgent& ag = agents[id];

            if (global_rtl_active && ag.drone_pos == ag.home_pos) {
                continue;
            }

            std::set<Position> blocked_tiles = reserved_next_tiles;
            Position chosen_next = ag.drone_pos;
            bool step_valid = false;
            bool is_hover = false;
            bool reached_truncated_corridor_end = false;

            // CASE 1: Execute the scheduled 1-frame hover at the end of a truncated 1-tile corridor path
            if (!global_rtl_active && ag.pending_hover_at_path_end) {
                ag.pending_hover_at_path_end = false;
                ag.current_plan.clear();
                ag.best_full_plan.clear();
                ag.sparse_domain.clear();
                ag.mcts_root.reset();
                if (blocked_tiles.count(ag.drone_pos) == 0) {
                    chosen_next = ag.drone_pos;
                    step_valid = true;
                    is_hover = true;
                }
            }

            // CASE 2: Normal plan execution
            if (!step_valid && !ag.current_plan.empty()) {
                Position candidate = ag.current_plan.front();
                const auto legal_moves = known_map.getLegalMoves(ag.drone_pos.x, ag.drone_pos.y, true, tsdf);
                bool move_is_legal = std::find(legal_moves.begin(), legal_moves.end(), candidate) != legal_moves.end();

                bool tile_conflict = blocked_tiles.count(candidate) > 0;
                bool swap_conflict = false;
                for (const auto& tr : reserved_transitions) {
                    if (tr.first == candidate && tr.second == ag.drone_pos) {
                        swap_conflict = true;
                        break;
                    }
                }

                if (move_is_legal && !tile_conflict && !swap_conflict) {
                    // Check if taking this step consumes the VERY LAST step of a truncated global path
                    // AND no other longer candidate paths exist in sparse_domain
                    if (!global_rtl_active &&
                        ag.best_full_plan.size() == 1 &&
                        ag.best_full_plan.front() == candidate) {

                        size_t max_other_sparse_len = 0;
                        for (const auto& cand : ag.sparse_domain) {
                            max_other_sparse_len = std::max(max_other_sparse_len, cand.path.size());
                        }

                        if (max_other_sparse_len <= 1) {
                            int mdx = candidate.x - ag.drone_pos.x;
                            int mdy = candidate.y - ag.drone_pos.y;
                            const auto target_legal = known_map.getLegalMoves(candidate.x, candidate.y, true, tsdf);

                            // Check if the destination tile is in a 1-tile-wide corridor, blind turn, or dead end
                            bool left_wall_or_fog  = !known_map.isSafe(candidate.x - mdy, candidate.y + mdx, true);
                            bool right_wall_or_fog = !known_map.isSafe(candidate.x + mdy, candidate.y - mdx, true);
                            bool front_wall_or_fog = !known_map.isSafe(candidate.x + mdx, candidate.y + mdy, true);

                            if (target_legal.size() <= 2 ||
                                (left_wall_or_fog && right_wall_or_fog) ||
                                (front_wall_or_fog && (left_wall_or_fog || right_wall_or_fog))) {
                                reached_truncated_corridor_end = true;
                            }
                        }
                    }

                    chosen_next = candidate;
                    ag.current_plan.pop_front();
                    step_valid = true;
                } else {
                    ag.current_plan.clear();
                    ag.best_full_plan.clear();
                    ag.sparse_domain.clear();
                    if (global_rtl_active) {
                        std::set<Position> rtl_obs = blocked_tiles;
                        for (size_t j = order_idx + 1; j < exec_order.size(); ++j) {
                            rtl_obs.insert(agents[exec_order[j]].drone_pos);
                        }
                        ag.rtl_path = calculateAStar(ag.drone_pos, ag.home_pos, known_map, tsdf, rtl_obs);
                        if (!ag.rtl_path.empty()) {
                            Position alt = ag.rtl_path.front();
                            ag.rtl_path.erase(ag.rtl_path.begin());
                            bool alt_swap = false;
                            for (const auto& tr : reserved_transitions) {
                                if (tr.first == alt && tr.second == ag.drone_pos) alt_swap = true;
                            }
                            if (blocked_tiles.count(alt) == 0 && !alt_swap) {
                                chosen_next = alt;
                                step_valid = true;
                            }
                        }
                    }
                }
            }

            // CASE 3: Fallback move selection (NEVER hovers in 3-tile corridors or open space!)
            if (!step_valid) {
                ag.current_plan.clear();
                ag.best_full_plan.clear();
                ag.sparse_domain.clear();

                const auto legal_moves = known_map.getLegalMoves(ag.drone_pos.x, ag.drone_pos.y, true, tsdf);
                double best_fallback_score = -std::numeric_limits<double>::infinity();
                int in_dx = (ag.prev_pos.x != -1) ? (ag.drone_pos.x - ag.prev_pos.x) : 0;
                int in_dy = (ag.prev_pos.y != -1) ? (ag.drone_pos.y - ag.prev_pos.y) : 0;
                int in2_dx = (ag.prev2_pos.x != -1 && ag.prev_pos.x != -1) ? (ag.prev_pos.x - ag.prev2_pos.x) : 0;
                int in2_dy = (ag.prev2_pos.y != -1 && ag.prev_pos.y != -1) ? (ag.prev_pos.y - ag.prev2_pos.y) : 0;

                // Filter fallback moves through the kinematic cone first so fallback never yanks >90 deg in open space
                std::vector<Position> cone_Check = filterKinematicConeMoves(
                    ag.drone_pos.x, ag.drone_pos.y, in_dx, in_dy, legal_moves, tsdf, safe_clearance);
                const auto& pool = !cone_Check.empty() ? cone_Check : legal_moves;

                for (const auto& m : pool) {
                    if (blocked_tiles.count(m) > 0) continue;
                    bool swap_c = false;
                    for (const auto& tr : reserved_transitions) {
                        if (tr.first == m && tr.second == ag.drone_pos) {
                            swap_c = true;
                            break;
                        }
                    }
                    if (swap_c) continue;

                    double s = computeKinematicSmoothness(in2_dx, in2_dy, in_dx, in_dy, m.x - ag.drone_pos.x, m.y - ag.drone_pos.y);

                    double m_tsdf = tsdf.getTSDFValue(m.x, m.y);
                    if (m_tsdf < safe_clearance) {
                        double def = safe_clearance - m_tsdf;
                        s -= 1100.0 * def * def;
                    }
                    s -= tsdf.getLateralCenteringPenalty(ag.drone_pos.x, ag.drone_pos.y, m.x, m.y, safe_clearance, 500.0);

                    if (global_rtl_active) {
                        s += 500.0 * (calculateHeuristic(ag.drone_pos, ag.home_pos) - calculateHeuristic(m, ag.home_pos))
                           - 100.0 * ag.flat_own_visits[m.y * width + m.x];
                    } else {
                        s += -300.0 * ag.flat_own_visits[m.y * width + m.x]
                           + 800.0 * (known_map.getDistanceToNearestUncovered(ag.drone_pos.x, ag.drone_pos.y) -
                                      known_map.getDistanceToNearestUncovered(m.x, m.y))
                           + known_map.bias(ag.drone_pos.x, ag.drone_pos.y, m.x, m.y, 1000.0);
                    }
                    if (m == ag.prev_pos) s -= 800.0;
                    if (m == ag.prev2_pos) s -= 500.0;
                    if (s > best_fallback_score) {
                        best_fallback_score = s;
                        chosen_next = m;
                        step_valid = true;
                    }
                }
            }

            Position old_pos = ag.drone_pos;
            Position old_prev_pos = ag.prev_pos;
            reserved_next_tiles.insert(chosen_next);
            reserved_transitions.push_back({ag.drone_pos, chosen_next});

            if (is_hover) {
                // 1-frame hover resets velocity vectors so the next turn plans 360-deg freely around the corner
                ag.prev2_pos = {-1, -1};
                ag.prev_pos = {-1, -1};
            } else {
                ag.prev2_pos = ag.prev_pos;
                ag.prev_pos = ag.drone_pos;
                ag.drone_pos = chosen_next;
            }

            if (!ag.best_full_plan.empty() && ag.best_full_plan.front() == chosen_next) {
                ag.best_full_plan.erase(ag.best_full_plan.begin());
            }
            for (auto& cand : ag.sparse_domain) {
                if (!cand.path.empty() && cand.path.front() == chosen_next) {
                    cand.path.erase(cand.path.begin());
                }
            }

            if (ag.mcts_root && !is_hover) {
                ag.mcts_root = ag.mcts_root->extractChild(ag.drone_pos);
            }

            if (!global_rtl_active) {
                if (!is_hover) {
                double move_length = (chosen_next.x != old_pos.x && chosen_next.y != old_pos.y) ? 1.41421356 : 1.0;
                ++run_stats.total_moves;
                run_stats.total_swarm_distance += move_length;
                double clearance = tsdf.getTSDFValue(chosen_next.x, chosen_next.y);
                run_stats.tsdf_sum += clearance;
                run_stats.min_tsdf = std::min(run_stats.min_tsdf, clearance);
                if (clearance < safe_clearance) ++run_stats.tsdf_violation_steps;
                if (old_prev_pos.x != -1) {
                    int in_dx = old_pos.x - old_prev_pos.x;
                    int in_dy = old_pos.y - old_prev_pos.y;
                    int out_dx = chosen_next.x - old_pos.x;
                    int out_dy = chosen_next.y - old_pos.y;
                    double in_length = (in_dx != 0 && in_dy != 0) ? 1.41421356 : 1.0;
                    double out_length = (out_dx != 0 && out_dy != 0) ? 1.41421356 : 1.0;
                    double cos_turn = (in_dx * out_dx + in_dy * out_dy) / (in_length * out_length);
                    if (cos_turn > 0.95) ++run_stats.turns_0_deg;
                    else if (cos_turn > 0.50) ++run_stats.turns_45_deg;
                    else if (cos_turn > -0.10) ++run_stats.turns_90_deg;
                    else if (cos_turn > -0.80) ++run_stats.turns_135_deg;
                    else ++run_stats.turns_180_deg;
                }
                ag.flat_own_visits[chosen_next.y * width + chosen_next.x] += 1;
                int& visits = known_map.visits[chosen_next.y][chosen_next.x];
                if (visits == 0) ++run_stats.unique_visited_tiles;
                ++visits;
                ++run_stats.total_tile_visits;
                } else {
                    ++run_stats.total_hovers;
                }
            }

            // If this step just landed on the final tile of a truncated 1-tile corridor path,
            // pre-load a 1-step hover into current_plan so the drone pauses 1 frame on Turn t+1!
            if (reached_truncated_corridor_end) {
                ag.pending_hover_at_path_end = true;
                ag.current_plan.clear();
                ag.current_plan.push_back(ag.drone_pos); // Pre-load hover step so Phase 1 doesn't replan before hovering
                ag.best_full_plan.clear();
                ag.sparse_domain.clear();
            }
        }

        bool any_new_walls = false;
        for (auto& ag : agents) {
            std::vector<Position> new_walls = known_map.updateLidarFOV(ag.drone_pos.x, ag.drone_pos.y, true_map.grid);
            if (!new_walls.empty()) {
                tsdf.updateTSDF(new_walls);
                any_new_walls = true;
            }
            known_map.observePhysicalCameraFOV(ag.drone_pos.x, ag.drone_pos.y, true_map.grid);
            if (!global_rtl_active) {
                recordRealOverlapForDrone(ag.id, ag.drone_pos, known_map, true_map.grid, run_stats);
            }
        }

        if (any_new_walls) {
            known_map.invalidateAllCellCaches();
            tsdf.TSDFToVisualiser();
        }
        known_map.rebuildFrontiers(tsdf, &true_map.grid);

        if (known_map.discovered_sar.size() > previous_sar_count) {
            for (size_t count = previous_sar_count; count < known_map.discovered_sar.size(); ++count) {
                run_stats.sar_discovery_steps.push_back(game_step + 1);
            }
            previous_sar_count = known_map.discovered_sar.size();
        }
        if (known_map.discovered_cp.size() > previous_cp_count) {
            for (size_t count = previous_cp_count; count < known_map.discovered_cp.size(); ++count) {
                run_stats.cp_discovery_steps.push_back(game_step + 1);
            }
            previous_cp_count = known_map.discovered_cp.size();
        }

        for (auto& ag : agents) {
            if (ag.mcts_root && any_new_walls) {
                ag.mcts_root->pruneInvalidNodes(known_map, tsdf);
            }

            // Skip plan validation/abort if a 1-frame corridor hover is queued for next step
            if (ag.pending_hover_at_path_end) {
                continue;
            }

            if (!global_rtl_active && !ag.current_plan.empty()) {
                const std::vector<Position>* evaluation_path = &ag.best_full_plan;
                std::vector<Position> fallback_path;
                if (evaluation_path->empty()) {
                    fallback_path.assign(ag.current_plan.begin(), ag.current_plan.end());
                    evaluation_path = &fallback_path;
                }
                Position last_planned = evaluation_path->back();
                int total_chain_new = 0;
                for (const auto& step_pos : *evaluation_path) {
                    Environment::SensorObservation cam_est = known_map.estimateCameraFOV(step_pos.x, step_pos.y, tsdf);
                    total_chain_new += cam_est.newly_covered + cam_est.known_cp_covered;
                    total_chain_new += known_map.estimateLidarFOV(step_pos.x, step_pos.y, tsdf).newly_covered;
                }

                double dist_now = known_map.getDistanceToNearestUncovered(ag.drone_pos.x, ag.drone_pos.y);
                double dist_end = known_map.getDistanceToNearestUncovered(last_planned.x, last_planned.y);
                double cp_bias_chain = known_map.bias(ag.drone_pos.x, ag.drone_pos.y, last_planned.x, last_planned.y, 1.0);

                if (total_chain_new == 0 && dist_end > dist_now + 1.5 && cp_bias_chain <= 0.0) {
                    ag.current_plan.clear();
                    ag.best_full_plan.clear();
                    ag.sparse_domain.clear();
                }
            }

            Position check_curr = ag.drone_pos;
            for (const auto& upcoming : ag.current_plan) {
                const auto step_legal = known_map.getLegalMoves(check_curr.x, check_curr.y, true, tsdf);
                if (std::find(step_legal.begin(), step_legal.end(), upcoming) == step_legal.end() ||
                    known_map.grid[upcoming.y][upcoming.x] == 1) {
                    ag.current_plan.clear();
                    ag.best_full_plan.clear();
                    ag.sparse_domain.clear();
                    break;
                }
                check_curr = upcoming;
            }
        }

        for (const auto& ag : agents) {
            if (!global_rtl_active) {
                std::cout << "PATH " << ag.id << " ";
                for (const auto& step : ag.current_plan) {
                    std::cout << step.x << " " << step.y << " ";
                }
                std::cout << "\n" << std::flush;

                std::cout << "BEST_PATH " << ag.id << " ";
                for (const auto& step : ag.best_full_plan) {
                    std::cout << step.x << " " << step.y << " ";
                }
                std::cout << "\n" << std::flush;

                std::cout << "SPARSE_PATHS " << ag.id << " ";
                for (size_t k = 0; k < ag.sparse_domain.size(); ++k) {
                    if (k > 0) std::cout << "| ";
                    for (const auto& step : ag.sparse_domain[k].path) {
                        std::cout << step.x << " " << step.y << " ";
                    }
                }
                std::cout << "\n" << std::flush;
            }
            std::cout << "MOVE " << ag.id << " " << ag.drone_pos.x << " " << ag.drone_pos.y << "\n" << std::flush;
        }

                if (!global_rtl_active) {
                        std::cout << "METRIC " << game_step + 1 << " "
              << countLidarCoveredTiles(known_map) << " "
              << countCameraCoveredTiles(known_map) << " "
              << run_stats.total_reachable_free_tiles << " "
              << run_stats.total_tile_visits - run_stats.unique_visited_tiles << " "
              << known_map.discovered_sar.size() << " "
              << known_map.discovered_cp.size() << "\n" << std::flush;
          }

        if (game_step == max_game_steps - 1) {
            emitStats("STUCK", max_game_steps, known_map, agents, tsdf, run_stats);
            std::cout << "STUCK\n" << std::flush;
        }
    }
    return 0;
}
