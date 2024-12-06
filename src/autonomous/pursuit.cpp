#include <autonomous/pursuit.hpp>
#include <paths/path.hpp>
#include <algorithm>
#include <utility>
#include <vector>
#include <cmath>

Pursuit::Pursuit(Path path, double lookahead_distance): lookahead(lookahead_distance), path(path) {
    last_found_idx = 0;
}

Pursuit::Pursuit(std::vector<double> x, std::vector<double> y, int point_count, double lookahead_distance): lookahead(lookahead_distance), path(x, y, point_count) {
    last_found_idx = 0;
}

double & Pursuit::lookahead_distance() {
    return lookahead;
}

Coordinate2D Pursuit::get_target(double from_x, double from_y) {
    std::vector<std::pair<int, Coordinate2D>> found_targets;
    if (path.distance_to_end(from_x, from_y) < lookahead) {
        return path.points.back();
    }
    for (int segment = last_found_idx; segment < path.points.size(); ++segment) {
        double x1, y1, x2, y2;
        bool point_1_is_valid = false, point_2_is_valid = false;
        if (path.points[segment].x == path.points[segment - 1].x) {
            double determinant = lookahead * lookahead - (path.points[segment].x - from_x) * (path.points[segment].x - from_x);
            if (determinant < 0) {
                continue;
            }
            x1 = x2 = path.points[segment].x;
            y1 = from_y + sqrt(determinant);
            y2 = from_y - sqrt(determinant);
            if ((path.points[segment - 1].y <= y1 and y1 <= path.points[segment].y) or (path.points[segment].y <= y1 and y1 <= path.points[segment - 1].y)) {
                point_1_is_valid = true;
            }
            if ((path.points[segment - 1].y <= y2 and y2 <= path.points[segment].y) or (path.points[segment].y <= y2 and y2 <= path.points[segment - 1].y)) {
                point_2_is_valid = true;
            }
        } else {
            double slope = (path.points[segment].y - path.points[segment - 1].y) / (path.points[segment].x - path.points[segment - 1].x);
            double y_intercept = path.points[segment].y - slope * path.points[segment].x;
            double leading_coefficient = slope * slope + 1.0;
            double linear_coefficient = (slope * y_intercept - slope * from_y - from_x) * 2.0;
            double constant = from_x * from_x + from_y * from_y + y_intercept * y_intercept - lookahead * lookahead - y_intercept * from_y * 2.0;
            double determinant = linear_coefficient * linear_coefficient - leading_coefficient * constant * 4.0;
            if (determinant < 0) {
                continue;
            }
            x1 = (-linear_coefficient + sqrt(determinant)) / (leading_coefficient * 2.0);
            y1 = slope * x1 + y_intercept;
            x2 = (-linear_coefficient - sqrt(determinant)) / (leading_coefficient * 2.0);
            y2 = slope * x2 + y_intercept;
            if ((path.points[segment - 1].x <= x1 and x1 <= path.points[segment].x) or (path.points[segment].x <= x1 and x1 <= path.points[segment - 1].x)) {
                point_1_is_valid = true;
            }
            if ((path.points[segment - 1].x <= x2 and x2 <= path.points[segment].x) or (path.points[segment].x <= x2 and x2 <= path.points[segment - 1].x)) {
                point_2_is_valid = true;
            }
        }
        if (point_1_is_valid and point_2_is_valid) {
            if (sqrt((x1 - path.points[segment].x) * (x1 - path.points[segment].x) + (y1 - path.points[segment].y) * (y1 - path.points[segment].y)) < sqrt((x2 - path.points[segment].x) * (x2 - path.points[segment].x) + (y2 - path.points[segment].y) * (y2 - path.points[segment].y))) {
                found_targets.push_back(std::make_pair(segment, Coordinate2D(x1, y1)));
            } else {
                found_targets.push_back(std::make_pair(segment, Coordinate2D(x2, y2)));
            }
        } else if (point_1_is_valid) {
            found_targets.push_back(std::make_pair(segment, Coordinate2D(x1, y1)));
        } else if (point_2_is_valid) {
            found_targets.push_back(std::make_pair(segment, Coordinate2D(x2, y2)));
        }
    }
    int last_found_index = last_found_idx;
    auto comparator = [last_found_index] (std::pair<int, Coordinate2D> pair_1, std::pair<int, Coordinate2D> pair_2) -> bool {
        auto rerank = [last_found_index] (int idx) -> int {
            return abs((last_found_index + 1) - idx);
        };
        return rerank(pair_1.first) < rerank(pair_2.first);
    };
    std::sort(found_targets.begin(), found_targets.end(), comparator);
    if (not found_targets.empty()) {
        auto [target_idx, target_point] = found_targets.front();
        last_found_idx = target_idx;
        return target_point;
    }
    double minimum_distance = 1e9;
    int minimum_index = -1;
    for (int i = 0; i < path.points.size(); ++i) {
        double distance = sqrt((path.points[i].x - from_x) * (path.points[i].x - from_x) + (path.points[i].y - from_y) * (path.points[i].y - from_y));
        if (distance < minimum_distance) {
            minimum_distance = distance;
            minimum_index = i;
        }
    }
    if (minimum_index != -1) {
        return path.points[minimum_index];
    }
    throw std::runtime_error("Something went wrong");
}

std::pair<double, double> Pursuit::get_relative_steering(double from_x, double from_y, double theta_bot, double width_bot) {
    Coordinate2D target = get_target(from_x, from_y);
    double dx = target.x - from_x;
    double dy = target.y - from_y;
    double angle = atan2(dy, dx);
    double alpha = angle - theta_bot;
    double sine = sin(alpha);
    if (sine == 0) {
        return std::make_pair(50.0, 50.0);
    }
    double distance = sqrt(dx * dx + dy * dy);
    double central_radius = distance / sine * 0.5;
    double left_steering = central_radius - width_bot * 0.5; 
    double right_steering = central_radius + width_bot * 0.5;
    double scale = 100.0 / (left_steering + right_steering);
    if (cos(alpha) < 0) {
        scale = -scale;
    }
    return std::make_pair(left_steering * scale, right_steering * scale);
}

// Unit testing result: https://www.desmos.com/calculator/0wlkkn4vn2