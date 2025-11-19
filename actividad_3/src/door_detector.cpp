//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"

#include <expected>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>

Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Doors doors;
    doors_cache.clear();

    if (points.size() < 2)
        return doors;

    // ========= 1) Detectar "peaks" con sliding_window(2) =========
    Peaks peaks;

    for (auto &&win : iter::sliding_window(points, 2))
    {
        const auto &p1 = win[0];
        const auto &p2 = win[1];

        const float diff = std::abs(p1.distance2d - p2.distance2d);
        if (diff > 1000.f)   // umbral del enunciado
        {
            // nos quedamos con el más cercano
            const auto &pshort = (p1.distance2d < p2.distance2d) ? p1 : p2;
            Eigen::Vector2f pos(pshort.x, pshort.y);
            float angle = pshort.phi;

            peaks.emplace_back(pos, angle);
        }
    }

    // ========= 2) Dibujar peaks (para depurar) =========
    if (scene)
    {
        static std::vector<QGraphicsItem*> drawn_peaks;

        for (auto *item : drawn_peaks)
        {
            scene->removeItem(item);
            delete item;
        }
        drawn_peaks.clear();

        QPen pen(Qt::yellow);
        pen.setWidth(10);

        for (const auto &[p, a] : peaks)
        {
            Q_UNUSED(a);
            auto item = scene->addRect(-20, -20, 40, 40, pen);
            item->setPos(p.x(), p.y());
            drawn_peaks.push_back(item);
        }
    }

    // ========= 3) Non-maximum suppression (NMS) de peaks =========
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
    {
        bool too_close = false;
        for (const auto &[q, aq] : nms_peaks)
        {
            Q_UNUSED(aq);
            if ((p - q).norm() < 500.f)
            {
                too_close = true;
                break;
            }
        }
        if (not too_close)
            nms_peaks.emplace_back(p, a);
    }
    peaks = std::move(nms_peaks);

    // ========= 4) Formar puertas a partir de parejas de peaks =========
    for (auto &&comb : iter::combinations(peaks, 2))
    {
        const auto &[p1, a1] = comb[0];
        const auto &[p2, a2] = comb[1];

        const float dist = (p1 - p2).norm();
        if (dist >= 800.f && dist <= 1200.f)   // rango puerta
        {
            // Door(Eigen::Vector2f point1, float angle1, Eigen::Vector2f point2, float angle2)
            Door d{p1, a1, p2, a2};
            doors.emplace_back(d);
        }
    }

    doors_cache = doors;
    return doors;
}

// Method to use the Doors vector to filter out the LiDAR points that come from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            filtered.emplace_back(p);
        }
    }
    return filtered;
}

std::expected<Door, std::string> DoorDetector::get_current_door() const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    return doors_cache[0];
}
