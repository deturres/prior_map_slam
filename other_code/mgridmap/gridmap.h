/*
 * Consist, a software for checking map consistency in SLAM
 * Copyright (C) 2013-2014 Mladen Mazuran and Gian Diego Tipaldi and
 * Luciano Spinello and Wolfram Burgard and Cyrill Stachniss
 *
 * This file is part of Consist.
 *
 * Consist is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Consist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Consist.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONSIST_GRIDMAP_H_
#define CONSIST_GRIDMAP_H_

#include <vector>
#include "support.h"

namespace consist {

class Visibility;

struct GridMapCell
{
    enum StatusType {
        Empty = 0, Filled, Boundary
    };

    double distance;
    StatusType status;
};

class GridMap
{
public:
    GridMap(double xmin, double ymin, double xmax, double ymax, double gridsize);
    GridMap(const Visibility &vis, double gridsize);
    virtual ~GridMap();

    GridMapCell &operator()(int x, int y);
    GridMapCell &operator()(double x, double y);
    GridMapCell &operator()(const Point &p);
    GridMapCell &cell(int x, int y);
    GridMapCell &cell(double x, double y);
    GridMapCell &cell(const Point &p);

    const GridMapCell &operator()(int x, int y) const;
    const GridMapCell &operator()(double x, double y) const;
    const GridMapCell &operator()(const Point &p) const;
    const GridMapCell &cell(int x, int y) const;
    const GridMapCell &cell(double x, double y) const;
    const GridMapCell &cell(const Point &p) const;

    bool cellexists(int x, int y) const;

    double xmin() const;
    double ymin() const;
    double xmax() const;
    double ymax() const;
    double gridsize() const;

    bool valid(const Point &p) const;
    bool valid(double x, double y) const;

    int xcells() const;
    int ycells() const;

    int xcell(double x) const;
    int ycell(double y) const;
    int xcell(const Point &p) const;
    int ycell(const Point &p) const;

    double xcellstart(int cellx) const;
    double ycellstart(int celly) const;

    void rasterize(const Visibility &vis);
    void rasterizeLine(const Point &from, const Point &to);
    void scanFill(const Visibility &vis);
    void computeDistanceTransform();

    void savePGM(const char *fname) const;

protected:
    std::vector<double> distanceTransform1D(const std::vector<double> &vec) const;

private:
    double _xmin, _ymin;
    double _gridsize;
    int _xcells, _ycells;
    GridMapCell *_grid;
};

} /* namespace consist */

#endif /* CONSIST_GRIDMAP_H_ */
