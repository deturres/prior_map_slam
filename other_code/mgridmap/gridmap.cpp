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

#include "gridmap.h"
#include "visibility.h"
#include "sanity.h"
#include <iostream>
#include <fstream>

#ifdef CONSIST_SEGMENT_TREE_LOOKUP
#   include <set>
#endif /* CONSIST_SEGMENT_TREE_LOOKUP */

namespace consist {

GridMap::GridMap(
        double xmin, double ymin, double xmax, double ymax, double gridsize) :
    _xmin(std::floor(xmin / gridsize) * gridsize - gridsize),
    _ymin(std::floor(ymin / gridsize) * gridsize - gridsize),
    _gridsize(gridsize),
    _xcells(std::ceil((xmax - xmin) / gridsize) + 2),
    _ycells(std::ceil((ymax - ymin) / gridsize) + 2),
    _grid(new GridMapCell[_xcells * _ycells])
{
}

GridMap::GridMap(const Visibility &vis, double gridsize) :
    _xmin(std::floor(vis.xmin() / gridsize) * gridsize - gridsize),
    _ymin(std::floor(vis.ymin() / gridsize) * gridsize - gridsize),
    _gridsize(gridsize),
    _xcells(std::ceil((vis.xmax() - vis.xmin()) / gridsize) + 2),
    _ycells(std::ceil((vis.ymax() - vis.ymin()) / gridsize) + 2),
    _grid(new GridMapCell[_xcells * _ycells])
{
    rasterize(vis);
    computeDistanceTransform();
}

GridMap::~GridMap()
{
    delete[] _grid;
}

/* ---------------------------------------------------------------------------------------------- */

GridMapCell &GridMap::operator()(int x, int y)
{
    return cell(x, y);
}

GridMapCell &GridMap::operator()(double x, double y)
{
    return cell(x, y);
}

GridMapCell &GridMap::operator()(const Point &p)
{
    return cell(p);
}

GridMapCell &GridMap::cell(int x, int y)
{
    consist_assert(x >= 0 && x < _xcells && y >= 0 && y < _ycells);
    return _grid[x + y * _xcells];
}

GridMapCell &GridMap::cell(double x, double y)
{
    return cell(xcell(x), ycell(y));
}

GridMapCell &GridMap::cell(const Point &p)
{
    return cell(xcell(p.x()), ycell(p.y()));
}

const GridMapCell &GridMap::operator()(int x, int y) const
{
    return cell(x, y);
}

const GridMapCell &GridMap::operator()(double x, double y) const
{
    return cell(x, y);
}

const GridMapCell &GridMap::operator()(const Point &p) const
{
    return cell(p);
}

const GridMapCell &GridMap::cell(int x, int y) const
{
    consist_assert(x >= 0 && x < _xcells && y >= 0 && y < _ycells);
    return _grid[x + y * _xcells];
}

const GridMapCell &GridMap::cell(double x, double y) const
{
    return cell(xcell(x), ycell(y));
}

const GridMapCell &GridMap::cell(const Point &p) const
{
    return cell(xcell(p.x()), ycell(p.y()));
}

/* ---------------------------------------------------------------------------------------------- */

bool GridMap::cellexists(int x, int y) const
{
    return x >= 0 && x < _xcells && y >= 0 && y < _ycells;
}

double GridMap::xmin() const
{
    return _xmin;
}

double GridMap::ymin() const
{
    return _ymin;
}

double GridMap::xmax() const
{
    return _xmin + _gridsize * _xcells;
}

double GridMap::ymax() const
{
    return _ymin + _gridsize * _ycells;
}

double GridMap::gridsize() const
{
    return _gridsize;
}

bool GridMap::valid(const Point &p) const
{
    return valid(p.x(), p.y());
}

bool GridMap::valid(double x, double y) const
{
    return x >= _xmin && x <= _xmin + _gridsize * _xcells &&
           y >= _ymin && y <= _ymin + _gridsize * _ycells;
}

/* ---------------------------------------------------------------------------------------------- */

int GridMap::xcells() const
{
    return _xcells;
}

int GridMap::ycells() const
{
    return _ycells;
}

int GridMap::xcell(double x) const
{
    consist_assert(x >= _xmin && x <= _xmin + _gridsize * _xcells);
    return (x - _xmin) / _gridsize;
}

int GridMap::ycell(double y) const
{
    consist_assert(y >= _ymin && y <= _ymin + _gridsize * _ycells);
    return (y - _ymin) / _gridsize;
}

int GridMap::xcell(const Point &p) const
{
    return xcell(p.x());
}

int GridMap::ycell(const Point &p) const
{
    return ycell(p.y());
}

double GridMap::xcellstart(int cellx) const
{
    return _xmin + cellx * _gridsize;
}

double GridMap::ycellstart(int celly) const
{
    return _ymin + celly * _gridsize;
}

/* ---------------------------------------------------------------------------------------------- */

void GridMap::rasterizeLine(const Point &from, const Point &to)
{
    double dx = std::abs(to.x() - from.x());
    double dy = std::abs(to.y() - from.y());

    int xcellfrom = xcell(from), xcellto = xcell(to);
    int ycellfrom = ycell(from), ycellto = ycell(to);

    double d, fromx, fromy, tox, toy;

    if(dx > dy) {
        if(xcellto < xcellfrom) {
            std::swap(xcellto, xcellfrom);
            d = (from.y() - to.y()) / (from.x() - to.x());
            fromx = to.x(); tox = from.x();
            fromy = to.y(); toy = from.y();
        } else {
            d = (to.y() - from.y()) / (to.x() - from.x());
            fromx = from.x(); tox = to.x();
            fromy = from.y(); toy = to.y();
        }

        for(int x = xcellfrom; x <= xcellto; x++) {
            double xstart = xcellstart(x), xend = xcellstart(x + 1);
            double ystart = d * (std::max(xstart, fromx) - fromx) + fromy;
            double yend   = d * (std::min(xend,   tox  ) - fromx) + fromy;
            int ystartcell = ycell(ystart), yendcell = ycell(yend);
            cell(x, ystartcell).distance = 0;
            cell(x, ystartcell).status   = GridMapCell::Boundary;
            if(yendcell != ystartcell) {
                cell(x, yendcell).distance = 0;
                cell(x, yendcell).status   = GridMapCell::Boundary;
            }
        }
    } else {
        if(ycellto < ycellfrom) {
            std::swap(ycellto, ycellfrom);
            d = (from.x() - to.x()) / (from.y() - to.y());
            fromx = to.x(); tox = from.x();
            fromy = to.y(); toy = from.y();
        } else {
            d = (to.x() - from.x()) / (to.y() - from.y());
            fromx = from.x(); tox = to.x();
            fromy = from.y(); toy = to.y();
        }

        for(int y = ycellfrom; y <= ycellto; y++) {
            double ystart = ycellstart(y), yend = ycellstart(y + 1);
            double xstart = d * (std::max(ystart, fromy) - fromy) + fromx;
            double xend   = d * (std::min(yend,   toy  ) - fromy) + fromx;
            int xstartcell = xcell(xstart), xendcell = xcell(xend);
            cell(xstartcell, y).distance = 0;
            cell(xstartcell, y).status   = GridMapCell::Boundary;
            if(xendcell != xstartcell) {
                cell(xendcell, y).distance = 0;
                cell(xendcell, y).status   = GridMapCell::Boundary;
            }
        }
    }
}

void GridMap::rasterize(const Visibility &vis)
{
    for(int i = 0; i < _xcells * _ycells; i++) {
        _grid[i].distance = (_xcells + _ycells) * (_xcells + _ycells);
        _grid[i].status   = GridMapCell::Empty;
    }

    scanFill(vis);

    Point prev = vis.viewPoint();
    fforeach(const Point &p, vis.points()) {
        rasterizeLine(prev, p);
        prev = p;
    }
    rasterizeLine(prev, vis.viewPoint());
}

#ifdef CONSIST_SEGMENT_TREE_LOOKUP
struct SegmentInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point from, to;
    double miny, maxy;
};

static inline bool minyCompare(const SegmentInfo *s1, const SegmentInfo *s2)
{
    return s1->miny < s2->miny;
}

static inline bool maxyCompare(const SegmentInfo *s1, const SegmentInfo *s2)
{
    return s1->maxy < s2->maxy;
}
#endif /* CONSIST_SEGMENT_TREE_LOOKUP */

void GridMap::scanFill(const Visibility &vis)
{
#ifdef CONSIST_SEGMENT_TREE_LOOKUP
    /*
     * Although this should be more efficient, practically it takes roughly
     * the same amount of time of the trivial version. Big constants, huh.
     */
    std::set<SegmentInfo *> active;
    std::vector<SegmentInfo *> starts, ends;
    PointArray vertices = vis.points();
    vertices.push_back(vis.viewPoint());

    Point prev = vis.viewPoint();
    SegmentInfo *infos = new SegmentInfo[vertices.size()];

    for(int i = 0; i < vertices.size(); i++) {
        const Point &p = vertices[i];
        infos[i].from = prev;
        infos[i].to   = p;
        infos[i].miny = std::min(prev.y(), p.y());
        infos[i].maxy = std::max(prev.y(), p.y());
        starts.push_back(infos + i);
        ends.push_back(infos + i);
        prev = p;
    }

    std::sort(starts.begin(), starts.end(), minyCompare);
    std::sort(ends.begin(), ends.end(), maxyCompare);
    std::vector<SegmentInfo *>::iterator its = starts.begin(), ite = ends.begin();

    for(int y = 0; y < _ycells; y++) {
        double thisy = ycellstart(y);
        std::vector<double> xintersections;

        while(its != starts.end() && (*its)->miny < thisy) {
            active.insert(*its);
            ++its;
        }

        while(ite != ends.end() && (*ite)->maxy < thisy) {
            active.erase(*ite);
            ++ite;
        }

        fforeach(const SegmentInfo *info, active) {
            xintersections.push_back(
                    (info->to.x() * thisy + info->from.x() * info->to.y() -
                     info->from.x() * thisy - info->to.x() * info->from.y()) /
                    (info->to.y() - info->from.y()));
        }

        std::sort(xintersections.begin(), xintersections.end());
        for(size_t i = 0; i < xintersections.size(); i += 2) {
            if(i + 1 == xintersections.size()) {
                std::cout << xintersections << std::endl;
            }
            int xstart = xcell(xintersections[i]) + 1;
            int xend   = xcell(xintersections[i + 1]);

            for(int x = xstart; x < xend; x++) {
                cell(x, y).status = GridMapCell::Filled;
            }
        }
    }

    delete[] infos;
#else /* CONSIST_SEGMENT_TREE_LOOKUP */
    PointArray vertices = vis.points();
    vertices.push_back(vis.viewPoint());
    for(int y = 0; y < _ycells; y++) {
        double thisy = ycellstart(y);
        std::vector<double> xintersections;
        Point prev = vis.viewPoint();

        fforeach(const Point &p, vertices) {
            if(std::max(prev.y(), p.y()) >= thisy && std::min(prev.y(), p.y()) < thisy) {
                xintersections.push_back(
                        (p.x() * thisy - prev.x() * thisy + prev.x() * p.y() - p.x() * prev.y()) /
                        (p.y() - prev.y()));
            }
            prev = p;
        }

        std::sort(xintersections.begin(), xintersections.end());
        for(size_t i = 0; i < xintersections.size(); i += 2) {
            if(i + 1 == xintersections.size()) {
                std::cout << xintersections << std::endl;
            }
            int xstart = xcell(xintersections[i]) + 1;
            int xend   = xcell(xintersections[i + 1]);

            for(int x = xstart; x < xend; x++) {
                cell(x, y).status = GridMapCell::Filled;
            }
        }
    }
#endif /* CONSIST_SEGMENT_TREE_LOOKUP */
}

void GridMap::savePGM(const char *fname) const
{
    std::ofstream f(fname);
    f << "P2\n# " << fname << "\n" << _xcells << " " << _ycells << "\n255\n";
    for(int y = _ycells - 1; y >= 0; y--) {
        for(int x = 0; x < _xcells; x++) {
            int value = std::min(std::max(cell(x, y).distance, 0.), 255.);
            //int value = cell(x, y).status == GridMapCell::Empty ? 127 : cell(x, y).status == GridMapCell::Filled ? 255 : 0;
            f << value << " ";
        }
        f << "\n";
    }
    f.close();
}

std::vector<double> GridMap::distanceTransform1D(const std::vector<double> &f) const
{
    const int n = f.size();

    std::vector<double> d(n);
    std::vector<int>    v(n);
    std::vector<double> z(n + 1);
    int k = 0;
    v[0] = 0;
    z[0] = -INFINITY;
    z[1] = +INFINITY;
    for(int q = 1; q < n; q++) {
        double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);

        while (s <= z[k]) {
            k--;
            s  = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = +INFINITY;
    }

    k = 0;
    for(int q = 0; q < n; q++) {
        while(z[k + 1] < q) {
            k++;
        }
        d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
    }

    return d;
}

void GridMap::computeDistanceTransform()
{
    for(int x = 0; x < _xcells; x++) {
        std::vector<double> vec(_ycells);
        for(int y = 0; y < _ycells; y++) {
            vec[y] = cell(x, y).distance;
        }
        std::vector<double> dt = distanceTransform1D(vec);
        for(int y = 0; y < _ycells; y++) {
            cell(x, y).distance = dt[y];
        }
    }

    for(int y = 0; y < _ycells; y++) {
        std::vector<double> vec(_xcells);
        for(int x = 0; x < _xcells; x++) {
            vec[x] = cell(x, y).distance;
        }
        std::vector<double> dt = distanceTransform1D(vec);
        for(int x = 0; x < _xcells; x++) {
            cell(x, y).distance = dt[x];
        }
    }

    for(int x = 0; x < _xcells; x++) {
        for(int y = 0; y < _ycells; y++) {
            cell(x, y).distance = _gridsize * std::sqrt(cell(x, y).distance);
        }
    }
}

} /* namespace consist */
