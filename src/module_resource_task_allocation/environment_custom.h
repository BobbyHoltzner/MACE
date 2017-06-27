#ifndef ENVIRONMENT_CUSTOM_H
#define ENVIRONMENT_CUSTOM_H

#include <map>
#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <iostream>

#include "voro_index.hh"
#include "container.hh"
#include <v_compute.hh>
#include <c_loops.hh>
#include <tuple>

/**
 * @brief The Point class is a simple container for holding x,y pairs
 */
class Point {
public:
    double x;
    double y;
    double z;

    Point() : x(0.0), y(0.0), z(0.0) {}
    Point(double x, double y, double z) : x(x), y(y), z(z) {}
};

/**
 * @brief The Cell struct is a simple container for holding cell sites and their corresponding cell vertices
 */
struct Cell {
    Point site;
    std::vector<Point> vertices;
};

/**
 * @brief The Node class is a container for nodes at each x,y location in our environment
 */
class Node {
public:
    Node() {}
    Node(Point loc, double val) : location(loc), value(val) {}

    Point location;
    double value;
};

/**
 * @brief The BoundingRect struct holds minimum and maximum x/y values for a bounding rectangle (for Voronoi)
 */
struct BoundingBox {
    Point min;
    Point max;
};

/**
 * @brief The Environment_Map class holds the environment boundary verticies, bounding rectangle, and Nodes on a 2D grid
 */
class Environment_Map
{
public:
    /**
     * @brief Environment_Map constructor
     * @param verts Vector of vertices that make up the environment boundary
     * @param gridSpacing Spacing between grid points
     */
    Environment_Map(const std::vector<Point> verts, double gridSpacing);

    /**
     * @brief initializeEnvironment Initialize each node in the grid with a 0 value
     * @param gridSpacing Grid spacing
     * @return initial environment map
     */
    std::map<double, std::map<double, Node> > initializeEnvironment(const double gridSpacing);

    /**
     * @brief computeVoronoi Given the bounding box and current vehicle positions, compute a voronoi diagram
     * @param bbox Bounding box
     * @param sitePositions Positions of vehicles (in x,y,z coordinates)
     */
    void computeVoronoi(const BoundingBox bbox, const std::vector<Point> sitePositions);

    /**
     * @brief setBoundaryVerts Set the new boundary vertices
     * @param verts Vector of Points defining the environment boundary
     */
    void setBoundaryVerts(std::vector<Point> verts);

    /**
     * @brief setNodeValue Set the value of the node closest to the given (x,y) point
     * @param location (x,y) point corresponding to the node we want to set
     * @param value New value for node
     * @return Success/failure
     */
    bool setNodeValue(const Point location, double value);

    /**
     * @brief getNodeValue Get the value of the node closest to the given (x,y) point
     * @param location (x,y) point corresponding to the node we want to set
     * @param node Node closest to the given point
     * @return Success/failure
     */
    bool getNodeValue(const Point location, Node &node);

    /**
     * @brief getNodesInPolygon Get the nodes contained in the polygon provided
     * @param boundary Vector of points that make up the footprint we want to check
     * @return Map of nodes within the footprint
     */
    std::map<double, std::map<double, Node> > getNodesInPolygon(std::vector<Point> boundary);

private:

    /**
     * @brief findClosestPoint Find the closest (x,y) grid point corresponding to the (x,y) pair given
     * @param testPoint Location we want to grab the corresponding grid point for
     * @param closestPoint Grid point closest to the provided point
     * @return Success/failure
     */
    bool findClosestPoint(Point testPoint, Point &closestPoint);

    /**
     * @brief distanceBetweenPoints Caluclate the distance between two points
     * @param pt1
     * @param pt2
     * @return Distance
     */
    double distanceBetweenPoints(Point pt1, Point pt2);

    /**
     * @brief findClosestNode Get the node at the closest point to the (x,y) pair provided
     * @param testPoint (x,y) pair we want to find the corresponding grid point for
     * @param closestNode Node at the closest grid point
     * @return Succes/failure
     */
    bool findClosestNode(Point testPoint, Node &closestNode);

    /**
     * @brief calculateBoundingRect Calculate the bounding rectangle given a set of vertices
     * @param verts Vertices to calculate the bounding rectangle for
     */
    void calculateBoundingRect(const std::vector<Point> verts);

    /**
     * @brief createRange Create a vector of evenly space numbers
     * @param min Minimum value
     * @param max Maximum value
     * @param N Number of values in the range
     * @return Vector of evenly spaced values from min to max
     */
    std::vector<double> createRange(double min, double max, int N);


    // method taken from: https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html and converted to my notation
    /**
     * @brief pointInPoly Determine if a point is in a polygon
     * @param vertices Vertices of a polygon to check
     * @param testPoint Point to check if in the polygon
     * @return True if inside the polygon, false if outside the polygon
     */
    bool pointInPoly(std::vector<Point> vertices, Point testPoint);


    /**
     * @brief sortCellVertices Sort the vertices of a cell in CCW fashion
     * @param cell Cell to update vertex ordering
     */
    void sortCellVerticesCCW(Cell &cell);


private:
    /**
     * @brief nodes Environment map
     */
    std::map<double, std::map<double, Node> > nodes;

    /**
     * @brief boundaryVerts Vertices that make up the environment boundary
     */
    std::vector<Point> boundaryVerts;

    /**
     * @brief boundingRect Bounding rectangle of the environment
     */
    BoundingBox boundingRect;

    /**
     * @brief cells Container for cells corresponding to each vehicle
     */
    std::map<int, Cell> cells;
};

#endif // ENVIRONMENT_CUSTOM_H
