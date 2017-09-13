#ifndef ENVIRONMENT_CUSTOM_H
#define ENVIRONMENT_CUSTOM_H

#include <map>
#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <iostream>

#include "voropp/voro_index.hh"
#include "voropp/container.hh"
#include "voropp/v_compute.hh"
#include "voropp/c_loops.hh"
#include <tuple>

#include "data_generic_state_item_topic/state_topic_components.h"
#include "base/pose/cartesian_position_2D.h"
#include "base/geometry/polygon_2dc.h"
#include "maps/bounded_2d_grid.h"

#include "data/timer.h"

/**
 * @brief The GridDirection enum to denote how we sort cell nodes
 */
enum GridDirection {
    NORTH_SOUTH,
    EAST_WEST,
    CLOSEST_POINT
};

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
    Point(const Point &obj) {
        this->x = obj.x;
        this->y = obj.y;
        this->z = obj.z;
    }

    void operator =(const Point &rhs) {
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
    }

    bool operator ==(const Point &rhs) {
        if(this->x != rhs.x)
            return false;
        if(this->y != rhs.y)
            return false;
        if(this->z != rhs.z)
            return false;

        return true;
    }

    bool operator !=(const Point &rhs) {
        return !((*this) == rhs);
    }
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
 * @brief The Cell struct is a simple container for holding cell sites and their corresponding cell vertices
 */
struct Cell {
    Point site;
    std::vector<Point> vertices;
    std::map<double, std::map<double, Node> > containedNodes;
    std::map<double, std::map<double, Node> > containedNodes_YX;
    std::vector<Point> unsortedContainedPoints;
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
    Environment_Map(const std::vector<Point> &verts, double &gridSpacing, const DataState::StateGlobalPosition &globalOrigin);

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
     * @return Success or Failure
     */
    bool computeVoronoi(const BoundingBox bbox, const std::map<int, Point> vehicles, GridDirection direction);

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
     * @brief setNodesInCell Get the nodes contained in the polygon provided in a cell
     * @param Cell Cell with boundary of points that make up the footprint we want to check
     */
    void setNodesInCell(Cell &cell);

    /**
     * @brief addVehicle Update/insert a vehicle in our map and re-compute the voronoi partition
     * @param vehicleID ID of the vehicle to add
     * @param position Last known position of the vehicle
     */
    bool updateVehiclePosition(const int &vehicleID, const Point &position, bool recomputeVoronoi);

    /**
     * @brief getCells Return the cells that make up our Voronoi partition
     * @return Cells making up the voronoi partition
     */
    std::map<int, Cell> getCells() { return cells; }

    /**
     * @brief sortNodesInGridSort the nodes in the cell in a grid fashion
     * @param cell Cell to sort the nodes
     * @param direction Direction to sort nodes (north/south, east/west, or by closest node)
     */
    std::vector<Point> sortNodesInGrid(Cell &cell, GridDirection direction);

    /**
     * @brief getBoundaryVerts Return the vector of points that make up the boundary
     * @return Vector of points making up a boundary
     */
    std::vector<Point> getBoundaryVerts() { return boundaryVerts ;}

    /**
     * @brief getBoundingBox Return the min/max of the rectangle encompassing the environment
     * @return Bounding box min/max
     */
    BoundingBox getBoundingBox() { return boundingRect; }

    /**
     * @brief updateEnvironmentOrigin Given a new global origin, update x,y,z positions of each node and update the global origin
     * @param globalOrigin New global origin
     */
    void updateEnvironmentOrigin(const DataState::StateGlobalPosition &globalOrigin);

    /**
     * @brief getGlobalOrigin Get the current global origin
     * @return Current global origin
     */
    std::shared_ptr<DataState::StateGlobalPosition> getGlobalOrigin() { return m_globalOrigin; }

private:

    /**
     * @brief getVehicleID Check a cell for the vehicle contained within the cell to grab its ID
     * @param cell Cell who's boundary we will check
     * @param vehicleList List of vehicles and their positions to check
     * @return
     */
    int getVehicleID(const Cell cell, const std::map<int, Point> vehicleList);

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
     * @brief distanceToSegment Determine shortest distance to a line segment
     * @param p1 First vertex of the line segment
     * @param p2 Second vertex of the line segment
     * @param testPoint Point to check
     * @return Distance to the line segment
     */
    double distanceToSegment(Point p1, Point p2, Point testPoint);


    /**
     * @brief sortCellVertices Sort the vertices of a cell in CCW fashion
     * @param cell Cell to update vertex ordering
     */
    void sortCellVerticesCCW(Cell &cell);

    /**
     * @brief setContainedNodesYX For sorting a grid in the North/South direction, we need the nodes in Y,X order instead of X,Y order
     * @param cell Cell to set our YX pairs for
     */
    void setContainedNodesYX(Cell &cell);


private:
    /**
     * @brief nodes Environment map (sorted Xval, Yval)
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

    /**
     * @brief vehicles Container for last known position of each vehicle (id, position)
     */
    std::map<int, Point> m_vehicles;

    /**
     * @brief m_globalOrigin Container for the current global origin
     */
    std::shared_ptr<DataState::StateGlobalPosition> m_globalOrigin;
};

#endif // ENVIRONMENT_CUSTOM_H
