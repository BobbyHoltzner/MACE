#include "polysplit.h"

/**
 * @brief initPolygon Given the boundary vertices, initialize the polygon for splitting
 * @param points Boundary vertices
 * @param numVehicles Number of vehicles we are splitting the polygon for
 */
void PolySplit::initPolygon(std::vector<Point> points, int numVehicles)
{
    polygon.clear();

    for(auto point : points) {
        polygon.push_back(Vector(point.x, point.y));
    }

//    polygon.push_back(Vector(400.0, 100.0));
//    polygon.push_back(Vector(900.0, 100.0));
//    polygon.push_back(Vector(900.0, 400.0));
//    polygon.push_back(Vector(400.0, 400.0));

    Polygon polygonToSplit = polygon;
    for(int i = 0; i < numVehicles; i++) {
        Polygon poly1, poly2;
        Line cut;
        double squareToCut = polygonToSplit.countSquare() / (points.size()-i);
        polygonToSplit.split(squareToCut, poly1, poly2, cut);

        // Set polygon to keep based on areas::
        if(poly1.countSquare() <= poly2.countSquare() || (points.size() - i) == 1) {
            splitPolygons.push_back(poly1);
            // Push centroid:
            splitCentroids.push_back(poly1.countCenter());
            // Set next polygon to split:
            polygonToSplit = poly2;

        }
        else {
            splitPolygons.push_back(poly2);
            // Push centroid:
            splitCentroids.push_back(poly2.countCenter());
            // Set next polygon to split:
            polygonToSplit = poly1;
        }
    }
}

/**
 * @brief getCentroids Return the centroids of the areas split from the environment boundary
 * @return Vector of points corresponding to area centroids
 */
std::vector<Point> PolySplit::getCentroids() {
    std::vector<Point> centroids;
    for(auto centroid : splitCentroids) {
        centroids.push_back(Point(centroid.x, centroid.y, centroid.z));
    }
    return centroids;
}
