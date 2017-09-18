#include "polysplit.h"

/**
 * @brief initPolygon Given the boundary vertices, initialize the polygon for splitting
 * @param boundary boundary polygon that defines the environment
 * @param numVehicles Number of vehicles we are splitting the polygon for
 */
void PolySplit::initPolygon(const mace::geometry::Polygon_2DC &boundary, const int &numVehicles)
{
    polygon.clear();
    std::vector<mace::pose::Position<mace::pose::CartesianPosition_2D>> boundaryVector = boundary.getVector();

    for(auto vertex : boundaryVector) {
        polygon.push_back(Vector(vertex.getXPosition(), vertex.getYPosition()));
    }

    Polygon polygonToSplit = polygon;
    for(int i = 0; i < numVehicles; i++) {
        Polygon poly1, poly2;
        Line cut;
        double squareToCut = polygonToSplit.countSquare() / (boundaryVector.size()-i);
        polygonToSplit.split(squareToCut, poly1, poly2, cut);

        // Set polygon to keep based on areas::
        if(poly1.countSquare() <= poly2.countSquare() || (boundaryVector.size() - i) == 1) {
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
std::vector<Point> PolySplit::getCentroids() const {
    std::vector<Point> centroids;
    for(auto centroid : splitCentroids) {
        centroids.push_back(Point(centroid.x, centroid.y, centroid.z));
    }
    return centroids;
}
