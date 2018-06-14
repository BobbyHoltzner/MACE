#ifndef OCTOMAP_SENSOR_DEFINITION_H
#define OCTOMAP_SENSOR_DEFINITION_H

#include <limits>
#include <string>

namespace mace{
namespace maps{

class OctomapSensorDefinition
{
public:

    OctomapSensorDefinition() :
        m_set_InitialLoad(false),
        m_set_Resolution(false),
        m_set_MinRange(false),
        m_set_MaxRange(false),
        m_set_occupancyThresh(false),
        m_set_probHit(false),
        m_set_probMiss(false),
        m_set_ThreshMin(false),
        m_set_ThreshMax(false)
    {

    }

    OctomapSensorDefinition(const OctomapSensorDefinition &copy)
    {
        this->m_set_InitialLoad = copy.m_set_InitialLoad;
        this->m_initialOctomapLoad = copy.m_initialOctomapLoad;

        this->m_set_Resolution = copy.m_set_Resolution;
        this->m_TreeResolution = copy.m_TreeResolution;

        this->m_set_MinRange = copy.m_set_MinRange;
        this->m_MinRange = copy.m_MinRange;

        this->m_set_MaxRange = copy.m_set_MaxRange;
        this->m_MaxRange = copy.m_MaxRange;

        this->m_set_occupancyThresh = copy.m_set_occupancyThresh;
        this->m_occupancyThresh = copy.m_occupancyThresh;

        this->m_set_probHit = copy.m_set_probHit;
        this->m_probHit = copy.m_probHit;

        this->m_set_probMiss = copy.m_set_probMiss;
        this->m_probMiss = copy.m_probMiss;

        this->m_set_ThreshMin = copy.m_set_ThreshMin;
        this->m_ThreshMin = copy.m_ThreshMin;

        this->m_set_ThreshMax = copy.m_set_ThreshMax;
        this->m_ThreshMax = copy.m_ThreshMax;
    }

    void setInitialLoadFile(const std::string &file)
    {
        m_initialOctomapLoad = file;
        m_set_InitialLoad = true;
    }
    bool isInitalLoadFileSet() const
    {
        return m_set_InitialLoad;
    }
    std::string getInitialLoadFile() const
    {
        return m_initialOctomapLoad;
    }

    void setTreeResolution(const double &resolution)
    {
        m_TreeResolution = resolution;
        m_set_Resolution = true;
    }
    bool isTreeResolutionSet() const
    {
        return m_set_Resolution;
    }
    double getTreeResolution() const
    {
        return m_TreeResolution;
    }

    void setMinRange(const double minRange)
    {
        m_MinRange = minRange;
        m_set_MinRange = true;
    }
    bool isMinRange() const
    {
        return m_set_MinRange;
    }
    double getMinRange() const
    {
        return m_MinRange;
    }


    void setMaxRange(const double maxRange)
    {
        m_MaxRange = maxRange;
        m_set_MaxRange = true;
    }
    bool isMaxRange() const
    {
        return m_set_MaxRange;
    }
    double getMaxRange() const
    {
        return m_MaxRange;
    }


    void setOccupancyThreshold(const double occupancythreshold)
    {
        m_occupancyThresh = occupancythreshold;
        m_set_occupancyThresh = true;
    }
    bool isOccupancyThreshold() const
    {
        return m_set_occupancyThresh;
    }
    double getOccupancyThreshold() const
    {
        return m_occupancyThresh;
    }


    void setProbHit(const double probHit)
    {
        m_probHit = probHit;
        m_set_probHit = true;
    }
    bool isProbHit() const
    {
        return m_set_probHit;
    }
    double getProbHit() const
    {
        return m_probHit;
    }


    void setProbMiss(const double probMiss)
    {
        m_probMiss = probMiss;
        m_set_probMiss = true;
    }

    bool isProbMiss() const
    {
        return m_set_probMiss;
    }

    double getProbMiss() const
    {
        return m_probMiss;
    }


    void setThreshMin(const double ThreshMin)
    {
        m_ThreshMin = ThreshMin;
        m_set_ThreshMin = true;
    }
    bool isThreshMin() const
    {
        return m_set_ThreshMin;
    }
    double getThreshMin() const
    {
        return m_ThreshMin;
    }


    void setThreshMax(const double ThreshMax)
    {
        m_ThreshMax = ThreshMax;
        m_set_ThreshMax = true;
    }

    bool isThreshMax() const
    {
        return m_set_ThreshMax;
    }
    double getThreshMax() const
    {
        return m_ThreshMax;
    }


    double getMinSizeX() const
    {
        return minSizeX;
    }

    double getMinSizeY() const
    {
        return minSizeY;
    }

public:
    OctomapSensorDefinition& operator = (const OctomapSensorDefinition &rhs)
    {
        this->m_set_InitialLoad = rhs.m_set_InitialLoad;
        this->m_initialOctomapLoad = rhs.m_initialOctomapLoad;

        this->m_set_Resolution = rhs.m_set_Resolution;
        this->m_TreeResolution = rhs.m_TreeResolution;

        this->m_set_MinRange = rhs.m_set_MinRange;
        this->m_MinRange = rhs.m_MinRange;

        this->m_set_MaxRange = rhs.m_set_MaxRange;
        this->m_MaxRange = rhs.m_MaxRange;

        this->m_set_occupancyThresh = rhs.m_set_occupancyThresh;
        this->m_occupancyThresh = rhs.m_occupancyThresh;

        this->m_set_probHit = rhs.m_set_probHit;
        this->m_probHit = rhs.m_probHit;

        this->m_set_probMiss = rhs.m_set_probMiss;
        this->m_probMiss = rhs.m_probMiss;

        this->m_set_ThreshMin = rhs.m_set_ThreshMin;
        this->m_ThreshMin = rhs.m_ThreshMin;

        this->m_set_ThreshMax = rhs.m_set_ThreshMax;
        this->m_ThreshMax = rhs.m_ThreshMax;
        return *this;
    }

private:

    bool m_set_InitialLoad;
    std::string m_initialOctomapLoad = "";

    bool m_set_Resolution;
    double m_TreeResolution = 0.5;

    bool   m_set_MinRange;
    double m_MinRange = 0.25;

    bool   m_set_MaxRange;
    double m_MaxRange = -1;

    bool   m_set_occupancyThresh;
    double m_occupancyThresh;

    bool   m_set_probHit;
    double m_probHit = 0.7;

    bool   m_set_probMiss;
    double m_probMiss = 0.4;

    bool   m_set_ThreshMin;
    double m_ThreshMin = 0.12;

    bool   m_set_ThreshMax;
    double m_ThreshMax = 0.97;

    double minSizeX = 0.0;
    double minSizeY = 0.0;
};


} //end of namespace maps
} //end of namespace mace
#endif // OCTOMAP_SENSOR_DEFINITION_H
