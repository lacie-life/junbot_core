//
// Created by lacie on 03/02/2023.
//

#include "OctoMapBuilder.h"

OctoMapBuilder::OctoMapBuilder(double resolution) :
        m_octree(NULL),
        m_maxRange(-1.0),
        m_useHeightMap(true),
        m_res(0.05),
        m_colorFactor(0.8),
        m_treeDepth(0),
        m_maxTreeDepth(0)
{

}