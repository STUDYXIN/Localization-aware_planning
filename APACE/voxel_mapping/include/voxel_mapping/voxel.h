#ifndef VOXEL_H
#define VOXEL_H

#include "exploration_types.h"

namespace voxel_mapping
{
  enum class OccupancyType
  {
    UNKNOWN,
    OCCUPIED,
    FREE
  };

  struct OccupancyVoxel
  {
    OccupancyType value = OccupancyType::UNKNOWN;
  };

  struct TSDFVoxel
  {
    FloatingPoint value = 0.0, weight = 0.0;
  };

  struct ESDFVoxel
  {
    FloatingPoint value = 0.0;
  };

  enum class FeatureType
  {
    UNKNOWN,
    HASFEATURE,
    NOFEATURE
  };

  struct FeatureVoxel
  {
    FeatureType value = FeatureType::UNKNOWN;
  };

} // namespace voxel_mapping

#endif // VOXEL_H