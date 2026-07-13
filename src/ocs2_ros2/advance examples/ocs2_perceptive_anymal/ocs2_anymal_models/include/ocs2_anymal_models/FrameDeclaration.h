//
// Created by rgrandia on 27.04.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace anymal {

struct CollisionDeclaration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string link;
  switched_model::scalar_t radius;
  switched_model::vector3_t offset;
};

struct SelfCollisionPairDeclaration {
  std::string first;
  std::string second;
  switched_model::scalar_t minimumDistance;
};

struct LimbFrames {
  std::string root;
  std::string tip;
  std::vector<std::string> joints;
  switched_model::scalar_t contactRadius = 0.0;
};

struct FrameDeclaration {
  std::string root;
  switched_model::feet_array_t<LimbFrames> legs;
  /** Collision bodies used only for terrain SDF avoidance. */
  std::vector<CollisionDeclaration> collisions;
  /** Collision bodies and pairs used only for robot self-collision avoidance. */
  std::vector<CollisionDeclaration> selfCollisions;
  std::vector<SelfCollisionPairDeclaration> selfCollisionPairs;
};

std::vector<std::string> getJointNames(const FrameDeclaration& frameDeclaration);

switched_model::feet_array_t<switched_model::scalar_t> getContactRadii(const FrameDeclaration& frameDeclaration);

LimbFrames limbFramesFromFile(const std::string& file, const std::string& field);

FrameDeclaration frameDeclarationFromFile(const std::string& file);

}  // namespace anymal
