//
// Created by rgrandia on 27.04.22.
//

#include "ocs2_anymal_models/FrameDeclaration.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include <sstream>
#include <stdexcept>

namespace anymal {

std::vector<std::string> getJointNames(const FrameDeclaration& frameDeclaration) {
  std::vector<std::string> jointNames;
  for (const auto& leg : frameDeclaration.legs) {
    jointNames.insert(jointNames.end(), leg.joints.begin(), leg.joints.end());
  }
  return jointNames;
}

switched_model::feet_array_t<switched_model::scalar_t> getContactRadii(const FrameDeclaration& frameDeclaration) {
  switched_model::feet_array_t<switched_model::scalar_t> contactRadii{};
  for (int i = 0; i < switched_model::NUM_CONTACT_POINTS; ++i) {
    contactRadii[i] = frameDeclaration.legs[i].contactRadius;
  }
  return contactRadii;
}

LimbFrames limbFramesFromFile(const std::string& file, const std::string& field) {
  LimbFrames frames;
  ocs2::loadData::loadCppDataType(file, field + ".root", frames.root);
  ocs2::loadData::loadCppDataType(file, field + ".tip", frames.tip);
  ocs2::loadData::loadStdVector(file, field + ".joints", frames.joints, false);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  ocs2::loadData::loadPtreeValue(pt, frames.contactRadius, field + ".contactRadius", false);
  return frames;
}

namespace {

std::vector<CollisionDeclaration> collisionDeclarationsFromFile(const std::string& file, const std::string& field) {
  std::vector<std::pair<std::string, ocs2::scalar_t>> collisionSpheres;
  ocs2::loadData::loadStdVectorOfPair(file, field + ".collisionSpheres", collisionSpheres, false);

  if (collisionSpheres.empty()) {
    return {};
  }

  ocs2::matrix_t offsets(collisionSpheres.size(), 3);
  ocs2::loadData::loadEigenMatrix(file, field + ".collisionOffsets", offsets);
  if (offsets.rows() != collisionSpheres.size() || offsets.cols() != 3) {
    throw std::runtime_error("[FrameDeclaration] " + field + ".collisionOffsets must have one 3D offset per sphere.");
  }

  std::vector<CollisionDeclaration> declarations;
  declarations.reserve(collisionSpheres.size());
  for (int i = 0; i < collisionSpheres.size(); ++i) {
    CollisionDeclaration collisionDeclaration;
    collisionDeclaration.link = collisionSpheres[i].first;
    collisionDeclaration.radius = collisionSpheres[i].second;
    collisionDeclaration.offset = offsets.row(i).transpose();
    declarations.push_back(std::move(collisionDeclaration));
  }

  return declarations;
}

std::vector<std::string> splitCommaSeparated(const std::string& specification) {
  std::vector<std::string> fields;
  std::stringstream stream(specification);
  std::string field;
  while (std::getline(stream, field, ',')) {
    const auto first = field.find_first_not_of(" \t");
    const auto last = field.find_last_not_of(" \t");
    fields.push_back(first == std::string::npos ? std::string{} : field.substr(first, last - first + 1));
  }
  return fields;
}

std::vector<SelfCollisionPairDeclaration> selfCollisionPairsFromFile(const std::string& file) {
  std::vector<std::string> specifications;
  ocs2::loadData::loadStdVector(file, "selfCollisions.pairs", specifications, false);

  std::vector<SelfCollisionPairDeclaration> pairs;
  pairs.reserve(specifications.size());
  for (const auto& specification : specifications) {
    const auto fields = splitCommaSeparated(specification);
    if (fields.size() != 3) {
      throw std::runtime_error("[FrameDeclaration] Each selfCollisions.pairs entry must be 'firstLink, secondLink, minimumDistance'.");
    }
    try {
      pairs.push_back({fields[0], fields[1], std::stod(fields[2])});
    } catch (const std::exception&) {
      throw std::runtime_error("[FrameDeclaration] Invalid self-collision pair: " + specification);
    }
  }
  return pairs;
}

}  // namespace

FrameDeclaration frameDeclarationFromFile(const std::string& file) {
  FrameDeclaration decl;
  ocs2::loadData::loadCppDataType(file, "root", decl.root);
  decl.legs[0] = limbFramesFromFile(file, "left_front");
  decl.legs[1] = limbFramesFromFile(file, "right_front");
  decl.legs[2] = limbFramesFromFile(file, "left_hind");
  decl.legs[3] = limbFramesFromFile(file, "right_hind");

  decl.collisions = collisionDeclarationsFromFile(file, "collisions");
  decl.selfCollisions = collisionDeclarationsFromFile(file, "selfCollisions");
  decl.selfCollisionPairs = selfCollisionPairsFromFile(file);

  return decl;
}

}  // namespace anymal
