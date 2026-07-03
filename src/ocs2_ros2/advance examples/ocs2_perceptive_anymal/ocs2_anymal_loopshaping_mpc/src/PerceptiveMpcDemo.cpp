//
// Created by rgrandia on 31.03.22.
//

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

#include <grid_map_core/iterators/GridMapIterator.hpp>

#include "rclcpp/rclcpp.hpp"

// Plane segmentation
#include <convex_plane_decomposition/LoadGridmapFromImage.h>
#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_ros/ParameterLoading.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

// ocs2_dev
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// ocs2_anymal
#include <ocs2_anymal_commands/ReferenceExtrapolation.h>
#include <ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h>
#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/QuadrupedPinocchioMapping.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <segmented_planes_terrain_model/SegmentedPlanesTerrainModel.h>
#include <segmented_planes_terrain_model/SegmentedPlanesTerrainModelRos.h>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

using namespace switched_model;

namespace {

std::string jsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string jsonFloat(double value) {
  std::ostringstream out;
  out << std::setprecision(12) << value;
  std::string text = out.str();
  if (text.find_first_of(".eE") == std::string::npos) {
    text += ".0";
  }
  return text;
}

void writeStringArrayJson(std::ofstream& file,
                          const std::vector<std::string>& values) {
  file << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      file << ", ";
    }
    file << "\"" << jsonEscape(values[i]) << "\"";
  }
  file << "]";
}

void writeJsonStringOrNull(std::ofstream& file, const std::string& value) {
  if (value.empty()) {
    file << "null";
  } else {
    file << "\"" << jsonEscape(value) << "\"";
  }
}

void writeCsvValue(std::ofstream& file, double value) {
  if (std::isfinite(value)) {
    file << value;
  } else {
    file << "nan";
  }
}

void writeCsvHeader(std::ofstream& file, const std::vector<std::string>& names,
                    const std::vector<std::string>& suffixes) {
  file << "time";
  for (const auto& name : names) {
    for (const auto& suffix : suffixes) {
      file << "," << name << suffix;
    }
  }
  file << "\n";
}

void writeGridMapLayerMatrixCsv(const grid_map::GridMap& map,
                                const std::string& layer,
                                const std::filesystem::path& filePath) {
  if (!map.exists(layer)) {
    throw std::runtime_error("Grid map layer '" + layer + "' does not exist.");
  }

  grid_map::GridMap mapCopy = map;
  mapCopy.convertToDefaultStartIndex();
  const auto& data = mapCopy.get(layer);

  std::ofstream file(filePath);
  if (!file) {
    throw std::runtime_error("Unable to open " + filePath.string() +
                             " for writing.");
  }
  file << std::setprecision(12);
  for (int row = 0; row < data.rows(); ++row) {
    for (int col = 0; col < data.cols(); ++col) {
      if (col > 0) {
        file << ",";
      }
      writeCsvValue(file, data(row, col));
    }
    file << "\n";
  }
}

void writeGridMapLayerPointsCsv(const grid_map::GridMap& map,
                                const std::string& layer,
                                const std::filesystem::path& filePath) {
  if (!map.exists(layer)) {
    throw std::runtime_error("Grid map layer '" + layer + "' does not exist.");
  }

  grid_map::GridMap mapCopy = map;
  mapCopy.convertToDefaultStartIndex();

  std::ofstream file(filePath);
  if (!file) {
    throw std::runtime_error("Unable to open " + filePath.string() +
                             " for writing.");
  }
  file << std::setprecision(12);
  file << "row,col,x,y,z\n";
  for (grid_map::GridMapIterator iterator(mapCopy); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position position;
    mapCopy.getPosition(index, position);
    file << index(0) << "," << index(1) << "," << position.x() << ","
         << position.y() << ",";
    writeCsvValue(file, mapCopy.at(layer, index));
    file << "\n";
  }
}

void writeGridMapInfoJson(std::ofstream& file, const grid_map::GridMap& map,
                          const std::string& layer,
                          const std::string& matrixFile,
                          const std::string& pointsFile,
                          const std::string& indent) {
  grid_map::GridMap mapCopy = map;
  mapCopy.convertToDefaultStartIndex();
  const auto& data = mapCopy.get(layer);

  file << indent << "{\n";
  file << indent << "  \"layer\": \"" << jsonEscape(layer) << "\",\n";
  file << indent << "  \"matrix_file\": \"" << jsonEscape(matrixFile)
       << "\",\n";
  file << indent << "  \"points_file\": \"" << jsonEscape(pointsFile)
       << "\",\n";
  file << indent << "  \"shape\": [" << data.rows() << ", " << data.cols()
       << "],\n";
  file << indent << "  \"resolution\": "
       << jsonFloat(mapCopy.getResolution()) << ",\n";
  file << indent << "  \"frame\": \"" << jsonEscape(mapCopy.getFrameId())
       << "\",\n";
  file << indent << "  \"center_position_xy\": ["
       << jsonFloat(mapCopy.getPosition().x()) << ", "
       << jsonFloat(mapCopy.getPosition().y()) << "],\n";
  file << indent << "  \"length_xy\": [" << jsonFloat(mapCopy.getLength().x())
       << ", " << jsonFloat(mapCopy.getLength().y()) << "]\n";
  file << indent << "}";
}

std::vector<pinocchio::FrameIndex> collectBodyFrameIds(
    const pinocchio::Model& model, std::vector<std::string>& bodyNames) {
  std::vector<pinocchio::FrameIndex> bodyFrameIds;
  for (pinocchio::FrameIndex frameId = 0; frameId < model.nframes; ++frameId) {
    const auto& frame = model.frames[frameId];
    const bool isLinkFrame = ((frame.type & pinocchio::FrameType::BODY) != 0);
    if (!isLinkFrame || frame.name.empty() || frame.name == "universe") {
      continue;
    }
    if (std::find(bodyNames.begin(), bodyNames.end(), frame.name) !=
        bodyNames.end()) {
      continue;
    }
    bodyNames.push_back(frame.name);
    bodyFrameIds.push_back(frameId);
  }
  return bodyFrameIds;
}

Eigen::VectorXd getPinocchioConfiguration(
    const pinocchio::Model& model,
    const anymal::QuadrupedPinocchioMapping& pinocchioMapping,
    const state_vector_t& state) {
  Eigen::VectorXd configuration(model.nq);
  configuration.setZero();

  const base_coordinate_t basePose = getBasePose(state);
  const joint_coordinate_t jointPositions = getJointPositions(state);
  const Eigen::Quaternion<scalar_t> baseQuaternion =
      quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));

  configuration.head<3>() = getPositionInOrigin(basePose);
  configuration.segment<4>(3) = baseQuaternion.coeffs();
  configuration.segment<JOINT_COORDINATE_SIZE>(7) =
      pinocchioMapping.getPinocchioJointVector(jointPositions);
  return configuration;
}

Eigen::VectorXd getPinocchioVelocity(
    const pinocchio::Model& model,
    const anymal::QuadrupedPinocchioMapping& pinocchioMapping,
    const state_vector_t& state, const input_vector_t& input) {
  Eigen::VectorXd velocity(model.nv);
  velocity.setZero();

  const base_coordinate_t baseLocalVelocities = getBaseLocalVelocities(state);
  const joint_coordinate_t jointVelocities = getJointVelocities(input);

  velocity.head<3>() = getLinearVelocity(baseLocalVelocities);
  velocity.segment<3>(3) = getAngularVelocity(baseLocalVelocities);
  velocity.segment<JOINT_COORDINATE_SIZE>(6) =
      pinocchioMapping.getPinocchioJointVector(jointVelocities);
  return velocity;
}

void writeMetadata(const std::filesystem::path& outputDir,
                   const std::vector<std::string>& dofNames,
                   const std::vector<std::string>& bodyNames, size_t numFrames,
                   double fps) {
  std::ofstream file(outputDir / "metadata.json");
  if (!file) {
    throw std::runtime_error("Unable to open metadata.json for writing.");
  }

  file << std::setprecision(12);
  file << "{\n";
  file << "  \"fps\": " << jsonFloat(fps) << ",\n";
  file << "  \"num_frames\": " << numFrames << ",\n";
  file << "  \"num_dofs\": " << dofNames.size() << ",\n";
  file << "  \"num_bodies\": " << bodyNames.size() << ",\n";
  file << "  \"dof_names\": ";
  writeStringArrayJson(file, dofNames);
  file << ",\n";
  file << "  \"body_names\": ";
  writeStringArrayJson(file, bodyNames);
  file << ",\n";
  file << "  \"shapes\": {\n";
  file << "    \"dof_positions\": [" << numFrames << ", " << dofNames.size()
       << "],\n";
  file << "    \"dof_velocities\": [" << numFrames << ", " << dofNames.size()
       << "],\n";
  file << "    \"body_positions\": [" << numFrames << ", " << bodyNames.size()
       << ", 3],\n";
  file << "    \"body_rotations\": [" << numFrames << ", " << bodyNames.size()
       << ", 4],\n";
  file << "    \"body_linear_velocities\": [" << numFrames << ", "
       << bodyNames.size() << ", 3],\n";
  file << "    \"body_angular_velocities\": [" << numFrames << ", "
       << bodyNames.size() << ", 3]\n";
  file << "  },\n";
  file << "  \"units\": {\n";
  file << "    \"dof_positions\": \"rad\",\n";
  file << "    \"dof_velocities\": \"rad/s\",\n";
  file << "    \"body_positions\": \"m\",\n";
  file << "    \"body_rotations\": \"unit_quaternion_wxyz\",\n";
  file << "    \"body_linear_velocities\": \"m/s\",\n";
  file << "    \"body_angular_velocities\": \"rad/s\"\n";
  file << "  },\n";
  file << "  \"quaternion_order\": \"wxyz\",\n";
  file << "  \"frame\": \"world\",\n";
  file << "  \"export_directory\": \""
       << jsonEscape(outputDir.string()) << "\",\n";
  file << "  \"amp_requirements\": {\n";
  file << "    \"fps_dtype\": \"float\",\n";
  file << "    \"dof_names_must_match\": \"robot.data.joint_names\",\n";
  file << "    \"dof_names_order_must_match\": true,\n";
  file << "    \"body_names_used_by\": [\"reference_body\", \"key_body_names\"],\n";
  file << "    \"body_positions_are_world_absolute\": true,\n";
  file << "    \"body_rotations_are_wxyz\": true\n";
  file << "  },\n";
  file << "  \"files\": {\n";
  file << "    \"scenario_metadata\": \"scenario_metadata.json\",\n";
  file << "    \"export_schema\": \"export_schema.json\",\n";
  file << "    \"robot_urdf\": \"robot.urdf\",\n";
  file << "    \"terrain_elevation\": \"terrain_elevation.csv\",\n";
  file << "    \"terrain_elevation_points\": \"terrain_elevation_points.csv\",\n";
  file << "    \"terrain_filtered_elevation\": \"terrain_filtered_elevation.csv\",\n";
  file << "    \"terrain_filtered_elevation_points\": \"terrain_filtered_elevation_points.csv\",\n";
  file << "    \"dof_positions\": \"dof_positions.csv\",\n";
  file << "    \"dof_velocities\": \"dof_velocities.csv\",\n";
  file << "    \"body_positions\": \"body_positions.csv\",\n";
  file << "    \"body_rotations\": \"body_rotations.csv\",\n";
  file << "    \"body_linear_velocities\": \"body_linear_velocities.csv\",\n";
  file << "    \"body_angular_velocities\": \"body_angular_velocities.csv\"\n";
  file << "  }\n";
  file << "}\n";
}

void writeExportSchema(const std::filesystem::path& outputDir) {
  std::ofstream file(outputDir / "export_schema.json");
  if (!file) {
    throw std::runtime_error("Unable to open export_schema.json for writing.");
  }

  file << "{\n";
  file << "  \"format\": \"ocs2_amp_motion_csv_bundle\",\n";
  file << "  \"description\": \"Motion data exported after perceptive MPC calculation for AMP/csv2npz conversion.\",\n";
  file << "  \"fields\": {\n";
  file << "    \"fps\": {\n";
  file << "      \"storage\": \"metadata.json\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [],\n";
  file << "      \"usage\": \"AMP computes dt = 1 / fps for interpolation sampling.\"\n";
  file << "    },\n";
  file << "    \"dof_names\": {\n";
  file << "      \"storage\": \"metadata.json\",\n";
  file << "      \"dtype\": \"string[]\",\n";
  file << "      \"shape\": [\"N_dofs\"],\n";
  file << "      \"requirement\": \"Must exactly match robot.data.joint_names, including order. motion_loader.get_dof_index(robot.data.joint_names) will assert on mismatch.\"\n";
  file << "    },\n";
  file << "    \"body_names\": {\n";
  file << "      \"storage\": \"metadata.json\",\n";
  file << "      \"dtype\": \"string[]\",\n";
  file << "      \"shape\": [\"N_bodies\"],\n";
  file << "      \"requirement\": \"Used by get_body_index(reference_body) and get_body_index(key_body_names). Names must match the AMP robot body names.\"\n";
  file << "    },\n";
  file << "    \"dof_positions\": {\n";
  file << "      \"storage\": \"dof_positions.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"N_frames\", \"N_dofs\"],\n";
  file << "      \"unit\": \"rad\",\n";
  file << "      \"csv_layout\": \"First column is time, remaining columns follow dof_names.\"\n";
  file << "    },\n";
  file << "    \"dof_velocities\": {\n";
  file << "      \"storage\": \"dof_velocities.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"N_frames\", \"N_dofs\"],\n";
  file << "      \"unit\": \"rad/s\",\n";
  file << "      \"note\": \"Exported from the MPC input trajectory. csv2npz may also recompute this from dof_positions with finite differences.\"\n";
  file << "    },\n";
  file << "    \"body_positions\": {\n";
  file << "      \"storage\": \"body_positions.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"N_frames\", \"N_bodies\", 3],\n";
  file << "      \"unit\": \"m\",\n";
  file << "      \"frame\": \"world\",\n";
  file << "      \"note\": \"Absolute world-frame positions computed with Pinocchio forward kinematics.\"\n";
  file << "    },\n";
  file << "    \"body_rotations\": {\n";
  file << "      \"storage\": \"body_rotations.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"N_frames\", \"N_bodies\", 4],\n";
  file << "      \"quaternion_order\": \"wxyz\",\n";
  file << "      \"note\": \"World-frame body orientations computed with Pinocchio forward kinematics.\"\n";
  file << "    },\n";
  file << "    \"body_linear_velocities\": {\n";
  file << "      \"storage\": \"body_linear_velocities.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"N_frames\", \"N_bodies\", 3],\n";
  file << "      \"unit\": \"m/s\",\n";
  file << "      \"frame\": \"world\"\n";
  file << "    },\n";
  file << "    \"body_angular_velocities\": {\n";
  file << "      \"storage\": \"body_angular_velocities.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"N_frames\", \"N_bodies\", 3],\n";
  file << "      \"unit\": \"rad/s\",\n";
  file << "      \"frame\": \"world\"\n";
  file << "    },\n";
  file << "    \"robot_urdf\": {\n";
  file << "      \"storage\": \"robot.urdf\",\n";
  file << "      \"dtype\": \"text/xml\",\n";
  file << "      \"usage\": \"Robot model used for Pinocchio forward kinematics and name matching.\"\n";
  file << "    },\n";
  file << "    \"terrain_source_image\": {\n";
  file << "      \"storage\": \"terrain_source.<ext>\",\n";
  file << "      \"dtype\": \"image\",\n";
  file << "      \"usage\": \"Original terrain image selected by terrain_name.\"\n";
  file << "    },\n";
  file << "    \"terrain_elevation\": {\n";
  file << "      \"storage\": \"terrain_elevation.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"rows\", \"cols\"],\n";
  file << "      \"unit\": \"m\",\n";
  file << "      \"frame\": \"world\",\n";
  file << "      \"note\": \"Height map loaded from the terrain image after applying terrain_scale and zeroing height at world origin.\"\n";
  file << "    },\n";
  file << "    \"terrain_elevation_points\": {\n";
  file << "      \"storage\": \"terrain_elevation_points.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"columns\": [\"row\", \"col\", \"x\", \"y\", \"z\"],\n";
  file << "      \"unit\": \"m\",\n";
  file << "      \"frame\": \"world\"\n";
  file << "    },\n";
  file << "    \"terrain_filtered_elevation\": {\n";
  file << "      \"storage\": \"terrain_filtered_elevation.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"shape\": [\"rows\", \"cols\"],\n";
  file << "      \"unit\": \"m\",\n";
  file << "      \"frame\": \"world\",\n";
  file << "      \"note\": \"Filtered/segmented map generated by the perception pipeline.\"\n";
  file << "    },\n";
  file << "    \"terrain_filtered_elevation_points\": {\n";
  file << "      \"storage\": \"terrain_filtered_elevation_points.csv\",\n";
  file << "      \"dtype\": \"float\",\n";
  file << "      \"columns\": [\"row\", \"col\", \"x\", \"y\", \"z\"],\n";
  file << "      \"unit\": \"m\",\n";
  file << "      \"frame\": \"world\"\n";
  file << "    }\n";
  file << "  }\n";
  file << "}\n";
}

void writeRobotAsset(const std::filesystem::path& outputDir,
                     const std::string& urdfString) {
  std::ofstream file(outputDir / "robot.urdf");
  if (!file) {
    throw std::runtime_error("Unable to open robot.urdf for writing.");
  }
  file << urdfString;
  if (urdfString.empty() || urdfString.back() != '\n') {
    file << "\n";
  }
}

std::string copyTerrainSourceImage(const std::filesystem::path& outputDir,
                                   const std::filesystem::path& sourcePath) {
  if (sourcePath.empty() || !std::filesystem::exists(sourcePath)) {
    return "";
  }

  std::string extension = sourcePath.extension().string();
  if (extension.empty()) {
    extension = ".dat";
  }
  const std::filesystem::path destination =
      outputDir / ("terrain_source" + extension);
  std::filesystem::copy_file(sourcePath, destination,
                             std::filesystem::copy_options::overwrite_existing);
  return destination.filename().string();
}

void writeScenarioMetadata(
    const std::filesystem::path& outputDir, const std::string& configName,
    const std::string& urdfSourcePath,
    const std::filesystem::path& terrainSourcePath,
    const std::string& terrainAssetFile, const std::string& terrainName,
    double terrainScale, double forwardVelocity, double forwardDistance,
    bool adaptReferenceToTerrain, const grid_map::GridMap& elevationMap,
    const grid_map::GridMap& filteredMap, const std::string& elevationLayer) {
  std::ofstream file(outputDir / "scenario_metadata.json");
  if (!file) {
    throw std::runtime_error(
        "Unable to open scenario_metadata.json for writing.");
  }

  file << "{\n";
  file << "  \"config_name\": \"" << jsonEscape(configName) << "\",\n";
  file << "  \"robot_urdf\": \"robot.urdf\",\n";
  file << "  \"urdf_source_path\": ";
  writeJsonStringOrNull(file, urdfSourcePath);
  file << ",\n";
  file << "  \"terrain_name\": \"" << jsonEscape(terrainName) << "\",\n";
  file << "  \"terrain_source_path\": \""
       << jsonEscape(terrainSourcePath.string()) << "\",\n";
  file << "  \"terrain_source_image\": ";
  writeJsonStringOrNull(file, terrainAssetFile);
  file << ",\n";
  file << "  \"terrain_scale\": " << jsonFloat(terrainScale) << ",\n";
  file << "  \"forward_velocity\": " << jsonFloat(forwardVelocity) << ",\n";
  file << "  \"forward_distance\": " << jsonFloat(forwardDistance) << ",\n";
  file << "  \"adapt_reference_to_terrain\": "
       << (adaptReferenceToTerrain ? "true" : "false") << ",\n";
  file << "  \"terrain_maps\": {\n";
  file << "    \"elevation\": ";
  writeGridMapInfoJson(file, elevationMap, elevationLayer,
                       "terrain_elevation.csv",
                       "terrain_elevation_points.csv", "    ");
  if (filteredMap.exists(elevationLayer)) {
    file << ",\n";
    file << "    \"filtered_elevation\": ";
    writeGridMapInfoJson(file, filteredMap, elevationLayer,
                         "terrain_filtered_elevation.csv",
                         "terrain_filtered_elevation_points.csv", "    ");
    file << "\n";
  } else {
    file << "\n";
  }
  file << "  }\n";
  file << "}\n";
}

void exportRobotAndTerrainAssets(
    const std::filesystem::path& outputDir, const std::string& urdfString,
    const std::string& urdfSourcePath, const std::string& configName,
    const std::filesystem::path& terrainSourcePath,
    const std::string& terrainName, double terrainScale, double forwardVelocity,
    double forwardDistance, bool adaptReferenceToTerrain,
    const grid_map::GridMap& elevationMap, const grid_map::GridMap& filteredMap,
    const std::string& elevationLayer) {
  writeRobotAsset(outputDir, urdfString);

  const std::string terrainAssetFile =
      copyTerrainSourceImage(outputDir, terrainSourcePath);

  writeGridMapLayerMatrixCsv(elevationMap, elevationLayer,
                             outputDir / "terrain_elevation.csv");
  writeGridMapLayerPointsCsv(elevationMap, elevationLayer,
                             outputDir / "terrain_elevation_points.csv");

  if (filteredMap.exists(elevationLayer)) {
    writeGridMapLayerMatrixCsv(filteredMap, elevationLayer,
                               outputDir / "terrain_filtered_elevation.csv");
    writeGridMapLayerPointsCsv(
        filteredMap, elevationLayer,
        outputDir / "terrain_filtered_elevation_points.csv");
  }

  writeScenarioMetadata(outputDir, configName, urdfSourcePath,
                        terrainSourcePath, terrainAssetFile, terrainName,
                        terrainScale, forwardVelocity, forwardDistance,
                        adaptReferenceToTerrain, elevationMap, filteredMap,
                        elevationLayer);
}

std::filesystem::path makeUniqueRunDirectory(
    const std::filesystem::path& parentOutputDir) {
  std::filesystem::create_directories(parentOutputDir);

  const auto now = std::chrono::system_clock::now();
  const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
  std::tm localTime{};
#if defined(_WIN32)
  localtime_s(&localTime, &nowTime);
#else
  localtime_r(&nowTime, &localTime);
#endif

  std::ostringstream stamp;
  stamp << std::put_time(&localTime, "%Y%m%d_%H%M%S");

  for (int suffix = 0; suffix < 1000; ++suffix) {
    std::ostringstream name;
    name << stamp.str();
    if (suffix > 0) {
      name << "_" << std::setw(3) << std::setfill('0') << suffix;
    }
    const auto runDir = parentOutputDir / name.str();
    if (std::filesystem::create_directory(runDir)) {
      return runDir;
    }
  }

  throw std::runtime_error("Unable to create a unique dataset run directory.");
}

void exportMotionDataset(
    const ocs2::PrimalSolution& systemSolution, const std::string& urdfString,
    const std::string& urdfSourcePath,
    const anymal::FrameDeclaration& frameDeclaration,
    const std::vector<std::string>& dofNames,
    const std::filesystem::path& parentOutputDir, double fps,
    const std::string& configName,
    const std::filesystem::path& terrainSourcePath,
    const std::string& terrainName, double terrainScale,
    double forwardVelocity, double forwardDistance,
    bool adaptReferenceToTerrain, const grid_map::GridMap& elevationMap,
    const grid_map::GridMap& filteredMap, const std::string& elevationLayer) {
  const size_t numFrames =
      std::min({systemSolution.timeTrajectory_.size(),
                systemSolution.stateTrajectory_.size(),
                systemSolution.inputTrajectory_.size()});
  if (numFrames == 0) {
    throw std::runtime_error("No frames available for dataset export.");
  }

  const auto outputDir = makeUniqueRunDirectory(parentOutputDir);
  exportRobotAndTerrainAssets(outputDir, urdfString, urdfSourcePath,
                              configName, terrainSourcePath, terrainName,
                              terrainScale, forwardVelocity, forwardDistance,
                              adaptReferenceToTerrain, elevationMap,
                              filteredMap, elevationLayer);

  const auto pinocchioInterface =
      anymal::createQuadrupedPinocchioInterfaceFromUrdfString(urdfString);
  const auto& model = pinocchioInterface.getModel();
  if (model.nq != 7 + JOINT_COORDINATE_SIZE ||
      model.nv != 6 + JOINT_COORDINATE_SIZE) {
    std::ostringstream error;
    error << "Unexpected Pinocchio model size: nq=" << model.nq
          << ", nv=" << model.nv << ". Expected nq="
          << (7 + JOINT_COORDINATE_SIZE) << ", nv="
          << (6 + JOINT_COORDINATE_SIZE) << ".";
    throw std::runtime_error(error.str());
  }

  anymal::QuadrupedPinocchioMapping pinocchioMapping(frameDeclaration,
                                                     pinocchioInterface);
  std::vector<std::string> bodyNames;
  const auto bodyFrameIds = collectBodyFrameIds(model, bodyNames);
  pinocchio::Data data(model);

  std::ofstream dofPositionsFile(outputDir / "dof_positions.csv");
  std::ofstream dofVelocitiesFile(outputDir / "dof_velocities.csv");
  std::ofstream bodyPositionsFile(outputDir / "body_positions.csv");
  std::ofstream bodyRotationsFile(outputDir / "body_rotations.csv");
  std::ofstream bodyLinearVelocitiesFile(outputDir /
                                         "body_linear_velocities.csv");
  std::ofstream bodyAngularVelocitiesFile(outputDir /
                                          "body_angular_velocities.csv");
  if (!dofPositionsFile || !dofVelocitiesFile || !bodyPositionsFile ||
      !bodyRotationsFile || !bodyLinearVelocitiesFile ||
      !bodyAngularVelocitiesFile) {
    throw std::runtime_error("Unable to open one or more dataset CSV files.");
  }

  dofPositionsFile << std::setprecision(12);
  dofVelocitiesFile << std::setprecision(12);
  bodyPositionsFile << std::setprecision(12);
  bodyRotationsFile << std::setprecision(12);
  bodyLinearVelocitiesFile << std::setprecision(12);
  bodyAngularVelocitiesFile << std::setprecision(12);

  writeCsvHeader(dofPositionsFile, dofNames, {""});
  writeCsvHeader(dofVelocitiesFile, dofNames, {""});
  writeCsvHeader(bodyPositionsFile, bodyNames, {"_x", "_y", "_z"});
  writeCsvHeader(bodyRotationsFile, bodyNames, {"_w", "_x", "_y", "_z"});
  writeCsvHeader(bodyLinearVelocitiesFile, bodyNames, {"_x", "_y", "_z"});
  writeCsvHeader(bodyAngularVelocitiesFile, bodyNames, {"_x", "_y", "_z"});

  for (size_t k = 0; k < numFrames; ++k) {
    const double time = systemSolution.timeTrajectory_[k];
    const state_vector_t state =
        systemSolution.stateTrajectory_[k].head(STATE_DIM);
    const input_vector_t input =
        systemSolution.inputTrajectory_[k].head(INPUT_DIM);
    const joint_coordinate_t jointPositions = getJointPositions(state);
    const joint_coordinate_t jointVelocities = getJointVelocities(input);

    dofPositionsFile << time;
    dofVelocitiesFile << time;
    for (int j = 0; j < JOINT_COORDINATE_SIZE; ++j) {
      dofPositionsFile << "," << jointPositions[j];
      dofVelocitiesFile << "," << jointVelocities[j];
    }
    dofPositionsFile << "\n";
    dofVelocitiesFile << "\n";

    const Eigen::VectorXd q =
        getPinocchioConfiguration(model, pinocchioMapping, state);
    const Eigen::VectorXd v =
        getPinocchioVelocity(model, pinocchioMapping, state, input);
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::updateFramePlacements(model, data);

    bodyPositionsFile << time;
    bodyRotationsFile << time;
    bodyLinearVelocitiesFile << time;
    bodyAngularVelocitiesFile << time;
    for (const auto frameId : bodyFrameIds) {
      const auto& placement = data.oMf[frameId];
      const Eigen::Quaterniond quaternion(placement.rotation());
      const auto frameVelocity = pinocchio::getFrameVelocity(
          model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED);

      bodyPositionsFile << "," << placement.translation().x() << ","
                        << placement.translation().y() << ","
                        << placement.translation().z();
      bodyRotationsFile << "," << quaternion.w() << "," << quaternion.x()
                        << "," << quaternion.y() << "," << quaternion.z();
      bodyLinearVelocitiesFile << "," << frameVelocity.linear().x() << ","
                               << frameVelocity.linear().y() << ","
                               << frameVelocity.linear().z();
      bodyAngularVelocitiesFile << "," << frameVelocity.angular().x() << ","
                                << frameVelocity.angular().y() << ","
                                << frameVelocity.angular().z();
    }
    bodyPositionsFile << "\n";
    bodyRotationsFile << "\n";
    bodyLinearVelocitiesFile << "\n";
    bodyAngularVelocitiesFile << "\n";
  }

  writeMetadata(outputDir, dofNames, bodyNames, numFrames, fps);
  writeExportSchema(outputDir);
  std::cout << "[PerceptiveMpcDemo] Dataset exported to "
            << outputDir.string() << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  const std::string path(__FILE__);
  const std::string ocs2_anymal =
      path.substr(0, path.find_last_of('/')) + "/../../";
  const std::string taskFolder =
      ocs2_anymal + "ocs2_anymal_loopshaping_mpc/config/";
  const std::string terrainFolder =
      ocs2_anymal + "ocs2_anymal_loopshaping_mpc/data/";

  std::string urdfString = getUrdfString(anymal::AnymalModel::Camel);
  const std::string robotName = "anymal";

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared(robotName + "anymal_perceptive_mpc_demo");
  node->declare_parameter("ocs2_anymal_description", "anymal");
  const std::string urdfPath =
      node->get_parameter("ocs2_anymal_description").as_string();
  if (urdfPath != "anymal")
    urdfString = anymal::getUrdfString(urdfPath);
  node->declare_parameter("config_name", "c_series");
  const std::string configName = node->get_parameter("config_name").as_string();
  const std::string configFolder = taskFolder + configName;
  const auto frameDeclaration =
      anymal::frameDeclarationFromFile(configFolder + "/frame_declaration.info");
  node->declare_parameter("export_dataset", false);
  const bool exportDataset = node->get_parameter("export_dataset").as_bool();
  node->declare_parameter("export_dataset_dir",
                          "/tmp/ocs2_perceptive_mpc_dataset");
  const std::string exportDatasetDir =
      node->get_parameter("export_dataset_dir").as_string();

  convex_plane_decomposition::PlaneDecompositionPipeline::Config
      perceptionConfig;
  perceptionConfig.preprocessingParameters =
      convex_plane_decomposition::loadPreprocessingParameters(
          node.get(),
          "/ocs2_anymal_loopshaping_mpc_perceptive_demo/preprocessing/");
  perceptionConfig.contourExtractionParameters =
      convex_plane_decomposition::loadContourExtractionParameters(
          node.get(),
          "/ocs2_anymal_loopshaping_mpc_perceptive_demo/contour_extraction/");
  perceptionConfig.ransacPlaneExtractorParameters =
      convex_plane_decomposition::loadRansacPlaneExtractorParameters(
          node.get(),
          "/ocs2_anymal_loopshaping_mpc_perceptive_demo/"
          "ransac_plane_refinement/");
  perceptionConfig.slidingWindowPlaneExtractorParameters =
      convex_plane_decomposition::loadSlidingWindowPlaneExtractorParameters(
          node.get(),
          "/ocs2_anymal_loopshaping_mpc_perceptive_demo/"
          "sliding_window_plane_extractor/");
  perceptionConfig.postprocessingParameters =
      convex_plane_decomposition::loadPostprocessingParameters(
          node.get(),
          "/ocs2_anymal_loopshaping_mpc_perceptive_demo/postprocessing/");

  auto anymalInterface =
      anymal::getAnymalLoopshapingInterface(urdfString, configFolder);
  const vector_t initSystemState =
      anymalInterface->getInitialState().head(STATE_DIM);

  // ====== Scenario settings ========
  node->declare_parameter("forward_velocity", 0.5);
  scalar_t forwardVelocity =
      node->get_parameter("forward_velocity").as_double();
  scalar_t gaitDuration{0.8};
  node->declare_parameter("forward_distance", 3.0);
  scalar_t forwardDistance =
      node->get_parameter("forward_distance").as_double();

  scalar_t initTime = 0.0;
  scalar_t stanceTime = 1.0;
  int numGaitCycles =
      std::ceil((forwardDistance / forwardVelocity) / gaitDuration);
  scalar_t walkTime = numGaitCycles * gaitDuration;
  scalar_t finalTime = walkTime + 2 * stanceTime;

  // Load a map
  const std::string elevationLayer{"elevation"};
  const std::string frameId{"world"};
  node->declare_parameter("terrain_name", "step.png");
  std::string terrainFile = node->get_parameter("terrain_name").as_string();
  const std::filesystem::path terrainSourcePath =
      std::filesystem::path(terrainFolder) / terrainFile;
  node->declare_parameter("terrain_scale", 0.35);
  double heightScale = node->get_parameter("terrain_scale").as_double();
  auto gridMap = convex_plane_decomposition::loadGridmapFromImage(
      terrainSourcePath.string(), elevationLayer, frameId,
      perceptionConfig.preprocessingParameters.resolution, heightScale);
  gridMap.get(elevationLayer).array() -=
      gridMap.atPosition(elevationLayer, {0., 0.});

  Gait stance;
  stance.duration = stanceTime;
  stance.eventPhases = {};
  stance.modeSequence = {STANCE};

  Gait gait;
  gait.duration = gaitDuration;
  gait.eventPhases = {0.5};
  gait.modeSequence = {LF_RH, RF_LH};

  GaitSchedule::GaitSequence gaitSequence{stance};
  for (int i = 0; i < numGaitCycles; ++i) {
    gaitSequence.push_back(gait);
  }
  gaitSequence.push_back(stance);

  // Reference trajectory
  node->declare_parameter("adaptReferenceToTerrain", true);
  bool adaptReferenceToTerrain =
      node->get_parameter("adaptReferenceToTerrain").as_bool();

  constexpr double dtRef = 0.1;
  const BaseReferenceHorizon commandHorizon{
      dtRef, static_cast<size_t>(walkTime / dtRef) + 1};
  BaseReferenceCommand command{};
  command.baseHeight = getPositionInOrigin(getBasePose(initSystemState)).z();
  command.yawRate = 0.0;
  command.headingVelocity = forwardVelocity;
  command.lateralVelocity = 0.0;

  // ====== Run the perception pipeline ========
  convex_plane_decomposition::PlaneDecompositionPipeline
      planeDecompositionPipeline(perceptionConfig);
  planeDecompositionPipeline.update(grid_map::GridMap(gridMap), elevationLayer);
  auto& planarTerrain = planeDecompositionPipeline.getPlanarTerrain();
  auto terrainModel =
      std::make_unique<SegmentedPlanesTerrainModel>(planarTerrain);

  // Read min-max from elevation map
  constexpr float heightMargin =
      0.5;  // Create SDF till this amount above and below the map.
  const auto& elevationData = gridMap.get(elevationLayer);
  const float minValue = elevationData.minCoeffOfFinites() - heightMargin;
  const float maxValue = elevationData.maxCoeffOfFinites() + heightMargin;
  terrainModel->createSignedDistanceBetween(
      {-1e30, -1e30, minValue},
      {1e30, 1e30, maxValue});  // will project XY range to map limits

  // ====== Generate reference trajectory ========
  const auto& baseToHipInBase =
      anymalInterface->getKinematicModel().baseToLegRootInBaseFrame(0);
  const double nominalStanceWidthInHeading =
      2.0 * (std::abs(baseToHipInBase.x()) + 0.15);
  const double nominalStanceWidthLateral =
      2.0 * (std::abs(baseToHipInBase.y()) + 0.10);

  BaseReferenceState initialBaseState{
      stanceTime, getPositionInOrigin(getBasePose(initSystemState)),
      getOrientation(getBasePose(initSystemState))};

  BaseReferenceTrajectory terrainAdaptedBaseReference;
  if (adaptReferenceToTerrain) {
    terrainAdaptedBaseReference = generateExtrapolatedBaseReference(
        commandHorizon, initialBaseState, command, planarTerrain.gridMap,
        nominalStanceWidthInHeading, nominalStanceWidthLateral);
  } else {
    terrainAdaptedBaseReference = generateExtrapolatedBaseReference(
        commandHorizon, initialBaseState, command, TerrainPlane());
  }

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.push_back(initTime);
  targetTrajectories.timeTrajectory.push_back(stanceTime);
  targetTrajectories.stateTrajectory.push_back(initSystemState);
  targetTrajectories.stateTrajectory.push_back(initSystemState);
  targetTrajectories.inputTrajectory.emplace_back(vector_t::Zero(INPUT_DIM));
  targetTrajectories.inputTrajectory.emplace_back(vector_t::Zero(INPUT_DIM));
  for (int k = 0; k < terrainAdaptedBaseReference.time.size(); ++k) {
    targetTrajectories.timeTrajectory.push_back(
        terrainAdaptedBaseReference.time[k]);

    const auto R_WtoB =
        rotationMatrixOriginToBase(terrainAdaptedBaseReference.eulerXyz[k]);

    Eigen::VectorXd costReference(STATE_DIM);
    costReference << terrainAdaptedBaseReference.eulerXyz[k],
        terrainAdaptedBaseReference.positionInWorld[k],
        R_WtoB * terrainAdaptedBaseReference.angularVelocityInWorld[k],
        R_WtoB * terrainAdaptedBaseReference.linearVelocityInWorld[k],
        getJointPositions(initSystemState);
    targetTrajectories.stateTrajectory.push_back(std::move(costReference));
    targetTrajectories.inputTrajectory.emplace_back(vector_t::Zero(INPUT_DIM));
  }
  targetTrajectories.timeTrajectory.push_back(stanceTime + walkTime);
  targetTrajectories.timeTrajectory.push_back(finalTime);
  targetTrajectories.stateTrajectory.push_back(
      targetTrajectories.stateTrajectory.back());
  targetTrajectories.stateTrajectory.push_back(
      targetTrajectories.stateTrajectory.back());
  targetTrajectories.inputTrajectory.emplace_back(vector_t::Zero(INPUT_DIM));
  targetTrajectories.inputTrajectory.emplace_back(vector_t::Zero(INPUT_DIM));

  // ====== Set the scenario to the correct interfaces ========
  auto referenceManager = anymalInterface->getQuadrupedInterface()
                              .getSwitchedModelModeScheduleManagerPtr();

  // Register the terrain model
  referenceManager->getTerrainModel().reset(std::move(terrainModel));

  // Register the gait
  referenceManager->getGaitSchedule()->setGaitSequenceAtTime(gaitSequence,
                                                             initTime);

  // Register the target trajectory
  referenceManager->setTargetTrajectories(targetTrajectories);

  // ====== Create MPC solver ========
  const auto mpcSettings =
      ocs2::mpc::loadSettings(configFolder + "/task.info");

  std::unique_ptr<ocs2::MPC_BASE> mpcPtr;
  const auto sqpSettings =
      ocs2::sqp::loadSettings(configFolder + "/multiple_shooting.info");
  switch (anymalInterface->modelSettings().algorithm_) {
    case Algorithm::DDP: {
      const auto ddpSettings =
          ocs2::ddp::loadSettings(configFolder + "/task.info");
      mpcPtr = getDdpMpc(*anymalInterface, mpcSettings, ddpSettings);
      break;
    }
    case Algorithm::SQP: {
      mpcPtr = getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
      break;
    }
  }
  ocs2::MPC_MRT_Interface mpcInterface(*mpcPtr);

  std::unique_ptr<ocs2::RolloutBase> rollout(
      anymalInterface->getRollout().clone());

  // ====== Execute the scenario ========
  ocs2::SystemObservation observation;
  observation.time = initTime;
  observation.state = anymalInterface->getInitialState();
  observation.input.setZero(switched_model_loopshaping::INPUT_DIM);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);
  while (!mpcInterface.initialPolicyReceived()) {
    mpcInterface.advanceMpc();
  }

  // run MPC till final time
  constexpr scalar_t mpcSimulationFrequency = 100.0;
  ocs2::PrimalSolution closedLoopSolution;
  std::vector<ocs2::PerformanceIndex> performances;
  while (observation.time < finalTime) {
    std::cout << "t: " << observation.time << "\n";
    try {
      // run MPC at current observation
      mpcInterface.setCurrentObservation(observation);
      mpcInterface.advanceMpc();
      mpcInterface.updatePolicy();

      performances.push_back(mpcInterface.getPerformanceIndices());

      // Evaluate the optimized solution - change to optimal controller
      vector_t tmp;
      mpcInterface.evaluatePolicy(observation.time, observation.state, tmp,
                                  observation.input, observation.mode);
      observation.input = ocs2::LinearInterpolation::interpolate(
          observation.time, mpcInterface.getPolicy().timeTrajectory_,
          mpcInterface.getPolicy().inputTrajectory_);

      closedLoopSolution.timeTrajectory_.push_back(observation.time);
      closedLoopSolution.stateTrajectory_.push_back(observation.state);
      closedLoopSolution.inputTrajectory_.push_back(observation.input);
      if (closedLoopSolution.modeSchedule_.modeSequence.empty()) {
        closedLoopSolution.modeSchedule_.modeSequence.push_back(
            observation.mode);
      } else if (closedLoopSolution.modeSchedule_.modeSequence.back() !=
                 observation.mode) {
        closedLoopSolution.modeSchedule_.modeSequence.push_back(
            observation.mode);
        closedLoopSolution.modeSchedule_.eventTimes.push_back(
            observation.time - 0.5 / mpcSimulationFrequency);
      }

      // perform a rollout
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory, inputTrajectory;
      const scalar_t finalTime =
          observation.time + 1.0 / mpcSimulationFrequency;
      auto modeschedule = mpcInterface.getPolicy().modeSchedule_;
      rollout->run(observation.time, observation.state, finalTime,
                   mpcInterface.getPolicy().controllerPtr_.get(), modeschedule,
                   timeTrajectory, postEventIndicesStock, stateTrajectory,
                   inputTrajectory);

      observation.time = finalTime;
      observation.state = stateTrajectory.back();
      observation.input.setZero(
          switched_model_loopshaping::INPUT_DIM);  // reset
    } catch (std::exception& e) {
      std::cout << "MPC failed\n";
      std::cout << e.what() << "\n";
      break;
    }
  }
  const auto closedLoopSystemSolution = loopshapingToSystemPrimalSolution(
      closedLoopSolution, *anymalInterface->getLoopshapingDefinition());
  if (exportDataset) {
    try {
      exportMotionDataset(closedLoopSystemSolution, urdfString, urdfPath,
                          frameDeclaration, anymalInterface->getJointNames(),
                          exportDatasetDir, mpcSimulationFrequency, configName,
                          terrainSourcePath, terrainFile, heightScale,
                          forwardVelocity, forwardDistance,
                          adaptReferenceToTerrain, gridMap,
                          planarTerrain.gridMap, elevationLayer);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Dataset export failed: " << e.what());
    }
  }

  // ====== Print result ==========
  const auto totalCost = std::accumulate(
      performances.cbegin(), performances.cend(), 0.0,
      [](double v, const ocs2::PerformanceIndex& p) { return v + p.cost; });
  const auto totalDynamics =
      std::accumulate(performances.cbegin(), performances.cend(), 0.0,
                      [](double v, const ocs2::PerformanceIndex& p) {
                        return v + std::sqrt(p.dynamicsViolationSSE);
                      });
  const auto maxDynamics =
      std::sqrt(std::max_element(performances.cbegin(), performances.cend(),
                                 [](const ocs2::PerformanceIndex& lhs,
                                    const ocs2::PerformanceIndex& rhs) {
                                   return lhs.dynamicsViolationSSE <
                                          rhs.dynamicsViolationSSE;
                                 })
                    ->dynamicsViolationSSE);
  const auto totalEquality =
      std::accumulate(performances.cbegin(), performances.cend(), 0.0,
                      [](double v, const ocs2::PerformanceIndex& p) {
                        return v + std::sqrt(p.equalityConstraintsSSE);
                      });
  const auto maxEquality =
      std::sqrt(std::max_element(performances.cbegin(), performances.cend(),
                                 [](const ocs2::PerformanceIndex& lhs,
                                    const ocs2::PerformanceIndex& rhs) {
                                   return lhs.equalityConstraintsSSE <
                                          rhs.equalityConstraintsSSE;
                                 })
                    ->equalityConstraintsSSE);

  double achievedWalkTime = observation.time - stanceTime;
  std::cout << "Speed: " << forwardVelocity << "\n";
  std::cout << "Scale: " << heightScale << "\n";
  std::cout << "Completed: "
            << std::min(1.0, (achievedWalkTime / walkTime)) * 100.0 << "\n";
  std::cout << "average Cost: " << totalCost / performances.size() << "\n";
  std::cout << "average Dynamics constr: "
            << totalDynamics / performances.size() << "\n";
  std::cout << "max Dynamics constr: " << maxDynamics << "\n";
  std::cout << "average Equality constr: "
            << totalEquality / performances.size() << "\n";
  std::cout << "max Equality constr: " << maxEquality << "\n";

  // ====== Visualize ==========
  QuadrupedVisualizer visualizer(anymalInterface->getKinematicModel(),
                                 anymalInterface->getJointNames(),
                                 anymalInterface->getBaseName(), node);
  auto elevationmapPublisher =
      node->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);
  auto filteredmapPublisher =
      node->create_publisher<grid_map_msgs::msg::GridMap>("filtered_map", 1);
  auto boundaryPublisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("boundaries",
                                                                   1);
  auto insetPublisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("insets", 1);
  auto distanceFieldPublisher =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "signed_distance_field", 1);

  // Create pointcloud for visualization (terrain model ownership is now with
  // the swing planner
  const auto* sdfPtr = dynamic_cast<const SegmentedPlanesSignedDistanceField*>(
      referenceManager->getSwingTrajectoryPlanner().getSignedDistanceField());
  sensor_msgs::msg::PointCloud2 pointCloud2Msg;
  if (sdfPtr != nullptr) {
    SegmentedPlanesTerrainModelRos::toPointCloud(
        *sdfPtr, pointCloud2Msg, 1, [](float val) { return val <= 0.0F; });
  }

  // Grid map
  grid_map_msgs::msg::GridMap filteredMapMessage =
      *(grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap));
  grid_map_msgs::msg::GridMap elevationMapMessage =
      *(grid_map::GridMapRosConverter::toMessage(gridMap));

  // Segmentation
  const double lineWidth = 0.005;  // [m] RViz marker size
  auto boundaries = convertBoundariesToRosMarkers(
      planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
      planarTerrain.gridMap.getTimestamp(), lineWidth);
  auto boundaryInsets = convertInsetsToRosMarkers(
      planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
      planarTerrain.gridMap.getTimestamp(), lineWidth);

  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    visualizer.publishOptimizedStateTrajectory(
        node->get_clock()->now(), closedLoopSystemSolution.timeTrajectory_,
        closedLoopSystemSolution.stateTrajectory_,
        closedLoopSystemSolution.modeSchedule_);
    visualizer.publishDesiredTrajectory(node->get_clock()->now(),
                                        targetTrajectories);

    filteredmapPublisher->publish(filteredMapMessage);
    elevationmapPublisher->publish(elevationMapMessage);
    boundaryPublisher->publish(boundaries);
    insetPublisher->publish(boundaryInsets);

    if (sdfPtr != nullptr) {
      distanceFieldPublisher->publish(pointCloud2Msg);
    }

    // Visualize the individual execution
    for (size_t k = 0; k < closedLoopSystemSolution.timeTrajectory_.size() - 1;
         k++) {
      double speed = 1.0;
      double frameDuration =
          speed * (closedLoopSystemSolution.timeTrajectory_[k + 1] -
                   closedLoopSystemSolution.timeTrajectory_[k]);
      if (double publishDuration = ocs2::timedExecutionInSeconds([&]() {
            ocs2::SystemObservation system_observation;
            system_observation.time =
                closedLoopSystemSolution.timeTrajectory_[k];
            system_observation.state =
                closedLoopSystemSolution.stateTrajectory_[k];
            system_observation.input =
                closedLoopSystemSolution.inputTrajectory_[k];
            system_observation.mode =
                closedLoopSystemSolution.modeSchedule_.modeAtTime(
                    system_observation.time);
            visualizer.publishObservation(node->get_clock()->now(),
                                          system_observation);
          });
          frameDuration > publishDuration) {
        const rclcpp::Duration duration =
            rclcpp::Duration::from_seconds(frameDuration - publishDuration);
        rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
      }
    }
    rate.sleep();
  }
}
