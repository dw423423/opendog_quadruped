/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>

#include "ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h"

namespace ocs2::legged_robot {
    LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface,
                                                         CentroidalModelInfo info,
                                                         const SwingTrajectoryPlanner &swingTrajectoryPlanner,
                                                         ModelSettings settings)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
          mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
          settings_(std::move(settings)) {
        eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
        mappingPtr_->setPinocchioInterface(pinocchioInterface_);
    }


    LeggedRobotPreComputation::LeggedRobotPreComputation(const LeggedRobotPreComputation &rhs)
        : pinocchioInterface_(rhs.pinocchioInterface_),
          info_(rhs.info_),
          swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
          mappingPtr_(rhs.mappingPtr_->clone()),
          settings_(rhs.settings_) {
        eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
        mappingPtr_->setPinocchioInterface(pinocchioInterface_);
    }


    void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t &x, const vector_t &u) {
        // 如果本次求解没有请求代价或约束相关计算，就不需要更新预计算缓存。
        if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
            return;
        }

        // 为指定足端构造法向速度约束配置。SwingTrajectoryPlanner 提供该足端
        // 在时刻 t 的期望 Z 向速度；如果启用 positionErrorGain，还会加入
        // Z 向位置误差反馈，使足端高度贴近规划的摆腿高度曲线。
        auto eeNormalVelConConfig = [&](size_t footIndex) {
            EndEffectorLinearConstraint::Config config;
            config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).
                    finished();
            // Av 只取足端速度的 Z 分量，对应地面法向速度约束。
            config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
            if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
                config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(
                    footIndex, t);
                // Ax 只取足端位置的 Z 分量，用于高度误差反馈。
                config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
            }
            return config;
        };

        // 只有求解器请求约束时，才更新四个足端的 normalVelocity 约束参数。
        if (request.contains(Request::Constraint)) {
            for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
                eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
            }
        }

        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        // 将 OCS2 的状态 x 转成 Pinocchio 使用的广义坐标 q。
        vector_t q = mappingPtr_->getPinocchioJointPosition(x);
        if (request.contains(Request::Approximation)) {
            // 线性化/二次近似阶段需要更完整的运动学、雅可比和质心动力学导数缓存。
            forwardKinematics(model, data, q);
            updateFramePlacements(model, data);
            updateGlobalPlacements(model, data);
            computeJointJacobians(model, data);

            updateCentroidalDynamics(pinocchioInterface_, info_, q);
            vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
            updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
        } else {
            // 普通代价/约束评估只需要足端位姿等基础运动学信息。
            forwardKinematics(model, data, q);
            updateFramePlacements(model, data);
        }
    }
}
