/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <bio_ik/goal_types.h>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

#include <mutex>

namespace bio_ik
{

void BalanceGoal::describe(GoalContext& context) const
{
    Goal::describe(context);
    balance_infos.clear();
    double total = 0.0;
    for(auto& link_name : context.getRobotModel().getLinkModelNames())
    {
        auto link_urdf = context.getRobotModel().getURDF()->getLink(link_name);
        if(!link_urdf) continue;
        if(!link_urdf->inertial) continue;
        const auto& center_urdf = link_urdf->inertial->origin.position;
        tf2::Vector3 center(center_urdf.x, center_urdf.y, center_urdf.z);
        double mass = link_urdf->inertial->mass;
        if(!(mass > 0)) continue;
        balance_infos.emplace_back();
        balance_infos.back().center = center;
        balance_infos.back().weight = mass;
        total += mass;
        context.addLink(link_name);
    }
    for(auto& b : balance_infos)
    {
        b.weight /= total;
    }
}

double BalanceGoal::evaluate(const GoalContext& context) const
{
    tf2::Vector3 center(0, 0, 0);
    for(size_t i = 0; i < balance_infos.size(); i++)
    {
        auto& info = balance_infos[i];
        auto& frame = context.getLinkFrame(i);
        auto c = info.center;
        quat_mul_vec(frame.rot, c, c);
        c += frame.pos;
        center += c * info.weight;
    }
    center -= target_;
    center -= axis_ * axis_.dot(center);
    return center.length2();
}
}
