/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_custom_plugins/tools/pose/pose_tool.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rviz_custom_plugins
{

namespace tools
{
PoseTool::PoseTool()
: rviz_common::Tool(), arrow_(nullptr), angle_(0)
{
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}
PoseTool::~PoseTool() = default;

void PoseTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void PoseTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void PoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int PoseTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  static double initz;
  static double prevz;
  static double prevangle;
  
  const double z_scale    = 50;
  const double z_interval = 0.5;

  // we need base_orient, since the arrow goes along the -z axis by default
  // (for historical reasons)
  Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI),
                                                Ogre::Vector3::UNIT_Y);

  auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  if (event.leftDown()) 
  { 
    // return processMouseLeftButtonPressed(point_projection_on_xy_plane);
    assert(state_ == Position);
    if (point_projection_on_xy_plane.first) 
    {
      arrow_position_ = point_projection_on_xy_plane.second;
      arrow_->setPosition(arrow_position_);
      state_           = Orientation;
      flags           |= Render;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left()) 
  {
    // return processMouseMoved(point_projection_on_xy_plane);
    if (state_ == Orientation) 
    {
      // compute angle in x-y plane
      if (point_projection_on_xy_plane.first) 
      {
        angle_ = calculateAngle(point_projection_on_xy_plane.second, arrow_position_);
        makeArrowVisibleAndSetOrientation(angle_);

        if ( event.right() )
          state_ = Height;
        
        initz     = arrow_position_.z;
        prevz     = event.y;
        prevangle = angle_;

        flags |= Render;
      }
    }
    if ( state_ == Height )
    {
      double z              = event.y;
      double dz             = z - prevz;
      prevz                 = z;
      arrow_position_.z     -= dz / z_scale;
      arrow_->setPosition( arrow_position_ );
      
      arrow_array.clear();
      
      int cnt = ceil( fabs(initz - arrow_position_.z) / z_interval );
      for (int k = 0; k < cnt; k++)
      {
        std::shared_ptr<rviz_rendering::Arrow> arrow__;

        arrow__ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
        arrow__->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
        arrow__->getSceneNode()->setVisible( true );       
        
        Ogre::Vector3 arr_pos = arrow_position_;
        arr_pos.z             = initz - ( (initz - arrow_position_.z > 0)?1:-1 ) * k * z_interval;
        arrow__->setPosition( arr_pos );
        arrow__->setOrientation( Ogre::Quaternion( Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z ) * orient_x );
        arrow_array.push_back(arrow__); 
      }
      flags |= Render;    
    }
  } 
  else if (event.leftUp()) 
  {
    // return processMouseLeftButtonReleased();
    if( state_ == Orientation || state_ == Height)
    {
      // Create a list of arrows
      arrow_array.clear();

      onPoseSet(arrow_position_.x, arrow_position_.y, arrow_position_.z, prevangle);
      flags |= (Finished|Render);
    }

  }
  return flags;
}

int PoseTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
  return 0;

  // int flags = 0;
  // assert(state_ == Position);
  // if (xy_plane_intersection.first)
  // {
  //   arrow_position_ = xy_plane_intersection.second;
  //   arrow_->setPosition(arrow_position_);

  //   state_ = Orientation;
  //   flags |= Render;
  // }
  // return flags;
}

int PoseTool::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
  return 0;

  // int flags = 0;
  // if (state_ == Orientation) 
  // {
  //   // compute angle in x-y plane
  //   if (xy_plane_intersection.first) 
  //   {
  //     angle_ = calculateAngle(xy_plane_intersection.second, arrow_position_);
  //     makeArrowVisibleAndSetOrientation(angle_);

  //     flags |= Render;
  //   }
  // }

  // return flags;
}

void PoseTool::makeArrowVisibleAndSetOrientation(double angle)
{
  arrow_->getSceneNode()->setVisible(true);

  // we need base_orient, since the arrow goes along the -z axis by default
  // (for historical reasons)
  Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI),
                                                Ogre::Vector3::UNIT_Y);

  arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
}

int PoseTool::processMouseLeftButtonReleased()
{
  return 0;

  // int flags = 0;
  // if (state_ == Orientation) 
  // {
  //   onPoseSet(arrow_position_.x, arrow_position_.y, angle_);
  //   flags |= (Finished | Render);
  // }

  // return flags;
}

double PoseTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
{
  return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
}

geometry_msgs::msg::Quaternion PoseTool::orientationAroundZAxis(double angle)
{
  auto orientation = geometry_msgs::msg::Quaternion();
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = sin(angle) / (2 * cos(angle / 2));
  orientation.w = cos(angle / 2);
  return orientation;
}

void PoseTool::logPose(
  std::string designation, geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation, double angle, std::string frame)
{
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Setting " << designation << " pose: Frame:" << frame << ", Position(" << position.x << ", " <<
      position.y << ", " << position.z << "), Orientation(" << orientation.x << ", " <<
      orientation.y << ", " << orientation.z << ", " << orientation.w <<
      ") = Angle: " << angle);
}

}  // namespace tools
}  // namespace rviz_custom_plugins
