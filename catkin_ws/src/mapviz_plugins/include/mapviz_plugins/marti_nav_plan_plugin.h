// *****************************************************************************
//
// Copyright (c) 2021, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MAPVIZ_PLUGINS_MARTI_NAV_PLAN_PLUGIN_H_
#define MAPVIZ_PLUGINS_MARTI_NAV_PLAN_PLUGIN_H_

// C++ standard libraries
#include <list>
#include <string>
#include <vector>

#include <mapviz/mapviz_plugin.h>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <marti_nav_msgs/Plan.h>
#include <marti_nav_msgs/PlanTrack.h>
#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_marti_nav_plan_config.h"

namespace mapviz_plugins
{
  class MartiNavPlanPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

   public:
    enum DrawStyle
    {
      STYLE_LINES = 1,
      STYLE_POINTS = 2
    };

    MartiNavPlanPlugin();
    virtual ~MartiNavPlanPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown()
    {
    }

    void Draw(double x, double y, double scale);

    void Transform() {};

    void LoadConfig(const YAML::Node& node, const std::string& path);
    void SaveConfig(YAML::Emitter& emitter, const std::string& path);
    void DrawStopWaypoint(double x, double y);
    void DrawRoute(const marti_nav_msgs::Plan &route);
    void DrawRoutePoint(const marti_nav_msgs::PlanPoint &point);

    QWidget* GetConfigWidget(QWidget* parent);

   protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

   protected Q_SLOTS:
    void SelectTopic();
    void SelectPositionTopic();
    void TopicEdited();
    void PositionTopicEdited();
    void SetDrawStyle(QString style);
    void DrawIcon();

   private:
    Ui::marti_nav_plan_config ui_;
    QWidget* config_widget_;

    int draw_style_;

    std::string topic_;
    std::string position_topic_;

    ros::Subscriber route_sub_;
    ros::Subscriber position_sub_;

    marti_nav_msgs::PlanConstPtr src_route_;
    marti_nav_msgs::PlanTrackConstPtr src_route_position_;
    
    void RouteCallback(const marti_nav_msgs::PlanConstPtr &msg);
    void PositionCallback(const marti_nav_msgs::PlanTrackConstPtr &msg);
  };
}

#endif  // MAPVIZ_PLUGINS_MARTI_NAV_PLAN_PLUGIN_H_