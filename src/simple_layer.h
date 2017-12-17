//from http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
namespace simple_layer_namespace
{
class SimpleLayer : public costmap_2d::Layer
//http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Layer.html
{
public:
SimpleLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
     
private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    double mark_x_, mark_y_;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif