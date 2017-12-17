#include <simple_layers/simple_layer.h> 
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
//what's pluginglib: http://wiki.ros.org/pluginlib
//In order to allow a class to be dynamically loaded, it must be marked as an exported class. 
//This is done through the special macro PLUGINLIB_EXPORT_CLASS

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
    ros::NodeHandle nh("~/"+name_);
    current_ = true;

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &SimpleLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled; 
}


//The updateBounds method does not change the costmap just yet. 
//It just defines the area that will need to be updated. We calculate the point we want to change (mark_x_, mark_y_) 
//and then expand the min/max bounds to be sure it includes the new point. 
void SimpleLayer::updateBounds(double robot_x,double robot_y,double robot_yaw,double* min_x,double* min_y,double* max_x,double* max_y)
{
    if(!enabled_) return;
    mark_x_ = robot_x + cos(robot_yaw);
    mark_y_ = robot_y + sin(robot_yaw);

    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
}

//First, we calculate which grid cell our point is in using worldToMap. Then we set the cost of that cell. Pretty simple. 
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,int max_j)
{
    if(!enabled_) return;
    unsigned int mx;
    unsigned int my;
    if(master_grid.worldToMap(mark_x_,mark_y_,mx,my)){
        master_grid.setCost(mx,my,LETHAL_OBSTACLE);
    }
}

}