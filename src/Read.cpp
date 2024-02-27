/*
 * Read.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "pointcloud_io/Read.hpp"

#include <filesystem>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/TextureMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
// pcl_ros
#include <pcl_ros/transforms.hpp>

#ifdef HAVE_VTK
#include <pcl/io/vtk_lib_io.h>
#endif


namespace pointcloud_io {

bool toRosVertices(
      const pcl::PCLPointCloud2 & cloud,
      std::vector<geometry_msgs::msg::Point>* vertices,
      rclcpp::Logger logger)
{
  /*DEBUG std::cout << "height: " << cloud.height
  << ", width: " << cloud.width
  << ", point_step: " << cloud.point_step
  << ", row_step: " << cloud.row_step
  << ", is_dense: " << (int)cloud.is_dense << std::endl;*/
  size_t x_offset, y_offset, z_offset, stride = cloud.point_step;

  // Usually the fields have size 4 and contains "x", "y", "z", "rgb" from a ply file
  bool type_error = false;
  for(size_t i=0; i< cloud.fields.size(); i++)
  {
    /*DEBUG std::cout << "field: " << cloud.fields[i].name
    << ", offset: " << cloud.fields[i].offset
    << ", dtype: " << (pcl::PCLPointField::PointFieldTypes)cloud.fields[i].datatype
    << ", count: " << cloud.fields[i].count << std::endl;*/
    if(cloud.fields[i].name == "x")
    {
      x_offset = cloud.fields[i].offset;
      type_error |= cloud.fields[i].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
    if(cloud.fields[i].name == "y")
    {
      y_offset = cloud.fields[i].offset;
      type_error |= cloud.fields[i].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
    if(cloud.fields[i].name == "z")
    {
      z_offset = cloud.fields[i].offset;
      type_error |= cloud.fields[i].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
  }
  if(type_error)
  {
    RCLCPP_ERROR(logger,"Only support FLOAT32 format.");
    return false;
  }

  /*std::cout << "xyz: " << x_offset << ", " << y_offset << ", " << z_offset << ", " << std::endl;*/
  vertices->resize(cloud.height*cloud.width);
  for(size_t i=0; i<vertices->size(); i++)
  {
    (*vertices)[i].x = static_cast<double>((*reinterpret_cast<const float*>(&cloud.data[stride*i + x_offset])));
    (*vertices)[i].y = static_cast<double>((*reinterpret_cast<const float*>(&cloud.data[stride*i + y_offset])));
    (*vertices)[i].z = static_cast<double>((*reinterpret_cast<const float*>(&cloud.data[stride*i + z_offset])));
  }
  return true;
}

bool toRosTriangles(
  const std::vector<pcl::Vertices> & polygons,
  std::vector<shape_msgs::msg::MeshTriangle> * triangles,
  rclcpp::Logger logger)
{
  triangles->resize(polygons.size() );
  for(size_t i=0; i<triangles->size(); i++)
  {
    if(polygons[i].vertices.size() != 3)
    {
      RCLCPP_ERROR(logger,"Detecting non-triangel primatives");
    }
    (*triangles)[i].vertex_indices[0] = polygons[i].vertices[0];
    (*triangles)[i].vertex_indices[1] = polygons[i].vertices[1];
    (*triangles)[i].vertex_indices[2] = polygons[i].vertices[2];
  }
  return true;
}

void toRosRgba(
  const pcl::TexMaterial::RGB & pcl,
  std_msgs::msg::ColorRGBA * ros)
{
  ros->r = pcl.r;
  ros->g = pcl.g;
  ros->b = pcl.b;
  ros->a = 1.0;
}

bool pclToRclcpp(
      const pcl::PolygonMesh &pclmesh,
      shape_msgs::msg::Mesh::SharedPtr & msg,
      rclcpp::Logger logger)
{
  msg = std::make_shared<shape_msgs::msg::Mesh>();

  bool res = toRosVertices(pclmesh.cloud, &(msg->vertices), logger);
  res &= toRosTriangles(pclmesh.polygons, &(msg->triangles), logger);

  return res;
}

bool pclToRclcpp(
      const pcl::TextureMesh &pcltexmesh,
      pointcloud_io::msg::TextureMesh::SharedPtr & msg,
      rclcpp::Logger logger)
{
  msg = std::make_shared<pointcloud_io::msg::TextureMesh>();
  //msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "/physiog";

  //pcltexmesh.cloud
  if( !toRosTriangles(pcltexmesh.tex_polygons[0], &(msg->triangles), logger) ) {
    return false;
  }

  if( !toRosVertices(pcltexmesh.cloud, &(msg->vertices), logger) ) {
    return false;
  }

  msg->uv_coordinates.resize( pcltexmesh.tex_coordinates[0].size() );
  for(size_t i=0; i<pcltexmesh.tex_coordinates[0].size(); i++ )
  {
    msg->uv_coordinates[i].u = pcltexmesh.tex_coordinates[0][i].x();
    msg->uv_coordinates[i].v = pcltexmesh.tex_coordinates[0][i].y();
  }
  msg->texture_name = pcltexmesh.tex_materials[0].tex_name;

  std::unordered_set<std::string> filetypes = { ".jpeg", ".png", ".tiff", ".jpg"};
  std::string ext = std::filesystem::path(pcltexmesh.tex_materials[0].tex_file).extension();
  if(filetypes.find(ext) == filetypes.end())
  {
    RCLCPP_ERROR_STREAM(logger,"Texture file type: " << ext << " not supported. Only supports jpg, jpeg, png, and tiff.");
    return false;
  }
  std::ifstream infile(pcltexmesh.tex_materials[0].tex_file, std::ios::binary);
  if( !infile.is_open() )
  {
    RCLCPP_ERROR_STREAM(logger,"File opening failed: " << pcltexmesh.tex_materials[0].tex_file);
    return false;
  }
  infile.seekg(0, std::ios::end);     //go to end of file
  size_t fsize = infile.tellg();
  infile.seekg(0, infile.beg);
  // read data as a block:
  //msg->texture.header; // left empty
  msg->texture.format = ext.substr(1, ext.length());
  msg->texture.data.resize(fsize);
  infile.read((char*)msg->texture.data.data(), fsize);
  infile.close();

  toRosRgba(pcltexmesh.tex_materials[0].tex_Ka, &(msg->ka) );
  toRosRgba(pcltexmesh.tex_materials[0].tex_Kd, &(msg->kd) );
  toRosRgba(pcltexmesh.tex_materials[0].tex_Ks, &(msg->ks) );
  msg->alpha = pcltexmesh.tex_materials[0].tex_d;
  msg->ns = pcltexmesh.tex_materials[0].tex_Ns;
  msg->illumination = pcltexmesh.tex_materials[0].tex_illum;

  //msg->color = std_msgs::msg::ColorRGBA().set__r(0.0f).set__g(0.0f).set__b(0.0f).set__a(1.0f);
  return true;
}

Read::Read() :
  rclcpp::Node("mesh_publisher"),
  pointCloudMessage_(new sensor_msgs::msg::PointCloud2())
{
  if (!readParameters()) {
    RCLCPP_WARN(this->get_logger(),"Shutting down due to missing parameters");
    rclcpp::shutdown();
  }

  pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    pointCloudTopic_, rclcpp::QoS(1));

  if(!meshTopic_.empty())
  {
    meshPublisher_ = this->create_publisher<shape_msgs::msg::Mesh>(
      meshTopic_, rclcpp::QoS(1));
  }
  if( !texMeshTopic_.empty() )
  {
    texMeshPublisher_ = this->create_publisher<pointcloud_io::msg::TextureMesh>(
      texMeshTopic_, rclcpp::QoS(1));
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  srvGetMesh_ = this->create_service<ServiceGetMesh>(
    "get_mesh", std::bind(&Read::ServeMesh, this, _1, _2));
  srvGetTextureMesh_ = this->create_service<ServiceGetTextureMesh>(
    "get_texture_mesh", std::bind(&Read::ServeTextureMesh, this, _1, _2));
  initialize();
}

bool Read::readParameters() {

  this->declare_parameter("file_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
  this->declare_parameter("rate", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("mesh_topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("texturemesh_topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("offset", offset_);
  this->declare_parameter("rpy_deg", ypr_);
  this->declare_parameter("scale", scale_);

  std::vector<rclcpp::Parameter> params;
  // throws exception if not found
  params = this->get_parameters({"file_path", "topic", "frame_id"});

  filePath_ = params[0].as_string();
  pointCloudTopic_ = params[1].as_string();
  pointCloudFrameId_ = params[2].as_string();

  rclcpp::Parameter rate_param("rate", 0.0);
  this->get_parameter("rate", rate_param);
  double updateRate = rate_param.as_double();
  RCLCPP_INFO(this->get_logger(), "Rate: %f", updateRate);
  if (updateRate <= 0.0) {
    isContinuouslyPublishing_ = false; // TODO: this is broken in ROS2
  } else {
    isContinuouslyPublishing_ = true;
    updateDuration_ = std::chrono::microseconds((long int)(1000000.0 / updateRate) );
    RCLCPP_INFO(this->get_logger(), "Publishing mesh at %f hz", updateRate);
  }

  this->get_parameter("mesh_topic", meshTopic_);
  this->get_parameter("texturemesh_topic", texMeshTopic_);

  this->get_parameter("offset", offset_);
  this->get_parameter("rpy_deg", ypr_);
  this->get_parameter("scale", scale_);

  return true;
}

void Read::initialize() {
  if (!readFile(filePath_, pointCloudFrameId_)) {
    RCLCPP_WARN(this->get_logger(),"Shutting down2");
    rclcpp::shutdown();
  }

  if (isContinuouslyPublishing_) {
    timer_ = this->create_wall_timer(updateDuration_, std::bind(&Read::timerCallback, this));

  } else {
    // Need this to get things ready before publishing.
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    if (!publish()) {
      RCLCPP_ERROR(this->get_logger(),"Something went wrong when trying to read and publish the point cloud file.");
    }
    // This causes exception
    RCLCPP_WARN(this->get_logger(),"Single shot published. Shutting down.");
    rclcpp::shutdown();
  }
}

bool Read::readFile(
      const std::string& filePath,
      const std::string& pointCloudFrameId)
{
  if (std::filesystem::path(filePath).extension() == ".ply") {
    pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloud;
    if (pcl::io::loadPLYFile(filePath, pointCloud) != 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot load PLY file: " << filePath);
      return false;
    }

    // Define PointCloud2 message.
    pcl::toROSMsg(pointCloud, *pointCloudMessage_);
  }
#ifdef HAVE_VTK
  else if (std::filesystem::path(filePath).extension() == ".vtk") {
    // Load .vtk file.
    pcl::PolygonMesh polygonMesh;
    pcl::io::loadPolygonFileVTK(filePath, polygonMesh);

    // Define PointCloud2 message.
    pcl_conversions::moveFromPCL(polygonMesh.cloud, *pointCloudMessage_);
  }
#endif
  else if (std::filesystem::path(filePath).extension() == ".obj") {
    pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloud;
    if (pcl::io::loadOBJFile(filePath, pointCloud) != 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot load PLY file: " << filePath);
      return false;
    }
    // Define PointCloud2 message.
    pcl::toROSMsg(pointCloud, *pointCloudMessage_);
  }
  else {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Data format not supported: " << std::filesystem::path(filePath).filename());
    return false;
  }

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  // Rotation, scale, and offset
  RCLCPP_INFO(this->get_logger(),"Set scale to: %f", scale_);
  RCLCPP_INFO(this->get_logger(),"Set eular ypr to: [%f, %f, %f]", ypr_[0], ypr_[1], ypr_[2]);
  const double TO_RAD = EIGEN_PI/180.0;
  //Eigen::Affine3d tf;
  //tf = Eigen::AngleAxisd(ypr_[0]*TO_RAD, Eigen::Vector3d::UnitX())
  //      * Eigen::AngleAxisd(ypr_[1]*TO_RAD, Eigen::Vector3d::UnitY())
  //      * Eigen::AngleAxisd(ypr_[2]*TO_RAD, Eigen::Vector3d::UnitZ())
  //      * Eigen::Scaling(scale_);
  //tf.scale(scale_);
  Eigen::Affine3d tf = Eigen::Translation3d(Eigen::Vector3d(offset_.data()))
        * Eigen::AngleAxisd(ypr_[0]*TO_RAD, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(ypr_[1]*TO_RAD, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ypr_[2]*TO_RAD, Eigen::Vector3d::UnitZ())
        * Eigen::Scaling(scale_);
  pcl_ros::transformPointCloud(tf.cast<float>().matrix(), *pointCloudMessage_, *pointCloudMessage_);

  RCLCPP_INFO(this->get_logger(),"Loaded point cloud msg with %d points", pointCloudMessage_->height * pointCloudMessage_->width);

  if(!meshTopic_.empty())
  {
    pcl::PolygonMesh polygonMesh;
    if( std::filesystem::path(filePath).extension() == ".ply" &&
        pcl::io::loadPLYFile(filePath, polygonMesh) !=0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot load PLY file: " << std::filesystem::path(filePath).filename());
      return false;
    }
    if( std::filesystem::path(filePath).extension() == ".obj" &&
        pcl::io::loadOBJFile(filePath, polygonMesh) !=0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot load OBJ file: " << std::filesystem::path(filePath).filename());
      return false;
    }

    // Transform the point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pcl::fromPCLPointCloud2(polygonMesh.cloud, pointCloud);
    pcl::transformPointCloud(pointCloud, pointCloud, tf);
    pcl::toPCLPointCloud2(pointCloud, polygonMesh.cloud);

    bool res = pclToRclcpp(polygonMesh, meshMessage_, this->get_logger());
    if( !res ) {
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"Loaded mesh msg with " << meshMessage_->vertices.size() << " points.");
  }

  if( !texMeshTopic_.empty() )
  {
    // Note: this is a hack, see https://github.com/PointCloudLibrary/pcl/issues/2252
    pcl::TextureMesh textureMseh;
    pcl::TextureMesh mesh2;
    int ret = pcl::io::loadPolygonFileOBJ(filePath, textureMseh);
    std::cout << "ret: " << ret << std::endl;
    if( pcl::io::loadOBJFile(filePath, mesh2) != 0 ) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open OBJ file: " << std::filesystem::path(filePath));
      return false;
    }
    textureMseh.tex_materials = mesh2.tex_materials;

    if( textureMseh.tex_materials.size()!= 1) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "OBJ contains " << textureMseh.tex_materials.size() << " textures. We only support one");
      return false;
    }

    // Transform the point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pc_temp;
    pcl::fromPCLPointCloud2(textureMseh.cloud, pc_temp);
    pcl::transformPointCloud(pc_temp, pc_temp, tf);
    pcl::toPCLPointCloud2(pc_temp, textureMseh.cloud);
    //Debug std::cout << "textureMseh.cloud.width: " << textureMseh.cloud.width
    //          << "\ntextureMseh.cloud.height: " << textureMseh.cloud.height
    //          << "\ntextureMseh.tex_polygons.size: " << textureMseh.tex_polygons.size()
    //          << "\ntextureMseh.tex_coordinates.size: " << textureMseh.tex_coordinates.size()
    //          << "\ntextureMseh.tex_materials.size: " << textureMseh.tex_materials.size()
    //          << std::endl;
    bool res = pclToRclcpp(textureMseh, texMeshMessage_, this->get_logger());
    if( !res ) {
      RCLCPP_ERROR(this->get_logger(), "Texture conversion failed");
      return false;
    }
    RCLCPP_INFO(this->get_logger(),
                "Loaded textured mesh msg with %ld points, %ld triangles, %ld uv-coords, and a texture data size: %ld bytes",
                texMeshMessage_->vertices.size(),
                texMeshMessage_->triangles.size(),
                texMeshMessage_->uv_coordinates.size(),
                texMeshMessage_->texture.data.size());
  }
  return true;
}

void Read::timerCallback() {
  if (!publish()) {
    RCLCPP_ERROR(this->get_logger(),"Something went wrong when trying to read and publish the point cloud file.");
  }
}

bool Read::publish() {
  pointCloudMessage_->header.stamp = this->get_clock()->now();
  if (pointCloudPublisher_->get_subscription_count() > 0u) {
    pointCloudPublisher_->publish(*pointCloudMessage_);
    RCLCPP_DEBUG_STREAM(this->get_logger(),"Point cloud published to topic \"" << pointCloudTopic_ << "\".");
  }
  if( meshPublisher_ && meshPublisher_->get_subscription_count() > 0u) {
    meshPublisher_->publish(*meshMessage_);
    RCLCPP_DEBUG_STREAM(this->get_logger(),"Mesh published to topic \"" << meshTopic_ << "\".");
  }

  if( texMeshPublisher_ ) {
    texMeshPublisher_->publish( *texMeshMessage_ );
    RCLCPP_DEBUG_STREAM(this->get_logger(),"TextureMesh published to topic \"" << texMeshTopic_ << "\".");
  }
  return true;
}

void Read::ServeMesh(
      const std::shared_ptr<ServiceGetMesh::Request> request,
      std::shared_ptr<ServiceGetMesh::Response>      response)
{
  (void)request;  // supress warning
  RCLCPP_INFO(this->get_logger(), "Service: sending mesh");
  if(!meshMessage_)
  {
    RCLCPP_ERROR(this->get_logger(), "Mesh not loaded!");
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "Serving mesh with with %ld vertices and %ld triangles.",
              meshMessage_->vertices.size(), meshMessage_->triangles.size());
  response->mesh = *meshMessage_;
}

void Read::ServeTextureMesh(
      const std::shared_ptr<ServiceGetTextureMesh::Request> request,
      std::shared_ptr<ServiceGetTextureMesh::Response>      response)
{
  (void)request;  // supress warning
  RCLCPP_INFO(this->get_logger(), "Service: sending textured mesh");
  if(!texMeshMessage_)
  {
    RCLCPP_ERROR(this->get_logger(), "Textured mesh not loaded!");
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "Serving texture mesh with with  %ld points, %ld triangles, %ld uv-coords, and a texture data size: %ld bytes",
              texMeshMessage_->vertices.size(),
              texMeshMessage_->triangles.size(),
              texMeshMessage_->uv_coordinates.size(),
              texMeshMessage_->texture.data.size());
  response->texture_mesh = *texMeshMessage_;
}


}  // namespace pointcloud_io
