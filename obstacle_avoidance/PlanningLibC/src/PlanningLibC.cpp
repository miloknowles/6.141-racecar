#include <string>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "occupancy_grid.hpp"
#include "point2.hpp"
#include "pose3.hpp"
#include "motion_planning_tree.hpp"

#include <ros/serialization.h>

using namespace boost::python;
using namespace plc;

// Typedefs for common tree structure use-cases.
typedef MotionPlanningTree<Point2f> MotionPlanningTreePose2;
typedef MotionPlanningTree<Pose3> MotionPlanningTreePose3;

/* Read a ROS message from a serialized string. */
template <typename M>
M from_python(const std::string str_msg) {
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i) {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string. */
template <typename M>
std::string to_python(const M& msg) {
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i) {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

class OccupancyGridWrapper : public OccupancyGrid {
 public:
 	// Default constructors are the same.
 	OccupancyGridWrapper() : OccupancyGrid() {};

 	// Deserialize the message and convert to OccupancyGrid.
 	OccupancyGridWrapper(const std::string& msg) : OccupancyGrid() {
		setOccupancyGridString(msg);
 	}

 	// Setting the occupancy grid requires serialization of a msg too.
 	void setOccupancyGridString(const std::string& msg) {
 		nav_msgs::OccupancyGrid og = from_python<nav_msgs::OccupancyGrid>(msg);
 		setOccupancyGrid(og);
 	}

  void dynamicUpdateString(const std::string& msg, float poseX, float poseY, float theta) {
    nav_msgs::OccupancyGrid og = from_python<nav_msgs::OccupancyGrid>(msg);
    Pose3 pose(poseX, poseY, theta);
    dynamicUpdate(og, pose);
  }

  // Serialize the OccupancyGrid msg.
  std::string getOccupancyGridMsgString() {
    nav_msgs::OccupancyGrid msg = getOccupancyGridMsg();
    return to_python<nav_msgs::OccupancyGrid>(msg);
  }
};

BOOST_PYTHON_MODULE(PlanningLibC) {
	class_<Point2f>("Point2f", init<float, float>())
		.def_readwrite("x", &Point2f::x)
		.def_readwrite("y", &Point2f::y)
		.def(self == other<Point2f>())
		.def(self != other<Point2f>())
    .def("distance", &Point2f::distance)
	;

	class_<Point2i>("Point2i", init<int, int>())
		.def_readwrite("x", &Point2i::x)
		.def_readwrite("y", &Point2i::y)
		.def(self == other<Point2i>())
		.def(self != other<Point2i>())
    .def("distance", &Point2i::distance)
	;

  class_<OccupancyGridWrapper>("OccupancyGridWrapper")

  	// Define class methods.
  	.def(init<std::string>())
    .def("pathOccupied", &OccupancyGridWrapper::pathOccupied)
    .def("pathOccupiedVectPoint2f", &OccupancyGridWrapper::pathOccupiedVectPoint2f)
    .def("pathOccupiedVectPose3", &OccupancyGridWrapper::pathOccupiedVectPose3)
    .def("sampleOccupancyGrid", &OccupancyGridWrapper::sampleOccupancyGrid)
    .def("sampleOccupancyGridValid", &OccupancyGridWrapper::sampleOccupancyGridValid)
    .def("getGridValue", &OccupancyGridWrapper::getGridValue)
    .def("getPointValue", &OccupancyGridWrapper::getPointValue)
    .def("setOccupancyGridString", &OccupancyGridWrapper::setOccupancyGridString)
    .def("setGridValue", &OccupancyGridWrapper::setGridValue)
    .def("setPointValue", &OccupancyGridWrapper::setPointValue)
    .def("getMapWidth", &OccupancyGridWrapper::getMapWidth)
    .def("getMapHeight", &OccupancyGridWrapper::getMapHeight)
    .def("getMapResolution", &OccupancyGridWrapper::getMapResolution)
    .def("getMapWidthMeters", &OccupancyGridWrapper::getMapWidthMeters)
    .def("getMapHeightMeters", &OccupancyGridWrapper::getMapHeightMeters)
    .def("getFreeGridCount", &OccupancyGridWrapper::getFreeGridCount)
    .def("getOccupiedGridCount", &OccupancyGridWrapper::getOccupiedGridCount)
    .def("getOriginPoint", &OccupancyGridWrapper::getOriginPoint)
    .def("dilateOccupancyGrid", &OccupancyGridWrapper::dilateOccupancyGrid)
    .def("getOccupancyGridMsgString", &OccupancyGridWrapper::getOccupancyGridMsgString)
    .def("dynamicUpdateString", &OccupancyGridWrapper::dynamicUpdateString)
    .def("updateGridData", &OccupancyGridWrapper::updateGridData)
  ;

  class_<IntegerList>("IntegerList")
    .def(vector_indexing_suite<IntegerList>());

  class_<Point2fList>("Point2fList")
    .def(vector_indexing_suite<Point2fList>());

  class_<MotionPlanningTreePose2>("MotionPlanningTree")
    .def(init<OccupancyGridWrapper>())
    .def("numNodes", &MotionPlanningTreePose2::numNodes)
    .def("nearestNeighbor", &MotionPlanningTreePose2::nearestNeighbor)
    .def("nearestNeighborIndex", &MotionPlanningTreePose2::nearestNeighborIndex)
    .def("getNode", &MotionPlanningTreePose2::getNode)
    .def("getParent", &MotionPlanningTreePose2::getParent)
    .def("getCost", &MotionPlanningTreePose2::getCost)
    .def("addNode", &MotionPlanningTreePose2::addNode)
    .def("setParent", &MotionPlanningTreePose2::setParent)
    .def("setCost", &MotionPlanningTreePose2::setCost)
    .def("getNearbyIndices", &MotionPlanningTreePose2::getNearbyIndices)
    .def("rewireThroughNode", &MotionPlanningTreePose2::rewireThroughNode)
    .def("occupancyGridSet", &MotionPlanningTreePose2::occupancyGridSet)
  ;
}
