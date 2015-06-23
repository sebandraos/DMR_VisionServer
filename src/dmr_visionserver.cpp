//Native Includes
#include <iostream>
#include <limits>
#include <fstream>
#include <vector>
#include <time.h>
#define NOMINMAX
#include <windows.h>

//Eigen Includes
#include <Eigen/Core>

//Boost Includes
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//PCL Includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/common/centroid.h>
#include <pcl/common/intersections.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/common/geometry.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/io_exception.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//PCL RealSense Includes
#include "real_sense_grabber.h"
#include "real_sense/real_sense_device_manager.h"

//Standard RealSense Includes
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "service/pxcsessionservice.h"

#include "pxchanddata.h"
#include "pxchandconfiguration.h"

//OSC Includes
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#define ADDRESS "192.168.1.255"
#define PORTIN 7000
#define PORTOUT 7001

#define OUTPUT_BUFFER_SIZE 1024

typedef pcl::PointXYZRGBA RefPointType;
typedef pcl::PointCloud<RefPointType> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ> BWCloud;
typedef Cloud::Ptr CloudPtr;
typedef BWCloud::Ptr BWCloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//User Switches
bool show_keypoints_ (false);
bool export_finds_(false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool xyz_only_ (false);
bool tracking_(true);
int print_level_(6);
//0. Features, 1. ParticleFilter, 2. Hough, 3. Geometric Consistency, 4. Planar Decomposition
std::string algorithms[] = {"Features","ParticleFilter","Hough","GC", "PD"};
int algorithm_(4);
bool hand_search(false);

//Algorithm Parameters
float model_ss_(0.005f);
float scene_ss_(0.01f);
float rf_rad_(0.015f);
float descr_rad_(0.01f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);
float downsampling_grid_size_(0.005f);
int max_normal_estimation_neighbours_(50);
int kd_depth_(15);
int icp_iterations_(120);
int min_plane_points(50);
float object_dim_tolerance(0.0075);
float plane_threshold(0.002);
float parallel_threshold(0.75);
float model_plane_threshold(0.005);
bool hands_available(false);
bool fake_search(false);
std::string search_obj;
bool filter_by_colour(false);
std::string colour_filter;
bool transmit(false);
bool approach(true);
int lost_object_counter(0);
float percentage_move(0.2);
float minimum_move(0.01);
pcl::PointXYZ offset(0,0,0.2);
bool export_objects(false);

//Objects
Cloud::Ptr cloud_pass_;
Cloud::Ptr cloud_pass_downsampled_;

//Helpers
std::string file_ending_(".xml");
std::string object_location_("objects");
std::string object_clouds_location_("pointclouds");
std::string coef_point_filename_("relLocation.txt");
boost::format object_locator_("%s/%s");
int poss_objects_(0);
int out_file_num(0);
Eigen::Matrix3f camToTool(3,3);

//OSC Temp
boost::mutex oscMutex;
const char *a4;

//Helper Functions----------------------------------------------------------------------//
void print_with_level(int level,const char* output){
	if (level<=print_level_){
		pcl::console::print_info(output);
	}
}

void print_with_level(int level,std::string output){
	if (level<=print_level_){
		pcl::console::print_info(output.c_str());
	}
}

//TODO Update printHelp
void printHelp ()
{
	std::cout << std::endl;
	std::cout << "****************************************************************************" << std::endl;
	std::cout << "*                                                                          *" << std::endl;
	std::cout << "*                        REAL SENSE VIEWER - Usage Guide                   *" << std::endl;
	std::cout << "*                                                                          *" << std::endl;
	std::cout << "****************************************************************************" << std::endl;
	std::cout << std::endl;
	std::cout << "Usage: " << " [Options] device_id" << std::endl;
	std::cout << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << std::endl;
	std::cout << "     --help, -h : Show this help"                                             << std::endl;
	std::cout << "     --list, -l : List connected RealSense devices"                           << std::endl;
	std::cout << "     --xyz      : View XYZ-only clouds"                                       << std::endl;
	std::cout << std::endl;
	std::cout << "Keyboard commands:"                                                           << std::endl;
	std::cout << std::endl;
	std::cout << "   When the focus is on the viewer window, the following keyboard commands"   << std::endl;
	std::cout << "   are available:"                                                            << std::endl;
	std::cout << "     * t/T : increase or decrease depth data confidence threshold"            << std::endl;
	std::cout << "     * k   : enable next temporal filtering method"                           << std::endl;
	std::cout << "     * b   : toggle bilateral filtering"                                      << std::endl;
	std::cout << "     * a/A : increase or decrease bilateral filter spatial sigma"             << std::endl;
	std::cout << "     * z/Z : increase or decrease bilateral filter range sigma"               << std::endl;
	std::cout << "     * s   : save the last grabbed cloud to disk"                             << std::endl;
	std::cout << "     * h   : print the list of standard PCL viewer commands"                  << std::endl;
	std::cout << std::endl;
	std::cout << "Notes:"                                                                       << std::endl;
	std::cout << std::endl;
	std::cout << "   The device to grab data from is selected using device_id argument. It"     << std::endl;
	std::cout << "   could be either:"                                                          << std::endl;
	std::cout << "     * serial number (e.g. 231400041-03)"                                     << std::endl;
	std::cout << "     * device index (e.g. #2 for the second connected device)"                << std::endl;
	std::cout << std::endl;
	std::cout << "   If device_id is not given, then the first available device will be used."  << std::endl;
	std::cout << std::endl;
}

void printDeviceList ()
{
	typedef boost::shared_ptr<pcl::RealSenseGrabber> RealSenseGrabberPtr;
	std::vector<RealSenseGrabberPtr> grabbers;
	std::cout << "Connected devices: ";
	boost::format fmt ("\n  #%i  %s");
	while (true)
	{
		try
		{
			grabbers.push_back (RealSenseGrabberPtr (new pcl::RealSenseGrabber));
			std::cout << boost::str (fmt % grabbers.size () % grabbers.back ()->getDeviceSerialNumber ());
		}
		catch (pcl::io::IOException& e)
		{
			break;
		}
	}
	if (grabbers.size ())
		std::cout << std::endl;
	else
		std::cout << "none" << std::endl;
}

void parseCommandLine(int argc, char *argv[])
{

	if (pcl::console::find_switch(argc, argv, "--help") || pcl::console::find_switch(argc, argv, "-h"))
	{
		printHelp();
		return;
	}

	if (pcl::console::find_switch(argc, argv, "--list") || pcl::console::find_switch(argc, argv, "-l"))
	{
		printDeviceList();
		return;
	}

	//Program behavior


	pcl::console::parse_argument(argc, argv, "--print_level", print_level_);
	if (print_level_<0){
		print_level_ = 0;
	}

	show_keypoints_ = pcl::console::find_switch(argc, argv, "--k");
	show_correspondences_ = pcl::console::find_switch(argc, argv, "--c");
	use_cloud_resolution_ = pcl::console::find_switch(argc, argv, "--r");
	xyz_only_ = pcl::console::find_switch(argc, argv, "--xyz");
	export_finds_ = pcl::console::find_switch(argc, argv, "--e");

	pcl::console::parse_argument(argc, argv, "--model_ss", model_ss_);
	pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
	pcl::console::parse_argument(argc, argv, "--rf_rad", rf_rad_);
	pcl::console::parse_argument(argc, argv, "--descr_rad", descr_rad_);
	pcl::console::parse_argument(argc, argv, "--cg_size", cg_size_);
	pcl::console::parse_argument(argc, argv, "--cg_thresh", cg_thresh_);
	pcl::console::parse_argument(argc, argv, "--downsample", downsampling_grid_size_);
	pcl::console::parse_argument(argc, argv, "--plane_thresh", plane_threshold);
	pcl::console::parse_argument(argc, argv, "--model_tolerance", object_dim_tolerance);
	pcl::console::parse_argument(argc, argv, "--model_plane_thresh", model_plane_threshold);

	float factor;
	pcl::console::parse_argument(argc, argv, "--model_plane_factor", factor);
	model_plane_threshold=downsampling_grid_size_*factor;

	if (print_level_>0){
		cout << "Show Keypoints: " << show_keypoints_ << endl;
		cout << "Show Correspondences: " << show_keypoints_ << endl;
		cout << "Model Sample Size: " << model_ss_ << endl;
		cout << "Scene Sample Size: " << scene_ss_ << endl;
		cout << "RefFrame Radius: " << rf_rad_ << endl;
		cout << "Descriptor Radius: " << descr_rad_ << endl;
		cout << "CG Size: " << cg_size_ << endl;
		cout << "CG Threshold: " << cg_thresh_ << endl;
		cout << "Downsampling Grid Size: " << downsampling_grid_size_ << endl;
	}

	std::string used_algorithm;
	if (pcl::console::parse_argument(argc, argv, "--algorithm", used_algorithm) != -1)
	{
		//TODO: add algorithms index finder
		if (used_algorithm.compare("ParticleFilter") == 0)
		{
			print_with_level(1,"Algorithm in use: Particle Filter\n");
			algorithm_ = 1;
		}
		else if (used_algorithm.compare("Hough") == 0)
		{
			print_with_level(1,"Algorithm in use: Hough\n");
			algorithm_ = 2;
		}
		else if (used_algorithm.compare("GC") == 0)
		{
			print_with_level(1,"Algorithm in use: GeometricConsistency\n");
			algorithm_ = 3;
		}
		else if (used_algorithm.compare("Features") == 0)
		{
			print_with_level(1,"Algorithm in use: Features\n");
			algorithm_ = 0;
		}
		else if (used_algorithm.compare("PD") == 0)
		{
			print_with_level(1,"Algorithm in use: Planar Decomposition\n");
			algorithm_ = 4;
		}
		else
		{
			PCL_WARN("Incorrect algorithm name.\n");
			PCL_WARN("Running default algorithm until otherwise instructed.\n");
			printHelp();
		}
	}
}

int& referifier(int& f, int target_val)
{
	f = target_val;
	return f;
}

bool hasEnding (std::string const &fullString, std::string const &ending) {
	if (fullString.length() >= ending.length()) {
		return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
	} else {
		return false;
	}
}

template<typename PointT> inline
	float sqDist(const PointT p1, const PointT p2){
		Eigen::Vector3f diff(p1.x-p2.x,p1.y-p2.y,p1.z-p2.z);
		return (diff.squaredNorm ());
}

struct point_indices_descending_size_sort
{
	inline bool operator() (const pcl::PointIndices& struct1, const pcl::PointIndices& struct2)
	{
		return (struct1.indices.size() > struct2.indices.size());
	}
};

//Real Functions------------------------------------------------------------------------//
void filterPassThrough(const Cloud::ConstPtr &cloud, Cloud &result)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 10.0);
	pass.setKeepOrganized(false);
	pass.setInputCloud(cloud);
	pass.filter(result);
}

void gridSampleApprox(const Cloud::ConstPtr &cloud, Cloud &result, double leaf_size)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
}

float planeDist(const pcl::ModelCoefficients plane, pcl::PointXYZ pt){
	pcl::PointXYZ ptOnPlane((-plane.values[3])/plane.values[0],0,0);
	Eigen::Vector3f dif(pt.x-ptOnPlane.x,pt.y-ptOnPlane.y,pt.z-ptOnPlane.z);
	Eigen::Vector3f normal(plane.values[0],plane.values[1],plane.values[2]);
	return float(std::abs(normal.dot(dif))/std::abs(normal.norm()));
}

float signedPlaneDist(const pcl::ModelCoefficients plane, pcl::PointXYZ pt){
	pcl::PointXYZ ptOnPlane((-plane.values[3])/plane.values[0],0,0);
	Eigen::Vector3f dif(pt.x-ptOnPlane.x,pt.y-ptOnPlane.y,pt.z-ptOnPlane.z);
	Eigen::Vector3f normal(plane.values[0],plane.values[1],plane.values[2]);
	return float(normal.dot(dif)/normal.norm());
}

float cosineSimilarity(Eigen::Vector3f axisA, Eigen::Vector3f axisB){
	return (axisA.dot(axisB)/(axisA.norm()*axisB.norm()));
}

float cosineSimilarity(pcl::ModelCoefficients::Ptr plane_a, pcl::ModelCoefficients::Ptr plane_b){
	Eigen::Vector3f axisA(plane_a->values[0],plane_a->values[1],plane_a->values[2]);
	Eigen::Vector3f axisB(plane_b->values[0],plane_b->values[1],plane_b->values[2]);
	return (axisA.dot(axisB)/(axisA.norm()*axisB.norm()));
}

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

Eigen::Matrix4f boxTrans( Eigen::Vector3f zAxis, Eigen::Vector3f xAxis, pcl::PointXYZ centroid){  
	// make sure that we actually have two unique vectors.
	Eigen::Matrix4f tM(Eigen::Matrix4f::Identity());
	if(zAxis==xAxis)return tM;

	zAxis.normalize();
	xAxis.normalize();

	tM.row(3)<<0,0,0,1;
	tM.block<3,1>(0,2)=zAxis;
	tM.block<3,1>(0,1)=zAxis.cross(xAxis);
	tM.block<3,1>(0,0)=tM.block<3,1>(0,1).cross(zAxis);
	tM.block<3,1>(0,3)<<centroid.x,centroid.y,centroid.z;
	return tM;
}

pcl::PointXYZ trans(pcl::PointXYZ pt, Eigen::Vector3f vec){

	pt.x+=vec.x();
	pt.y+=vec.y();
	pt.z+=vec.z();
	return pt;
}

bool dimCheck(std::vector<float> test_dims, std::vector<float> cam_dims,float tolerance){

	if(!(test_dims.size()==3&&cam_dims.size()==3)){
		return false;
	}
	std::sort(test_dims.begin(),test_dims.end());
	std::sort(cam_dims.begin(),cam_dims.end());
	return ((cam_dims[0]>=(test_dims[0]-tolerance)&&cam_dims[0]<=(test_dims[0]+tolerance))&&
		(cam_dims[1]>=(test_dims[1]-tolerance)&&cam_dims[1]<=(test_dims[1]+tolerance))&&
		(cam_dims[2]>=(test_dims[2]-tolerance)&&cam_dims[2]<=(test_dims[2]+tolerance)));
}

Eigen::Vector3f rgbToHSL(float r, float g, float b){
	r/=255.0f;
	g/=255.0f;
	b/=255.0f;
	float max;
	int max_int;
	float min;
	float h(0.0f);
	float s(0.0f);

	if(r>=g&&r>=b){
		max_int=0;
		max=r;
		min=g>b?b:g;
	}else if(g>=b){
		max_int=1;
		max=g;
		min=r>b?b:r;
	}else{
		max_int=2;
		max=b;
		min=r>g?g:r;
	}

	float delta(max-min);

	float l((min+max)/2.0f);

	if(delta!=0.0f){
		switch(max_int){
		case 0:{
			float mod(std::fmod((g-b)/delta,6));
			h=(60.0f*(mod<0?(mod+6.0f):mod));
			break;
			   }
		case 1:{
			h=(60.0f*(2.0f+((b-r)/delta)));
			break;
			   }
		case 2:{
			h=(60.0f*(4.0f+((r-g)/delta)));
			break;
			   }
		}
		s=(delta/(1.0f-std::abs((2.0f*l)-1)));
	}

	return Eigen::Vector3f(h,s,l);
}

pxcCHAR* GestureStateToString(PXCHandData::GestureStateType label)
{
	pxcCHAR* gestureState = L"";
	switch (label)
	{
	case PXCHandData::GESTURE_STATE_START:			    {gestureState = L"GESTURE_STATE_START";break;}
	case PXCHandData::GESTURE_STATE_IN_PROGRESS:			{gestureState = L"GESTURE_STATE_IN_PROGRESS";break;}
	case PXCHandData::GESTURE_STATE_END:				    {gestureState = L"GESTURE_STATE_END";break;}        
	}
	return gestureState;
}

Eigen::Quaternionf slerpercentage(Eigen::Quaternionf quat, float percentage, float min){
	Eigen::Quaternionf nQ(quat);

	float cosHalfTheta(nQ.w());

	if(std::abs(cosHalfTheta)>=1.0f) return nQ;

	float halfTheta(std::acos(cosHalfTheta));
	float pc(percentage);

	if(halfTheta<min) pc=1.0f;

	float sinHalfTheta(std::sqrt(1.0f-(cosHalfTheta*cosHalfTheta)));

	if(std::fabs(sinHalfTheta)<0.001f){
	nQ.w()*=0.5f;
	nQ.w()+=0.5f;
	nQ.x()*=0.5f;
	nQ.y()*=0.5f;
	nQ.z()*=0.5f;
	nQ.normalize();
	return nQ;
	}

	float ratA(std::sin((1.0f-pc)*halfTheta)/sinHalfTheta);
	float ratB(std::sin(pc*halfTheta)/sinHalfTheta);
	nQ.w()=((ratA)+(nQ.w()*ratB));
	nQ.x()*=ratB;
	nQ.y()*=ratB;
	nQ.z()*=ratB;
	nQ.normalize();

	return nQ;
}

//Classes------------------------------------------------------------------------------//
struct PrimitiveColour{
	PrimitiveColour();
	int h_min;
	int h_max;
	Eigen::Vector3d rgb;
	std::string name;
	PrimitiveColour(std::string, int, int, Eigen::Vector3d);
};

PrimitiveColour::PrimitiveColour(std::string n, int min, int max, Eigen::Vector3d rgb_def){
	name=n;
	h_min=min;
	h_max=max;
	rgb=rgb_def;
};

PrimitiveColour::PrimitiveColour(){};

struct ColourLibrary{
	std::vector<PrimitiveColour> colour_library;
	void add(PrimitiveColour);
	PrimitiveColour getColour(float);
};

void ColourLibrary::add(PrimitiveColour pc){
	colour_library.push_back(pc);
};

PrimitiveColour ColourLibrary::getColour(float h){
	for (std::vector<PrimitiveColour>::iterator cit = colour_library.begin (); cit != colour_library.end (); ++cit){
		if(h<=cit->h_max && h>cit->h_min){
			return *cit;
		}
	}
	return colour_library[0];
};

struct BoxPartPlane{
	BoxPartPlane();
	BoxPartPlane(const BoxPartPlane&);
	pcl::PointXYZ centroid;
	pcl::ModelCoefficients::Ptr coeffs;
	Cloud::Ptr points;
	pcl::PointCloud<pcl::PointXYZ> flatRectangleVertices;
	pcl::PointCloud<pcl::PointXYZ> rectangleVertices;
};

BoxPartPlane::BoxPartPlane(){};

BoxPartPlane::BoxPartPlane(const BoxPartPlane& bpp){
	centroid					=bpp.centroid				;
	coeffs					=bpp.coeffs					;
	points					=bpp.points					;
	flatRectangleVertices	=bpp.flatRectangleVertices	;
	rectangleVertices		=bpp.rectangleVertices		;
};

struct Box{
	Box();
	std::string name;
	std::vector<BoxPartPlane> faces;
	pcl::PointXYZ centroid;
	std::vector<pcl::PointXYZ> verts;
	float width;
	float length;
	float height;
	Eigen::Matrix4f transformation_matrix;
	Eigen::Quaternionf base_quaternion;
	Eigen::Quaternionf quaternion;
	void calcVerts();
	void calcQuat();
	std::vector<float> dims;
	std::vector<Eigen::Vector3f> axes;
	bool valid_box;
	float hue;
	float saturation;
	float lightness;
	PrimitiveColour reference_colour;
	void movify();
};

Box::Box(){
	valid_box=false;
}

void Box::calcVerts(){

	pcl::PointXYZ a,b,c,d,e,f,g,h,faceACent;
	Eigen::Vector3f xAxis(transformation_matrix.block<3,1>(0,0));
	Eigen::Vector3f yAxis(transformation_matrix.block<3,1>(0,1));
	Eigen::Vector3f zAxis(transformation_matrix.block<3,1>(0,2));

	xAxis*=width/2.0;
	yAxis*=length/2.0;
	zAxis*=height/2.0;

	a=trans(centroid,(-xAxis-yAxis-zAxis));
	b=trans(centroid,(xAxis-yAxis-zAxis));
	c=trans(centroid,(xAxis+yAxis-zAxis));
	d=trans(centroid,(-xAxis+yAxis-zAxis));

	zAxis*=2.0;

	e=trans(a,zAxis);
	f=trans(b,zAxis);
	g=trans(c,zAxis);
	h=trans(d,zAxis);

	verts.push_back(a);
	verts.push_back(b);
	verts.push_back(c);
	verts.push_back(d);
	verts.push_back(e);
	verts.push_back(f);
	verts.push_back(g);
	verts.push_back(h);

}

void Box::calcQuat(){
	base_quaternion=Eigen::Quaternionf(transformation_matrix.block<3,3>(0,0));
}

void Box::movify(){
transformation_matrix.block<3,3>(0,0)=camToTool*transformation_matrix.block<3,3>(0,0);
transformation_matrix.block<3,1>(0,0)=transformation_matrix.block<3,1>(0,1);
transformation_matrix.block<3,1>(0,1)=transformation_matrix.block<3,1>(0,2).cross(transformation_matrix.block<3,1>(0,0));
calcQuat();
quaternion=slerpercentage(base_quaternion,percentage_move,0.1);
Eigen::Vector3f relPos(centroid.getVector3fMap());
relPos.x()-=offset.x;
relPos.y()-=offset.y;
relPos.z()-=offset.z;
if(approach){
	Eigen::Vector3f zAxis(transformation_matrix.block<3,1>(0,2));
	zAxis*=-0.15f;
	relPos.x()+=zAxis.x();
	relPos.y()+=zAxis.y();
	relPos.z()+=zAxis.z();
	if(relPos.norm()<=minimum_move){
		approach=false;
	}
}
	relPos*=1000.0f;
	relPos*=percentage_move;
	centroid.x=relPos.x();
	centroid.y=relPos.y();
	centroid.z=relPos.z();
}

struct PrimitiveBox{
	PrimitiveBox();
	PrimitiveBox(float,float,float);
	float width;
	float length;
	float height;
	std::vector<float> dims;
	bool uniqueEdges;
	pcl::PointXYZ approach;
};

PrimitiveBox::PrimitiveBox(float l, float w, float h){
	length=l;
	width=w;
	height=h;
	dims.push_back(length);
	dims.push_back(width);
	dims.push_back(height);
	uniqueEdges=(l!=w&&l!=h&&w!=h);
	approach=pcl::PointXYZ(0,0,0);
};

struct SimpleHand{
	pcl::PointXYZ centroid;
	Eigen::Vector3f normal;
	Eigen::Matrix4f transformation_matrix;
	Eigen::Quaternionf quat;
	Eigen::Quaternionf base_quaternion;
	Eigen::Quaternionf raw_quat;
	bool open;
	int openness;
	void movify();
	void calcQuat();
};

void SimpleHand::calcQuat(){
	base_quaternion=Eigen::Quaternionf(transformation_matrix.block<3,3>(0,0));
	base_quaternion.normalize();
}

void SimpleHand::movify(){
transformation_matrix.block<3,3>(0,0)=camToTool*transformation_matrix.block<3,3>(0,0);
transformation_matrix.block<3,1>(0,0)=transformation_matrix.block<3,1>(0,1);
transformation_matrix.block<3,1>(0,1)=transformation_matrix.block<3,1>(0,2).cross(transformation_matrix.block<3,1>(0,0));
calcQuat();
base_quaternion.w()=-base_quaternion.w();
quat=slerpercentage(base_quaternion,percentage_move,0.1);
Eigen::Vector3f relPos(centroid.getVector3fMap());
relPos.x()-=offset.x;
relPos.y()-=offset.y;
relPos.z()-=offset.z;
if(approach){
	normal*=-0.15f;
	relPos.x()+=normal.x();
	relPos.y()+=normal.y();
	relPos.z()+=normal.z();
	if(relPos.norm()<=minimum_move){
		approach=false;
	}
}
	relPos*=1000.0f;
	relPos*=percentage_move;
	centroid.x=relPos.x();
	centroid.y=relPos.y();
	centroid.z=relPos.z();
}

class DMRPacketListener : public osc::OscPacketListener {

protected:

	virtual void ProcessMessage( const osc::ReceivedMessage& m, 
		const IpEndpointName& remoteEndpoint )
	{
		(void) remoteEndpoint;

		try{       	
			if( std::strcmp( m.AddressPattern(), "/ping" ) == 0 ){
				osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
				oscMutex.lock();
				args >> a4 >> osc::EndMessage;
				oscMutex.unlock();
				transmit=true;
			}else if( std::strcmp( m.AddressPattern(), "/filter" ) == 0 ){
				transmit=true;
				lost_object_counter=0;
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				int argNum(0);
				oscMutex.lock();
				for (osc::ReceivedMessage::const_iterator mit = m.ArgumentsBegin(); mit != m.ArgumentsEnd(); ++mit){
					switch(argNum){
					case 0:
						search_obj=(*mit).AsString();
						filter_by_colour=false;
						approach=true;
						if(std::strcmp(search_obj.c_str(),"hand") == 0){
							hand_search=true;
						}else{
							hand_search=false;
						}
						break;
					case 1:
						colour_filter=(*mit).AsString();
						filter_by_colour=true;
						break;
					}
					argNum++;
				}
				oscMutex.unlock();
			}
		}catch( osc::Exception& e ){
			std::cout << "error while parsing message: "
				<< m.AddressPattern() << ": " << e.what() << "\n";
		}
	}
};

void DMRListener(){
	DMRPacketListener listener;
	UdpListeningReceiveSocket s(
		IpEndpointName( IpEndpointName::ANY_ADDRESS, PORTIN ),&listener );
	s.Run();
}

//SearchParameters
//3 options, 0. none, 1. single/specific list of objects, 2. all/availability search
int search_mode_(0);
std::map<std::string,PrimitiveBox> box_library;
ColourLibrary col_lib;

class RealSenseTracker
{

public:

	RealSenseTracker (pcl::RealSenseGrabber& grabber)
		: grabber_ (grabber)
		, viewer_ ("Seb's Badass Robot Eyes")
		, window_ (3)
		, threshold_ (7)
		, temporal_filtering_ (pcl::RealSenseGrabber::RealSense_None)
		, got_new_cloud_(false)
		, looper_(0)
		, objects_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
		, surface_hull_(new pcl::PointCloud<pcl::PointXYZRGBA>)
		, numHands(0)
		, out_socket(IpEndpointName(ADDRESS,PORTOUT))
	{
		viewer_.registerKeyboardCallback (&RealSenseTracker::keyboardCallback, *this);
		viewer_.setBackgroundColor(0,0,0);
		viewer_.setRepresentationToWireframeForAllActors();
	}

	~RealSenseTracker ()
	{
		connection_.disconnect ();
	}

	void
		run ()
	{
		boost::function<void (const typename Cloud::ConstPtr&)> f = boost::bind (&RealSenseTracker::cloudCallback, this, _1);
		connection_ = grabber_.registerCallback (f);
		grabber_.start ();

		//Insert hand grabbing info here
		g_session = PXCSession::CreateInstance();
		if(!g_session)
		{
			std::printf("Failed Creating PXCSession\n");
		}else{
			g_senseManager = g_session->CreateSenseManager();
			if(!g_senseManager)
			{
				std::printf("Failed Creating PXCSenseManager\n");
			}else{

				if(g_senseManager->EnableHand() != PXC_STATUS_NO_ERROR)
				{
					std::printf("Failed Enabling Hand Module\n");
				}else{
					g_handModule = g_senseManager->QueryHand();
					if(!g_handModule)
					{
						std::printf("Failed Creating PXCHandModule\n");
						return;
					}else{
						g_handDataOutput = g_handModule->CreateOutput();
						if(!g_handDataOutput)
						{
							std::printf("Failed Creating PXCHandData\n");
							return;
						}else{

							g_handConfiguration = g_handModule->CreateActiveConfiguration();
							if(!g_handConfiguration)
							{
								std::printf("Failed Creating PXCHandConfiguration\n");
								return;
							}else{
								if(g_senseManager->Init() == PXC_STATUS_NO_ERROR)
								{
									g_handConfiguration->SetTrackingMode(PXCHandData::TRACKING_MODE_FULL_HAND);
									g_handConfiguration->ApplyChanges();
									hands_available=true;
								}
							}
						}
					}
				}
			}
		}

		while (!viewer_.wasStopped ())
		{
			if (got_new_cloud_)
			{
				got_new_cloud_ = false;
				print_with_level(3,"Got New Cloud\n");
				boost::mutex::scoped_lock lock (new_cloud_mutex_);
				print_with_level(4,"Updating Point Cloud?\n");
				//if (!viewer_.updatePointCloud (new_cloud_, "cloud"))
				//{
				//	print_with_level(4,"Updating Point Cloud\n");
				//	viewer_.addPointCloud (new_cloud_, "cloud");
				//	viewer_.resetCamera ();
				//}

				//Visualisation
				last_cloud_ = new_cloud_;
				new_cloud_.reset ();
				drawFiltered(viewer_);
				//drawInliers(viewer_, inliers);
				displaySettings ();
				if(transmit){
					char buffer[OUTPUT_BUFFER_SIZE];
					osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
					if(hand_search){
						if(numHands>0){
						SimpleHand closest;
						float closest_dist(std::numeric_limits<float>::max());
						for(std::vector<SimpleHand>::iterator cit=hands.begin(); cit!=hands.end(); ++cit){
							SimpleHand h=*cit;
							float dist(h.centroid.getVector3fMap().norm());
							if(dist<closest_dist){
								closest=h;
								closest_dist=dist;
							}
						}
						obj_distance=closest_dist;
						closest_obj_centroid=closest.centroid;
						closest.movify();
						if(!closest.open){
							closest_dist+=100;
						}else if(!approach && closest_dist<0.07){
							closest_dist-=0.05;
						}

						p << osc::BeginBundleImmediate
						<< osc::BeginMessage( "/io" ) 
						<< "objdist" << (float)closest_dist*1000.0f << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objx" << (float)closest.centroid.x << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objy" << (float)closest.centroid.y << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objz" << (float)closest.centroid.z << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq1" << (float)closest.quat.w() << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq2" << (float)closest.quat.x() << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq3" << (float)closest.quat.y() << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq4" << (float)closest.quat.z() << osc::EndMessage
						<< osc::EndBundle;
					out_socket.Send( p.Data(), p.Size() );
						}else{
							lost_object_counter++;
						}
					}else if(!(std::strcmp(search_obj.c_str(),"")==0)){
						if(validBoxes.size()>0){
						Box closest;
						float closest_dist(std::numeric_limits<float>::max());
						for(std::vector<Box>::iterator cit=validBoxes.begin(); cit!=validBoxes.end(); ++cit){
							Box b=*cit;
							float dist(b.centroid.getVector3fMap().norm()*1000.0f);
							if(dist<closest_dist){
								closest=b;
								closest_dist=dist;
							}
						}
						obj_distance=closest_dist;
						closest_obj_centroid=closest.centroid;
						closest.movify();
						p << osc::BeginBundleImmediate
						<< osc::BeginMessage( "/io" ) 
						<< "objdist" << (float)closest_dist << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objx" << (float)closest.centroid.x << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objy" << (float)closest.centroid.y << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objz" << (float)closest.centroid.z << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq1" << (float)closest.quaternion.w() << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq2" << (float)closest.quaternion.x() << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq3" << (float)closest.quaternion.y() << osc::EndMessage
						<< osc::BeginMessage( "/io" ) 
						<< "objq4" << (float)closest.quaternion.z() << osc::EndMessage
						<< osc::EndBundle;
						out_socket.Send( p.Data(), p.Size() );
						}else{
							lost_object_counter++;
						}
					}
					
					//Create new message/bundle with distance from coefficients' closest point to targetpoint (offset)
					char b[OUTPUT_BUFFER_SIZE];
					osc::OutboundPacketStream d( b, OUTPUT_BUFFER_SIZE );
					float dist(0);
					if(plane_vision_){
						dist=planeDist(*coefficients,offset);
					}
					plane_distance=dist;
						d << osc::BeginBundleImmediate
						<< osc::BeginMessage( "/io" ) 
						<< "distance" << (float)dist*1000.0f << osc::EndMessage
						<< osc::EndBundle;
						out_socket.Send(d.Data(), d.Size());
					
					cout << "Sending OSC Messages" << endl;
				}

				print_with_level(3,"Loop End\n");
			}
			viewer_.spinOnce (1, true);
		}
		print_with_level(2,"It's all over\n");
		grabber_.stop ();
	}

private:

	void
		cloudCallback (const Cloud::ConstPtr cloud)
	{
		if (!viewer_.wasStopped () && !got_new_cloud_)
		{
			print_with_level(2,"Starting Callback\n");
			boost::mutex::scoped_lock lock (new_cloud_mutex_);
			{
				print_with_level(4,"Mutexed\n");

				//Downsampling cloud
				print_with_level(4,"Resetting all my clouds\n");
				cloud_pass_.reset(new Cloud);
				cloud_pass_downsampled_.reset(new Cloud);
				print_with_level(4,"Just PassingThrough\n");
				clock_t filterStart = clock();
				filterPassThrough(cloud, *cloud_pass_);
				std::vector<int> indices;
				pcl::removeNaNFromPointCloud(*cloud_pass_,*cloud_pass_, indices);
				print_with_level(4,"Approximating the Sample of my Grid\n");
				gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
				printf("Filter Processing Time taken: %.3fs\n", (double)(clock() - filterStart) / CLOCKS_PER_SEC);
				new_cloud_ = cloud_pass_;

				//Fully clear all lists of objects
				boxes.clear();
				validBoxes.clear();
				hands.clear();
				subBoxes.clear();
				objects_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
				closest_obj_centroid=pcl::PointXYZ(0,0,0);
				obj_distance=0.0f;
				plane_distance=0.0f;

				if(hand_search){

					if(g_senseManager->AcquireFrame(true) == PXC_STATUS_NO_ERROR)
					{
						// Get current hand outputs
						if(g_handDataOutput->Update() == PXC_STATUS_NO_ERROR)
						{
							// Display gestures
							PXCHandData::GestureData gestureData;
							numHands=g_handDataOutput->QueryNumberOfHands();
							for(int i=0; i<numHands; ++i){

								PXCHandData::IHand* hand = NULL;
								if(g_handDataOutput->QueryHandData(PXCHandData::ACCESS_ORDER_NEAR_TO_FAR,i,hand) == PXC_STATUS_NO_ERROR)
								{
									SimpleHand h;
									PXCPoint3DF32 centre = hand->QueryMassCenterWorld();
									h.centroid=pcl::PointXYZ(centre.x,centre.y,centre.z);

									PXCPoint4DF32 palm_orientation=hand->QueryPalmOrientation();
									Eigen::Quaternionf palm_up(palm_orientation.w,palm_orientation.x,palm_orientation.y,palm_orientation.z);
									h.raw_quat=palm_up;
									Eigen::Matrix3f hand_rot_mat=palm_up.toRotationMatrix();
									h.normal=hand_rot_mat.block<3,1>(0,2);
									h.normal*=sgn(cosineSimilarity(h.normal,Eigen::Vector3f(0,0,1)));
									h.transformation_matrix=boxTrans(h.normal,Eigen::Vector3f(1.0f,0.0f,0.0f),h.centroid);

									h.calcQuat();

									h.openness=hand->QueryOpenness();

									h.open=(h.openness>=70);

									hands.push_back(h);
								}
							}
						}
						g_senseManager->ReleaseFrame();
					}
				}else{
					//Plane Based Segmentation
					clock_t segmentationStart = clock();
					//while segmentedcloud is still bigger than 0.5m2???PCL Extracting indices example
					coefficients.reset (new pcl::ModelCoefficients);
					inliers.reset(new pcl::PointIndices);
					// Create the segmentation object
					pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
					seg.setOptimizeCoefficients (true);
					seg.setModelType (pcl::SACMODEL_PLANE);
					seg.setMethodType (pcl::SAC_RANSAC);
					seg.setDistanceThreshold (plane_threshold);
					seg.setInputCloud (cloud_pass_downsampled_);
					seg.segment (*inliers, *coefficients);

					filtered_cloud_.reset(new Cloud);
					pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
					extract.setInputCloud (cloud_pass_downsampled_);
					extract.setIndices (inliers);
					extract.setNegative (true);
					extract.filter (*filtered_cloud_);
					printf("Segmentation Processing Time taken: %.3fs\n", (double)(clock() - segmentationStart) / CLOCKS_PER_SEC);
					//End while

					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projected_plane_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);

					//Extract Plane
					extract.setNegative (false);
					extract.filter (*plane_cloud_);
					print_with_level(2,"Plane Extracted\n");

					clock_t plane_clustering = clock();
					//Extract the main surface
					pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr plane_tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
					plane_tree->setInputCloud (plane_cloud_);
					std::vector<pcl::PointIndices> plane_indices;
					pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> pec;
					pec.setClusterTolerance (downsampling_grid_size_*1.5);
					pec.setMinClusterSize (50);
					pec.setMaxClusterSize (100000);
					pec.setSearchMethod (plane_tree);
					pec.setInputCloud (plane_cloud_);
					pec.extract (plane_indices);
					printf("Found %i clusters in plane model, Processing Time taken: %.3fs\n",plane_indices.size(), (double)(clock() - plane_clustering) / CLOCKS_PER_SEC);

					if(plane_indices.size()>0){
						print_with_level(4,"Biggest Plane Setup\n");
						std::sort(plane_indices.begin(),plane_indices.end(),point_indices_descending_size_sort());
						pcl::PointCloud<pcl::PointXYZRGBA>::Ptr biggest_plane_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
						pcl::PointIndices biggest_plane_indices_=plane_indices[0];
						inliers->indices=biggest_plane_indices_.indices;
						print_with_level(4,"Found Biggest Plane\n");
						for (std::vector<int>::const_iterator pit = biggest_plane_indices_.indices.begin (); pit != biggest_plane_indices_.indices.end (); ++pit)
							biggest_plane_->points.push_back (plane_cloud_->points[*pit]);
						plane_cloud_=biggest_plane_;
						print_with_level(4,"Biggest Plane Created\n");
					}else{
						print_with_level(4,"Plane was already whole\n");
					}

					//Check for plane system
					plane_vision_=plane_cloud_->points.size()>0;

					//Project Plane Points to plane
					pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
					proj.setModelType (pcl::SACMODEL_PLANE);
					proj.setInputCloud (plane_cloud_);
					proj.setModelCoefficients (coefficients);
					proj.filter (*projected_plane_cloud_);
					print_with_level(4,"Plane Projected\n");

					//Planar Hull
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr base_cloud_hull_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
					pcl::ConvexHull<pcl::PointXYZRGBA> chull;
					chull.setInputCloud (projected_plane_cloud_);
					chull.setDimension(2);
					chull.reconstruct (*base_cloud_hull_);
					print_with_level(4,"Hulled\n");
					surface_hull_=base_cloud_hull_;

					//Euclidean clustering of non-planar points
					clock_t clustering = clock();
					print_with_level(4,"Starting Clustering\n");
					pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
					tree->setInputCloud (filtered_cloud_);
					std::vector<pcl::PointIndices> cluster_indices;

					pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
					ec.setClusterTolerance (downsampling_grid_size_);
					ec.setMinClusterSize (100);
					ec.setMaxClusterSize (25000);
					ec.setSearchMethod (tree);
					ec.setInputCloud (filtered_cloud_);
					ec.extract (cluster_indices);
					printf("Found %i clusters, Processing Time taken: %.3fs\n",cluster_indices.size(), (double)(clock() - clustering) / CLOCKS_PER_SEC);

					//Cluster Centroid Projection and Cluster Selection
					std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cluster_clouds_;

					{
						int j=0;
						for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
						{
							clock_t validation = clock();
							pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster_cloud_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
							for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
								cluster_cloud_->points.push_back (filtered_cloud_->points[*pit]);
							printf("Cluster Cloud has %i points\n",cluster_cloud_->points.size());

							if(plane_vision_){
								//Calculate Cluster Centroid
								Eigen::Vector4f cluster_centroid_;
								pcl::compute3DCentroid<pcl::PointXYZRGBA>(*cluster_cloud_, cluster_centroid_);
								pcl::PointXYZRGBA centroid_;
								centroid_.x=cluster_centroid_[0];
								centroid_.y=cluster_centroid_[1];
								centroid_.z=cluster_centroid_[2];


								//Add centroid to existing hull points
								pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projection_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
								pcl::copyPointCloud(*projected_plane_cloud_,*projection_cloud_);
								projection_cloud_->push_back(centroid_);

								//printf("Projection Cloud has %i points\n",projection_cloud_->points.size());
								pcl::PointCloud<pcl::PointXYZRGBA>::Ptr test_hull_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
								proj.setInputCloud (projection_cloud_);
								proj.filter (*test_hull_);
								//printf("Projected Cloud has %i points\n",test_hull_->points.size());

								chull.setInputCloud(test_hull_);
								chull.reconstruct(*test_hull_);
								//printf("Test Hull Cloud has %i points\n",test_hull_->points.size());

								if(test_hull_->points.size()==base_cloud_hull_->points.size()){
									cluster_clouds_.push_back(cluster_cloud_);
									*objects_cloud_+=*cluster_cloud_;
								}

							}else{
								cluster_clouds_.push_back(cluster_cloud_);
							}

							printf("Cluster %i validation time taken: %.3fs\n",j, (double)(clock() - validation) / CLOCKS_PER_SEC);

							j++;
						}
					}

					//Search mode switch

					switch(search_mode_){
					case 0:{
						print_with_level(4,"Searching for nothing\n");
						break;
						   }

					case 1:
					case 2:
						{
							print_with_level(4,"Searching for something\n");
							//Algorithm switch
							switch(algorithm_){
							case 4:{
								print_with_level(4,"Planar Decomposition\n");
								//extract all possible planes from the cluster until the size is below a certain threshold
								for (int alpha=0; alpha<cluster_clouds_.size();++alpha){

									Box box;
									box.name="";
									printf("Cluster entry %i\n",alpha);
									Cloud::Ptr cld = cluster_clouds_[alpha];
									Cloud::Ptr used_cloud(new Cloud());
									print_with_level(4,"Cloud transfered\n");
									bool goodPlanes(true);
									int good_iterations(0);
									while(goodPlanes){

										print_with_level(4,"In while\n");
										if(cld->points.size()>min_plane_points){
											BoxPartPlane subBox;
											print_with_level(4,"Good plane accepted\n");
											pcl::PointIndices::Ptr subPlaneInliers(new pcl::PointIndices);
											subBox.coeffs.reset(new pcl::ModelCoefficients);
											print_with_level(4,"All reset\n");

											// Create the segmentation object
											pcl::SACSegmentation<RefPointType> subseg;
											subseg.setOptimizeCoefficients (true);
											subseg.setModelType (pcl::SACMODEL_PLANE);
											subseg.setMethodType (pcl::SAC_RANSAC);
											subseg.setDistanceThreshold (model_plane_threshold);
											subseg.setInputCloud (cld);
											subseg.segment (*subPlaneInliers, *subBox.coeffs);
											print_with_level(4,"Plane extracted\n");
											if(subPlaneInliers->indices.size()>min_plane_points){
												print_with_level(4,"Good plane\n");
												good_iterations++;
												subBox.points.reset(new Cloud);
												pcl::ExtractIndices<pcl::PointXYZRGBA> subextract;
												subextract.setInputCloud (cld);
												subextract.setIndices (subPlaneInliers);
												subextract.setNegative (false);
												subextract.filter (*subBox.points);
												subextract.setNegative(true);
												subextract.filter(*cld);

												*used_cloud+=*subBox.points;
												//Filter outliers from possible euclidean clustering?

												print_with_level(4,"Full extraction\n");
												Eigen::Vector4f face_centroid_;
												pcl::compute3DCentroid<pcl::PointXYZRGBA>(*subBox.points, face_centroid_);
												subBox.centroid.x=face_centroid_[0];
												subBox.centroid.y=face_centroid_[1];
												subBox.centroid.z=face_centroid_[2];
												print_with_level(4,"Centroid computed\n");
												box.faces.push_back(subBox);
												subBoxes.push_back(subBox);
												print_with_level(4,"Plane Stored\n");
											}else{
												goodPlanes=false;
												print_with_level(4,"Good planes over A\n");
											}
										}else{
											print_with_level(4,"Good planes over B\n");
											goodPlanes=false;
										}
										//End of good planes while
									}

									print_with_level(4,"Preparing for planar decomposition\n");
									cout<< "Good iters: " << good_iterations << ", Box num Faces: " << box.faces.size() <<endl;
									if(good_iterations>0){

										int originalNumSides(box.faces.size());
										bool single_face(false);

										//Generic box setup parameters
										int parallel_to_base_plane(-1);
										box.dims.clear();
										box.axes.clear();
										box.centroid.x=0;
										box.centroid.y=0;
										box.centroid.z=0;
										box.valid_box=false;

										switch(originalNumSides){

										case 1: {
											print_with_level(2,"True Single plane\n");
											single_face=true;
											break;
												}

										case 2:{
											print_with_level(2,"Double plane\n");
											if(plane_vision_){
												if(std::abs(cosineSimilarity(coefficients,box.faces[0].coeffs))>parallel_threshold){
													single_face=true;
													break;
												}
											}
											double angular_tolerance=0.1;
											Eigen::Vector4f plane_a(box.faces[0].coeffs->values[0],box.faces[0].coeffs->values[1],box.faces[0].coeffs->values[2],box.faces[0].coeffs->values[3]);
											Eigen::Vector4f plane_b(box.faces[1].coeffs->values[0],box.faces[1].coeffs->values[1],box.faces[1].coeffs->values[2],box.faces[1].coeffs->values[3]);
											Eigen::VectorXf line;

											if(pcl::planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance)){

												Eigen::Vector3f f0_normal;
												Eigen::Vector3f f1_normal;
												float thirdDim(0.0f);
												Eigen::Vector3f f2_normal(line[3],line[4],line[5]);

												float r(0);
												float g(0);
												float b(0);
												float totNumPts(0);

												for(int i=0;i<originalNumSides;++i){
													BoxPartPlane nBPP(box.faces[i]);
													totNumPts+=float(box.faces[i].points->points.size());

													std::vector<float> widthvals;
													for(int j=0;j<box.faces[i].points->points.size();++j){
														Eigen::Vector3f pt(
															box.faces[i].points->points[j].x,
															box.faces[i].points->points[j].y,
															box.faces[i].points->points[j].z);
														widthvals.push_back(float(pt.dot(f2_normal)));

														r+=box.faces[i].points->points[j].r;
														g+=box.faces[i].points->points[j].g;
														b+=box.faces[i].points->points[j].b;

													}

													std::sort(widthvals.begin(),widthvals.end());
													thirdDim+=widthvals.back()-widthvals.front();

													bool par_plane(false);
													if(std::abs(cosineSimilarity(coefficients,box.faces[i].coeffs))>parallel_threshold){
														par_plane=true;
														parallel_to_base_plane=i;
													}

													pcl::PointXYZ reflectionCentre(0,0,0);
													int other;
													Eigen::Vector3f normal(box.faces[i].coeffs->values[0],box.faces[i].coeffs->values[1],box.faces[i].coeffs->values[2]);
													switch(i){
													case 0:
														other=1;
														f0_normal=normal;
														break;
													case 1:
														other=0;
														f1_normal=normal;
														break;
													}

													Eigen::Vector3f thisCentroid(box.faces[i].centroid.x,box.faces[i].centroid.y,box.faces[i].centroid.z);
													Eigen::Vector3f otherCentroid(box.faces[other].centroid.x,box.faces[other].centroid.y,box.faces[other].centroid.z);
													float thisDist(thisCentroid.dot(normal));
													float otherDist(otherCentroid.dot(normal));
													float signedDist(thisDist-otherDist);

													if(par_plane && 4.0f*std::abs(signedDist)>planeDist(*coefficients,box.faces[i].centroid)){
														if(signedDist<0){
															signedDist-=(plane_threshold*0.5f);
														}else{
															signedDist+=(plane_threshold*0.5f);
														}
													}
													cout<<"Signed dist: "<<signedDist<<endl;
													Eigen::Vector3f nCentroid(box.faces[i].centroid.x,box.faces[i].centroid.y,box.faces[i].centroid.z);
													nCentroid+=((2*signedDist)*normal);
													nBPP.centroid.x=nCentroid.x();
													nBPP.centroid.y=nCentroid.y();
													nBPP.centroid.z=nCentroid.z();

													//Box centroid
													box.centroid.x+=box.faces[i].centroid.x;
													box.centroid.y+=box.faces[i].centroid.y;
													box.centroid.z+=box.faces[i].centroid.z;
													box.centroid.x+=nBPP.centroid.x;
													box.centroid.y+=nBPP.centroid.y;
													box.centroid.z+=nBPP.centroid.z;

													//Lib comparison additions
													box.dims.push_back(2*std::abs(signedDist));
													box.axes.push_back(normal);
												}
												cout << "Cluster PreDiv RGB: " << r << ", " << g << ", " << b << endl;
												r/=totNumPts;
												g/=totNumPts;
												b/=totNumPts;
												cout << "Cluster RGB: " << r << ", " << g << ", " << b << endl;
												cout << "Tot RGB Pts: " << totNumPts << endl;

												Eigen::Vector3f boxHSL(rgbToHSL((r),(g),(b)));
												box.hue=boxHSL.x();
												box.saturation=boxHSL.y();
												box.lightness=boxHSL.z();
												box.reference_colour=col_lib.getColour(box.hue);

												box.centroid.x/=4.0;
												box.centroid.y/=4.0;
												box.centroid.z/=4.0;

												box.dims.push_back(0.5f*std::abs(thirdDim));
												box.axes.push_back(f2_normal);
												box.valid_box=true;
											}

											break;
											   }

										case 3:
										case 4:
										case 5:
										case 6:
										case 7:
										case 8:{

											print_with_level(2,"Multi plane\n");
											if(plane_vision_){
												if(std::abs(cosineSimilarity(coefficients,box.faces[0].coeffs))>parallel_threshold){
													single_face=true;
													break;
												}
											}

											float r(0);
											float g(0);
											float b(0);
											float totNumPts(0);

											for(int i=0;i<3;++i){

												totNumPts+=float(box.faces[i].points->points.size());
												for(int j=0;j<box.faces[i].points->points.size();++j){
													r+=box.faces[i].points->points[j].r;
													g+=box.faces[i].points->points[j].g;
													b+=box.faces[i].points->points[j].b;
												}
												bool par_plane(false);
												if(std::abs(cosineSimilarity(coefficients,box.faces[i].coeffs))>parallel_threshold){
													par_plane=true;
													parallel_to_base_plane=i;
												}
												BoxPartPlane nBPP(box.faces[i]);
												pcl::PointXYZ reflectionCentre(0,0,0);
												switch(i){
												case 0:
													reflectionCentre.x=(box.faces[1].centroid.x+box.faces[2].centroid.x)/2;
													reflectionCentre.y=(box.faces[1].centroid.y+box.faces[2].centroid.y)/2;
													reflectionCentre.z=(box.faces[1].centroid.z+box.faces[2].centroid.z)/2;
													break;
												case 1:
													reflectionCentre.x=(box.faces[0].centroid.x+box.faces[2].centroid.x)/2;
													reflectionCentre.y=(box.faces[0].centroid.y+box.faces[2].centroid.y)/2;
													reflectionCentre.z=(box.faces[0].centroid.z+box.faces[2].centroid.z)/2;
													break;
												case 2:
													reflectionCentre.x=(box.faces[1].centroid.x+box.faces[0].centroid.x)/2;
													reflectionCentre.y=(box.faces[1].centroid.y+box.faces[0].centroid.y)/2;
													reflectionCentre.z=(box.faces[1].centroid.z+box.faces[0].centroid.z)/2;
													break;
												}

												float signedDist(signedPlaneDist(*(box.faces[i].coeffs),reflectionCentre));
												if(par_plane && 2.0f*std::abs(signedDist)>planeDist(*coefficients,reflectionCentre)){
													if(signedDist<0){
														signedDist-=(plane_threshold*0.5f);
													}else{
														signedDist+=(plane_threshold*0.5f);
													}
												}
												cout<<"Signed dist: "<<signedDist<<endl;
												Eigen::Vector3f nCentroid(box.faces[i].centroid.x,box.faces[i].centroid.y,box.faces[i].centroid.z);
												Eigen::Vector3f planeNorm(box.faces[i].coeffs->values[0],box.faces[i].coeffs->values[1],box.faces[i].coeffs->values[2]);
												nCentroid+=((2*signedDist)*planeNorm);
												nBPP.centroid.x=nCentroid.x();
												nBPP.centroid.y=nCentroid.y();
												nBPP.centroid.z=nCentroid.z();

												//Box centroid
												box.centroid.x+=box.faces[i].centroid.x;
												box.centroid.y+=box.faces[i].centroid.y;
												box.centroid.z+=box.faces[i].centroid.z;
												box.centroid.x+=nBPP.centroid.x;
												box.centroid.y+=nBPP.centroid.y;
												box.centroid.z+=nBPP.centroid.z;

												//Lib comparison additions
												box.dims.push_back(2*std::abs(signedDist));
												box.axes.push_back(planeNorm);

												//Storage
												box.faces.push_back(nBPP);
											}
											r/=totNumPts;
											g/=totNumPts;
											b/=totNumPts;

											Eigen::Vector3f boxHSL(rgbToHSL((r),(g),(b)));
											box.hue=boxHSL.x();
											box.saturation=boxHSL.y();
											box.lightness=boxHSL.z();
											box.reference_colour=col_lib.getColour(box.hue);

											box.centroid.x/=6.0;
											box.centroid.y/=6.0;
											box.centroid.z/=6.0;
											box.valid_box=true;
											break;
											   }
											   //End of box generation switch
										}

										//Single face run
										if(single_face && !box.valid_box){
											print_with_level(2,"Single face\n");
											BoxPartPlane nBPP=box.faces[0];
											box.faces.clear();
											box.faces.push_back(nBPP);
											Eigen::Vector3f zAxis(nBPP.coeffs->values[0],nBPP.coeffs->values[1],nBPP.coeffs->values[2]);
											print_with_level(4,"Got Box temp Z axis\n");

											float r(0);
											float g(0);
											float b(0);
											float totNumPts(nBPP.points->points.size());

											for(int j=0;j<nBPP.points->points.size();++j){
												r+=nBPP.points->points[j].r;
												g+=nBPP.points->points[j].g;
												b+=nBPP.points->points[j].b;
											}

											r/=totNumPts;
											g/=totNumPts;
											b/=totNumPts;
											Eigen::Vector3f boxHSL(rgbToHSL((r),(g),(b)));
											box.hue=boxHSL.x();
											box.saturation=boxHSL.y();
											box.lightness=boxHSL.z();
											box.reference_colour=col_lib.getColour(box.hue);

											//Project all points to plane
											pcl::PointCloud<pcl::PointXYZRGBA>::Ptr bpp_projected (new pcl::PointCloud<pcl::PointXYZRGBA>);
											pcl::ProjectInliers<pcl::PointXYZRGBA> bppproj;
											bppproj.setModelType (pcl::SACMODEL_PLANE);
											bppproj.setInputCloud (nBPP.points);
											bppproj.setModelCoefficients (nBPP.coeffs);
											bppproj.filter (*bpp_projected);
											print_with_level(4,"BPP Plane Projected\n");

											//Calculate convex hull
											pcl::PointCloud<pcl::PointXYZRGBA>::Ptr bpp_hull_pts (new pcl::PointCloud<pcl::PointXYZRGBA>);
											pcl::ConvexHull<pcl::PointXYZRGBA> bpp_hull;
											bpp_hull.setInputCloud (bpp_projected);
											bpp_hull.setDimension(2);
											bpp_hull.reconstruct (*bpp_hull_pts);
											box.faces[0].points=bpp_hull_pts;
											print_with_level(4,"BPP Hulled\n");

											//Edges
											std::vector<Eigen::Vector3f> edges;

											for(int i=0;i<bpp_hull_pts->points.size();++i){
												if(i==bpp_hull_pts->points.size()-1){
													Eigen::Vector3f edge(
														bpp_hull_pts->points[0].x-bpp_hull_pts->points[i].x,
														bpp_hull_pts->points[0].y-bpp_hull_pts->points[i].y,
														bpp_hull_pts->points[0].z-bpp_hull_pts->points[i].z);
													edge.normalize();
													edges.push_back(edge);
												}else{
													Eigen::Vector3f edge(
														bpp_hull_pts->points[i+1].x-bpp_hull_pts->points[i].x,
														bpp_hull_pts->points[i+1].y-bpp_hull_pts->points[i].y,
														bpp_hull_pts->points[i+1].z-bpp_hull_pts->points[i].z);
													edge.normalize();
													edges.push_back(edge);
												}
											}
											print_with_level(4,"Edges initialised\n");

											Eigen::Vector3f xAxis(0,0,0);
											Eigen::Vector3f yAxis(0,0,0);
											float min_area(std::numeric_limits<float>::max());
											float xDim(0);
											float yDim(0);

											cout<<"PreHull Option Dims: "<< xDim << ", " << yDim << ", " << min_area <<endl;
											for(int i=0; i<edges.size(); ++i){
												Eigen::Vector3f xAxis_temp=edges[i];
												Eigen::Vector3f yAxis_temp=xAxis_temp.cross(zAxis);
												std::vector<float> xvals;
												std::vector<float> yvals;
												float xDif;
												float yDif;
												float area;
												for(int j=0;j<bpp_hull_pts->points.size();++j){
													Eigen::Vector3f pt(
														bpp_hull_pts->points[j].x,
														bpp_hull_pts->points[j].y,
														bpp_hull_pts->points[j].z);
													xvals.push_back(float(pt.dot(xAxis_temp)));
													yvals.push_back(float(pt.dot(yAxis_temp)));
												}
												std::sort(xvals.begin(),xvals.end());
												std::sort(yvals.begin(),yvals.end());
												xDif=xvals.back()-xvals.front();
												yDif=yvals.back()-yvals.front();
												area=(xDif*1000)*(yDif*1000);
												if(area<min_area){
													min_area=area;
													xAxis=xAxis_temp;
													yAxis=yAxis_temp;
													xDim=xDif;
													yDim=yDif;
												}
											}

											print_with_level(4,"BPP Edged\n");

											box.axes.push_back(xAxis);
											box.axes.push_back(yAxis);
											box.axes.push_back(zAxis);
											box.dims.push_back(xDim);
											box.dims.push_back(yDim);

											//Jiggerypokery to find the zDim
											if(plane_vision_){
												float similarity(cosineSimilarity(coefficients,nBPP.coeffs));
												if(std::abs(similarity)>parallel_threshold){
													float simSign(similarity<0?-1.0f:1.0f);
													float planeDist(signedPlaneDist(*coefficients,nBPP.centroid)*simSign);
													box.dims.push_back(std::abs(planeDist));
													float halfDist(planeDist/2.0f);
													box.centroid.x=nBPP.centroid.x-(halfDist*zAxis.x());
													box.centroid.y=nBPP.centroid.y-(halfDist*zAxis.y());
													box.centroid.z=nBPP.centroid.z-(halfDist*zAxis.z());
												}else{
													box.dims.push_back(plane_threshold);
													box.centroid.x=nBPP.centroid.x;
													box.centroid.y=nBPP.centroid.y;
													box.centroid.z=nBPP.centroid.z;
												}
											}else{
												box.dims.push_back(plane_threshold);
												box.centroid.x=nBPP.centroid.x;
												box.centroid.y=nBPP.centroid.y;
												box.centroid.z=nBPP.centroid.z;
											}
											box.valid_box=true;
										}

										//Test box against library
										if(box.valid_box){
											bool found_in_lib(false);
											for(std::map<std::string,PrimitiveBox>::iterator blit=box_library.begin(); blit!=box_library.end(); ++blit){
												if(dimCheck(blit->second.dims,box.dims,object_dim_tolerance)){
													found_in_lib=true;
													box.name=blit->first;
													box.length=blit->second.length;
													box.width=blit->second.width;
													box.height=blit->second.height;

													if(blit->second.uniqueEdges){
														//Get which original values were used for which dim
														std::vector<float> sortDims(box.dims);
														std::vector<float> sortPrimDims(blit->second.dims);
														std::sort(sortDims.begin(),sortDims.end());
														std::sort(sortPrimDims.begin(),sortPrimDims.end());

														int lIndex(std::find(box.dims.begin(), box.dims.end(), sortDims[std::find(sortPrimDims.begin(), sortPrimDims.end(), box.length) - sortPrimDims.begin()]) - box.dims.begin());
														int wIndex(std::find(box.dims.begin(), box.dims.end(), sortDims[std::find(sortPrimDims.begin(), sortPrimDims.end(), box. width) - sortPrimDims.begin()]) - box.dims.begin());
														int hIndex(std::find(box.dims.begin(), box.dims.end(), sortDims[std::find(sortPrimDims.begin(), sortPrimDims.end(), box.height) - sortPrimDims.begin()]) - box.dims.begin());

														//sort axes according to lengths
														Eigen::Vector3f xaxis(box.axes[wIndex]);
														Eigen::Vector3f yaxis(box.axes[lIndex]);
														Eigen::Vector3f zaxis(box.axes[hIndex]);

														cout << "Base XAxis: " << xaxis <<endl;
														cout << "Base YAxis: " << yaxis <<endl;
														cout << "Base ZAxis: " << zaxis <<endl;
														cout << "Length: " << box.dims[lIndex] <<endl;
														cout << "Width: " << box.dims[wIndex] <<endl;
														cout << "Height: " << box.dims[hIndex] <<endl;

														zaxis*=sgn(cosineSimilarity(zaxis,Eigen::Vector3f(0,0,1)));
														xaxis*=sgn(cosineSimilarity(xaxis,Eigen::Vector3f(1,0,0)));

														box.transformation_matrix=boxTrans(zaxis,xaxis,box.centroid);
														cout << box.transformation_matrix << endl;
														box.calcVerts();
														box.faces.clear();

													}else{

													}

													bool filtered(false);
													if(std::strcmp(box.name.c_str(),search_obj.c_str())==0){
														if(filter_by_colour){
															if(std::strcmp(box.reference_colour.name.c_str(),colour_filter.c_str())==0){
																validBoxes.push_back(box);
																filtered=true;
															}
														}else{
															validBoxes.push_back(box);
															filtered=true;
														}
													}
													if(!filtered){
														boxes.push_back(box);
													}
													break;
												}
											}
										}
									}
									//End of for all clusters								
								}
								break;
								//end of algorithm switch case 4
								   }
							default:{}
									//End of algorithm switch
							}
							break;
							//End of Search switch cases 1/2
						}
						//End of search switch
					}
				}
			}

			if(export_objects){
			std::ofstream out((boost::format("%i_%s")%out_file_num%coef_point_filename_).str());
				++out_file_num;

				if(plane_vision_){
					out << "Plane:\n" << "Coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
				}else{
					out << "Plane:\n" << std::endl;
				}
				if(hand_search){
					for(int i=0; i<hands.size();++i){
					out << "Hand: " << std::endl;
					hands[i].movify();
					out	<< "Centroid: " << hands[i].centroid.x << ", " << hands[i].centroid.y << "," << hands[i].centroid.z << std::endl;
					out << "Normal: " << hands[i].normal.x() << ", " << hands[i].normal.y() << "," << hands[i].normal.z() << std::endl;
					out << "RawQuat: " << hands[i].raw_quat.w() << ", " << hands[i].raw_quat.x() << ", " << hands[i].raw_quat.y() << ", " << hands[i].raw_quat.z() << std::endl;
					out << "Quaternion: " << hands[i].base_quaternion.w() << ", " << hands[i].base_quaternion.x() << ", " << hands[i].base_quaternion.y() << ", " << hands[i].base_quaternion.z() << std::endl;
					out << "PartialQuat: " << hands[i].quat.w() << ", " << hands[i].quat.x() << ", " << hands[i].quat.y() << ", " << hands[i].quat.z() << std::endl;
				}
				}else{
				out << "Valid Boxes: " << validBoxes.size() << std::endl;
				for(int i=0; i<validBoxes.size();++i){
					Box tempBox(validBoxes[i]);
					out << "Box: " << validBoxes[i].name << std::endl;
					out	<< "Centroid: " << tempBox.centroid.x*1000.0f << ", " << tempBox.centroid.y*1000.0f << ", " << tempBox.centroid.z*1000.0f << std::endl;
					tempBox.movify();
					out << "Transformatrix: " << tempBox.transformation_matrix << std::endl;
					out << "Quaternion: " << tempBox.base_quaternion.w() << ", " << tempBox.base_quaternion.x() << ", " << tempBox.base_quaternion.y() << ", " << tempBox.base_quaternion.z() << std::endl;
					out << "Colour: " << tempBox.reference_colour.name << std::endl;
					out	<< "PartialCent: " << tempBox.centroid.x << ", " << tempBox.centroid.y << ", " << tempBox.centroid.z << std::endl;
					out << "PartialQuat: " << tempBox.quaternion.w() << ", " << tempBox.quaternion.x() << ", " << tempBox.quaternion.y() << ", " << tempBox.quaternion.z() << std::endl;
				}
				for(int i=0; i<boxes.size();++i){
					out << "Box: " << boxes[i].name << std::endl;
					out	<< "Centroid: " << boxes[i].centroid.x*1000.0f << ", " << boxes[i].centroid.y*1000.0f << ", " << boxes[i].centroid.z*1000.0f << std::endl;
					boxes[i].movify();
					out << "Transformatrix: " << boxes[i].transformation_matrix << std::endl;
					out << "Quaternion: " << boxes[i].base_quaternion.w() << ", " << boxes[i].base_quaternion.x() << ", " << boxes[i].base_quaternion.y() << ", " << boxes[i].base_quaternion.z() << std::endl;
					out << "Colour: " << boxes[i].reference_colour.name << std::endl;
					out	<< "PartialCent: " << boxes[i].centroid.x << ", " << boxes[i].centroid.y << ", " << boxes[i].centroid.z << std::endl;
					out << "PartialQuat: " << boxes[i].quaternion.w() << ", " << boxes[i].quaternion.x() << ", " << boxes[i].quaternion.y() << ", " << boxes[i].quaternion.z() << std::endl;
				}
				}
				out.close();
				export_objects=false;
			
			}

			got_new_cloud_ = true;
			print_with_level(3,"Done With Callback\n");
		}
	}

	void
		keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.keyDown ())
		{
			if (event.getKeyCode () == 's' || event.getKeyCode () == 'S')
			{
				tracking_=!tracking_;
				std::ofstream out(coef_point_filename_);
				out.close();
				if(!tracking_){
					//reset all clouds in the scan game
				}
			}
			if (event.getKeyCode () == 't' || event.getKeyCode () == 'T')
			{
				threshold_ += event.getKeyCode () == 't' ? 1 : -1;
				if (threshold_ < 0)
					threshold_ = 0;
				if (threshold_ > 15)
					threshold_ = 15;
				grabber_.setConfidenceThreshold (threshold_);
			}

			if (event.getKeyCode () == 'w')
			{
				search_mode_=1;
			}

			if (event.getKeyCode () == 'b')
			{
				fake_search=!fake_search;
				if(!fake_search)search_obj="";
			}

			if (event.getKeyCode () == 'm')
			{
				hand_search=!hand_search;
			}

			if (event.getKeyCode () == 'l')
			{
				export_objects=true;
			}

			displaySettings ();
		}
	}

	void displaySettings ()
	{
		const int dx = 5;
		const int dy = 14;
		const int fs = 10;
		boost::format name_fmt ("text%i");
		const char* TF[] = {"off", "median", "average"};
		std::vector<boost::format> entries;
		entries.push_back (boost::format ("%.1f fps") % grabber_.getFramesPerSecond ());
		
		// Current Model
		entries.push_back(boost::format("Distance to closest instance: %-0.3f") % obj_distance);
		entries.push_back(boost::format("Position of closest instance: %-0.3f, %-0.3f, %-0.3f") % closest_obj_centroid.x % closest_obj_centroid.y % closest_obj_centroid.z);
		entries.push_back(boost::format("Model: %s") % search_obj);
		if(hands_available){
			entries.push_back (boost::format ("Num Hands: %i") % numHands);
		}
		entries.push_back (boost::format ("Found Objects: %i") % boxes.size());
		entries.push_back (boost::format ("Plane Distance: %-0.3f") % plane_distance);
		entries.push_back (boost::format ("Algorithm: %s") % algorithms[algorithm_]);	

		// Settings
		entries.push_back (boost::format ("confidence threshold: %i") % threshold_);
		std::string tfs = boost::str (boost::format (", window size %i") % window_);
		entries.push_back (boost::format ("temporal filtering: %s%s") % TF[temporal_filtering_] % (temporal_filtering_ == pcl::RealSenseGrabber::RealSense_None ? "" : tfs));
		entries.push_back (boost::format ("Track Mode: %i") % search_mode_);
		entries.push_back(boost::format("Mode: %s") % (tracking_?"Tracking":"Scanning"));
		for (size_t i = 0; i < entries.size (); ++i)
		{
			std::string name = boost::str (name_fmt % i);
			std::string entry = boost::str (entries[i]);
			if (!viewer_.updateText (entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name))
				viewer_.addText (entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name);
		}
	}

	void drawInliers(pcl::visualization::PCLVisualizer& viz, pcl::PointIndices::Ptr inliers)
	{
		//Set pointCloud with particle's points
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inlier_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (size_t i = 0; i < inliers->indices.size (); ++i){
			pcl::PointXYZRGBA point;
			point.x = cloud_pass_downsampled_->points[inliers->indices[i]].x;
			point.y = cloud_pass_downsampled_->points[inliers->indices[i]].y;
			point.z = cloud_pass_downsampled_->points[inliers->indices[i]].z;
			inlier_cloud_->points.push_back(point);
		}

		//Draw red particles 
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red_color(inlier_cloud_, 250, 99, 71);

			if (!viz.updatePointCloud(inlier_cloud_, red_color, "inlier cloud"))
				viz.addPointCloud(inlier_cloud_, red_color, "inlier cloud");
		}
	}

	void drawFiltered(pcl::visualization::PCLVisualizer& viz)
	{
		print_with_level(4,"Viz start\n");
		viz.removeAllShapes();
		boost::format obj_name("found_obj_%s%i");
		for(int i=0;i<50;++i) viz.removePointCloud((obj_name%""%i).str());
		print_with_level(4,"Algorithm specific viz\n");

		if(algorithm_==4){
			boost::format cloud_labelling("box_%i_%s_%i_%i");
		for(int i=0;i<10;++i) viz.removeText3D((cloud_labelling%i%"name"%0%0).str());
		for(int i=0;i<10;++i) viz.removeText3D((cloud_labelling%i%"vname"%0%0).str());
			for(int i=0;i<boxes.size();++i){
				print_with_level(4,"Box gen\n");

				Eigen::Vector3f zAxisToTop(boxes[i].transformation_matrix.block<3,1>(0,2));
				zAxisToTop*=((boxes[i].height+0.01f)*(-0.5f));
				pcl::PointXYZ text_centroid(boxes[i].centroid.x+zAxisToTop.x(),boxes[i].centroid.y+zAxisToTop.y(),boxes[i].centroid.z+zAxisToTop.z());

				std::string cubeName((cloud_labelling%i%"validbox"%0%0).str());
				viz.addText3D((boost::format("%s_%s")%boxes[i].name%boxes[i].reference_colour.name).str(),text_centroid,0.01,1,1,1,(cloud_labelling%i%"name"%0%0).str());
				//viz.addCube(validBoxes[i].centroid.getArray3fMap(), relQ, validBoxes[i].width, validBoxes[i].length, validBoxes[i].height,cubeName);
				viz.addLine(boxes[i].verts[0],boxes[i].verts[1],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"ab"%0%0).str());
				viz.addLine(boxes[i].verts[1],boxes[i].verts[2],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"bc"%0%0).str());
				viz.addLine(boxes[i].verts[2],boxes[i].verts[3],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"cd"%0%0).str());
				viz.addLine(boxes[i].verts[3],boxes[i].verts[0],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"da"%0%0).str());

				viz.addLine(boxes[i].verts[4],boxes[i].verts[5],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"ef"%0%0).str());
				viz.addLine(boxes[i].verts[5],boxes[i].verts[6],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"fg"%0%0).str());
				viz.addLine(boxes[i].verts[6],boxes[i].verts[7],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"gh"%0%0).str());
				viz.addLine(boxes[i].verts[7],boxes[i].verts[4],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"he"%0%0).str());

				viz.addLine(boxes[i].verts[0],boxes[i].verts[4],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"ae"%0%0).str());
				viz.addLine(boxes[i].verts[1],boxes[i].verts[5],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"bf"%0%0).str());
				viz.addLine(boxes[i].verts[2],boxes[i].verts[6],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"cg"%0%0).str());
				viz.addLine(boxes[i].verts[3],boxes[i].verts[7],boxes[i].reference_colour.rgb.x(),boxes[i].reference_colour.rgb.y(),boxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"dh"%0%0).str());

				print_with_level(4,"box drawn\n");
			}
			for(int i=0;i<validBoxes.size();++i){
				print_with_level(4,"Valid Box gen\n");

				Eigen::Vector3f zAxisToTop(boxes[i].transformation_matrix.block<3,1>(0,2));
				zAxisToTop*=((validBoxes[i].height+0.01f)*(-0.5f));
				pcl::PointXYZ text_centroid(boxes[i].centroid.x+zAxisToTop.x(),boxes[i].centroid.y+zAxisToTop.y(),boxes[i].centroid.z+zAxisToTop.z());

				boost::format obj_locator("%s\n\t%s\n\tM:%s\n\tM:%s\n\tM:%s\n\tqw:%.3f\n\tqx:%.3f\n\tqy:%.3f\n\tqz:%.3f");

				viz.addText3D((obj_locator%boxes[i].name
					%validBoxes[i].reference_colour.name
					%(validBoxes[i].centroid.z>0?"forward":"back")
					%(validBoxes[i].centroid.y>0?"up":"down")
					%(validBoxes[i].centroid.x>0?"right":"left")
					%validBoxes[i].quaternion.w()
					%validBoxes[i].quaternion.x()
					%validBoxes[i].quaternion.y()
					%validBoxes[i].quaternion.z()
					).str(),text_centroid,0.01,255,255,255,(cloud_labelling%i%"vname"%0%0).str());

				viz.addLine(validBoxes[i].verts[0],validBoxes[i].verts[1],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vab"%0%0).str());
				viz.addLine(validBoxes[i].verts[1],validBoxes[i].verts[2],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vbc"%0%0).str());
				viz.addLine(validBoxes[i].verts[2],validBoxes[i].verts[3],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vcd"%0%0).str());
				viz.addLine(validBoxes[i].verts[3],validBoxes[i].verts[0],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vda"%0%0).str());

				viz.addLine(validBoxes[i].verts[4],validBoxes[i].verts[5],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vef"%0%0).str());
				viz.addLine(validBoxes[i].verts[5],validBoxes[i].verts[6],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vfg"%0%0).str());
				viz.addLine(validBoxes[i].verts[6],validBoxes[i].verts[7],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vgh"%0%0).str());
				viz.addLine(validBoxes[i].verts[7],validBoxes[i].verts[4],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vhe"%0%0).str());

				viz.addLine(validBoxes[i].verts[0],validBoxes[i].verts[4],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vae"%0%0).str());
				viz.addLine(validBoxes[i].verts[1],validBoxes[i].verts[5],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vbf"%0%0).str());
				viz.addLine(validBoxes[i].verts[2],validBoxes[i].verts[6],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vcg"%0%0).str());
				viz.addLine(validBoxes[i].verts[3],validBoxes[i].verts[7],validBoxes[i].reference_colour.rgb.x(),validBoxes[i].reference_colour.rgb.y(),validBoxes[i].reference_colour.rgb.z(),(cloud_labelling%i%"vdh"%0%0).str());

				print_with_level(4,"valid box drawn\n");
			}
		}

		if(hand_search){	
			boost::format hand_namer("hand_%i_%s");
			boost::format hand_typer("hand_%i_%s@%i%%\n\tM:%-1.3f\n\tM:%-1.3f\n\tM:%-1.3f\n\tNX:%-1.3f\n\tNY:%-1.3f\n\tNZ:%-1.3f");
			for(int i=0; i<hands.size();++i){
				SimpleHand h=hands[i];
				viz.addSphere(h.centroid,0.01,255,255,0,(hand_namer%i%"centroid").str());
				viz.addText3D((hand_typer%i%(h.open?"open":"closed")%h.openness
					%(h.centroid.x)
					%(h.centroid.y)
					%(h.centroid.z)
					%(h.normal.x())
					%(h.normal.y())
					%(h.normal.z())
					).str(),h.centroid,0.01,255,255,255,(hand_namer%i%"text").str());
			}
		}

		boost::format hull_line_name("hull_line_%i");
		for(int i=0;i<surface_hull_->points.size();++i){
			if(i==((surface_hull_->points.size())-1)){
				viz.addLine(surface_hull_->points[i],surface_hull_->points[0],1,0.7,0,(hull_line_name%i).str());
			}else{
				viz.addLine(surface_hull_->points[i],surface_hull_->points[i+1],1,0.7,0,(hull_line_name%i).str());
			}
		}
	}

	pcl::RealSenseGrabber& grabber_;
	pcl::visualization::PCLVisualizer viewer_;
	boost::signals2::connection connection_;

	int window_;
	int threshold_;
	int looper_;
	pcl::RealSenseGrabber::TemporalFilteringType temporal_filtering_;

	bool got_new_cloud_;
	mutable boost::mutex new_cloud_mutex_;
	Cloud::ConstPtr new_cloud_;
	Cloud::ConstPtr last_cloud_;
	Cloud::Ptr filtered_cloud_;
	Cloud::Ptr objects_cloud_;
	Cloud::Ptr surface_hull_;
	pcl::PointIndices::Ptr inliers;
	RefPointType scan_object_centroid_;
	pcl::ModelCoefficients::Ptr coefficients;
	bool plane_vision_;
	float plane_distance;
	pcl::PointXYZ closest_obj_centroid;
	float obj_distance;

	std::vector<Box> boxes;
	std::vector<Box> validBoxes;
	std::vector<BoxPartPlane> subBoxes;

	UdpTransmitSocket out_socket;

	//Hands
	PXCSession *g_session;
	PXCSenseManager *g_senseManager;
	PXCHandModule *g_handModule;
	PXCHandData *g_handDataOutput;
	PXCHandConfiguration *g_handConfiguration;
	std::vector<SimpleHand> hands;
	int numHands;
};

int main (int argc, char** argv)
{
	if (print_level_<=5){
		pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	}

	camToTool<<0,1,0,1,0,0,0,0,1;
	parseCommandLine(argc,argv);

	//Read in all objects
	try{
		boost::filesystem::path rel_path( object_location_ );
		if(boost::filesystem::exists(rel_path)){
			print_with_level(2,"Found object file path...\n");
			if(boost::filesystem::is_directory(rel_path)){
				print_with_level(2,"And it's a directory with");

				std::vector<boost::filesystem::path> paths;

				std::copy(boost::filesystem::directory_iterator(rel_path), boost::filesystem::directory_iterator(), std::back_inserter(paths));

				for (std::vector<boost::filesystem::path>::const_iterator it (paths.begin()); it != paths.end(); ++it)
				{
					boost::filesystem::path indi_file_(*it);

					if(boost::filesystem::is_regular_file(indi_file_) && (0==boost::filesystem::extension(indi_file_).compare(file_ending_)))	{

						cout << "XML File :" << indi_file_ << endl;
						boost::property_tree::ptree pt;
						boost::property_tree::read_xml(indi_file_.string(), pt);
						std::string name = pt.get<std::string>("box.name");
						PrimitiveBox pb(pt.get<float>("box.dimensions.l",0.0f),pt.get<float>("box.dimensions.w",0.0f),pt.get<float>("box.dimensions.h",0.0f));
						pb.approach=pcl::PointXYZ(pt.get<float>("box.approach.x",0.0f),pt.get<float>("box.approach.y",0.0f),pt.get<float>("box.approach.z",0.0f));
						cout<<"Creation Complete"<<endl;
						cout << "Box Primitive loaded from :" << indi_file_ << "Object Name :" << name << endl;

						if(box_library.count(name)==0){

							box_library.insert(std::pair<std::string,PrimitiveBox>(name,pb));
							print_with_level(1,"Object successfully added to map");

						}else{

							PCL_ERROR("An object with this name already exists, please check your object files");

						}

					}else if(boost::filesystem::is_regular_file(indi_file_)){

						cout << "Some Other File :" << indi_file_ << endl;

					}else if(boost::filesystem::is_directory(indi_file_)){

						cout << "Directory :" << indi_file_ << endl;
					}
				}
			}
		}else{
			print_with_level(2,"Couldn't find object file path\n");
		}
	}catch(const boost::filesystem::filesystem_error& ex){

		cout<<ex.what()<<endl;

	}

	//Setup colour library (could be XML but not needed yet)
	col_lib.add(PrimitiveColour("black",-1,0,Eigen::Vector3d(0,0,0)));
	col_lib.add(PrimitiveColour("red",0,15,Eigen::Vector3d(1,0,0)));
	col_lib.add(PrimitiveColour("red",340,360,Eigen::Vector3d(1,0,0)));
	col_lib.add(PrimitiveColour("orange",15,45,Eigen::Vector3d(1,0.65,0)));
	col_lib.add(PrimitiveColour("yellow",45,60,Eigen::Vector3d(1,1,0)));
	col_lib.add(PrimitiveColour("green",60,150,Eigen::Vector3d(0,1,0)));
	col_lib.add(PrimitiveColour("blue",150,250,Eigen::Vector3d(0,0,1)));
	col_lib.add(PrimitiveColour("purple",250,290,Eigen::Vector3d(0.62,0,1)));
	col_lib.add(PrimitiveColour("pink",290,340,Eigen::Vector3d(1,0,1)));

	//Start OSC Server
	boost::thread oscServer(DMRListener);
	print_with_level(2,"OSC Listener up and running\n");
	std::string device_id = "";
	try{
		pcl::RealSenseGrabber grabber (device_id);
		RealSenseTracker tracker (grabber);
		tracker.run ();
	}
	catch (pcl::io::IOException& e)
	{
		pcl::console::print_error ("Failed to create a grabber: %s\n", e.what ());
		return (1);
	}
	return 0;
}