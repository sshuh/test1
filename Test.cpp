// Test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Test.h"

// Need to delete DEBUG_NEW
//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif

#include <iostream>

#undef max	// Need to undef max for PCL
#undef min	// Need to undef min for PCL

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
//#include <pcl/io/openni_grabber.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/octree/octree.h>

#include "OpenCV2.4.5\include\opencv2\opencv.hpp"
//#include "OpenCV2.4.5/include/opencv2/gpu/gpu.hpp"



#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/filters/filter.h>

#include <vtkCubeSource.h>
#include <vtkRenderer.h>

#include <vtkWin32OpenGLRenderWindow.h>
#include <vtkWin32RenderWindowInteractor.h>

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>


//=============================
// Displaying cubes is very long!
// so we limit their numbers.
 const int MAX_DISPLAYED_CUBES(15000);
//=============================



// The one and only application object

CWinApp theApp;

//using namespace std;
using namespace cv;
/*
class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer"){}
	pcl::visualization::CloudViewer viewer;
	pcl::visualization::ImageViewer imgViewer;

	void cloud_cb_1(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
		if(!viewer.wasStopped())
			viewer.showCloud (cloud);
	}

	void runDepth()
	{
		pcl::Grabber* interface1 = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_1, this, _1);

		interface1->registerCallback(f);

		interface1->start();

		while(!viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

		interface1->stop();
	}

	void cloud_cb_2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		static unsigned count = 0;
		static double last = pcl::getTime();
		if (++count == 30)
		{
			double now = pcl::getTime();
			std::cout << "distance of center pixel :" << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
			count = 0;
			last = now;
		}

		if(!viewer.wasStopped())
			viewer.showCloud(cloud);
	}


	void runDepthColor()
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* interface1 = new pcl::OpenNIGrabber( );

		// make callback function from member function
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_2, this, _1);

		// connect callback function for desired signal. In this case its a point cloud with color values
		boost::signals2::connection c = interface1->registerCallback(f);

		// start receiving point clouds
		interface1->start();

		
		//interface1-
		// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
		while(true)
			boost::this_thread::sleep( boost::posix_time::seconds(1) );

		// stop the grabber
		interface1->stop();
	}
	
	cv::Mat m_imgDepth;
	cv::Mat m_imgColor;
	cv::Mat m_imgColorBGR;

	void Initial()
	{
		m_imgDepth.create( 480, 640, CV_32FC1 );
		m_imgColorBGR.create( 480, 640, CV_8UC3 );
		m_imgColor.create( 480, 640, CV_8UC3 );
	}

	void callbackColorImage( const boost::shared_ptr<openni_wrapper::Image> &imgColor )
	{
		//if(!imgViewer.wasStopped())
		//	imgViewer.showFloatImage( (float*)cloud,  );
		imgColor->fillRGB(640, 480, m_imgColorBGR.data);
		cv::cvtColor( m_imgColorBGR, m_imgColor, CV_BGR2RGB );
        
		circle( m_imgColor, Point(320,240), 5, CV_RGB(255,0,0), 1 );
		imshow("Color Display", m_imgColor);

        cv::waitKey(30);
		
	}

	void callbackDepthImage( const boost::shared_ptr<openni_wrapper::DepthImage> &imgDepth )
	{
		//if(!imgViewer.wasStopped())
		//	imgViewer.showFloatImage( (float*)cloud,  );

		imgDepth->fillDepthImage(640, 480, (float*)m_imgDepth.data);
        imshow("Depth Display", m_imgDepth);

		float fRange = m_imgDepth.at<float>(240,320);
		printf("dist=%.2f\n", fRange );
        cv::waitKey(30);
		
	}

	void callbackDepthColor( const boost::shared_ptr<openni_wrapper::Image> &imgColor, const boost::shared_ptr<openni_wrapper::DepthImage> &imgDepth )
	{
		//if(!imgViewer.wasStopped())
		//	imgViewer.showFloatImage( (float*)cloud,  );

		imgColor->fillRGB(640, 480, m_imgColorBGR.data);
		cv::cvtColor( m_imgColorBGR, m_imgColor, CV_BGR2RGB );
        
		circle( m_imgColor, Point(320,240), 5, CV_RGB(255,0,0), 1 );
		imshow("Color Display", m_imgColor);


		imgDepth->fillDepthImage(640, 480, (float*)m_imgDepth.data);
        imshow("Depth Display", m_imgDepth);

		float fRange = m_imgDepth.at<float>(240,320);
		printf("dist=%.2f\n", fRange );
        cvWaitKey(30);
		
	}

	void runDepthImage()
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* interface1 = new pcl::OpenNIGrabber( );

		// make callback function from member function
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& )> imgColor = boost::bind(&SimpleOpenNIViewer::callbackColorImage, this, _1);
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>& )> imgDepth = boost::bind(&SimpleOpenNIViewer::callbackDepthImage, this, _1);
		
		// make callback function from member function
//		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>& )> f = boost::bind(&SimpleOpenNIViewer::callbackDepthColor, this, _1, _2);

		// connect callback function for desired signal. In this case its a point cloud with color values
		boost::signals2::connection c = interface1->registerCallback(imgColor);
		boost::signals2::connection d = interface1->registerCallback(imgDepth);

//		boost::signals2::connection c = interface1->registerCallback(f);
		// start receiving point clouds
		interface1->start();

		
		//interface1-
		// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
		while(true)
			boost::this_thread::sleep( boost::posix_time::seconds(1) );

		// stop the grabber
		interface1->stop();
	}

};
*/


void PCL_writePCD()
{

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 1000;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 4 * rand () / (RAND_MAX + 1.0f);
  }


  pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

}


void PCL_readPCD()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>);

	// table_scene_lms400.pcd
	// test_pcd.pcd
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("octreelog.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
	}

	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;
/*	
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		std::cout << "    " << cloud->points[i].x
			      << " "    << cloud->points[i].y
				  << " "    << cloud->points[i].z << std::endl;
	}
*/	
	

/*
	// Plane model segmentation


	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud->makeShared());
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return;
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
										<< coefficients->values[1] << " "
										<< coefficients->values[2] << " " 
										<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	for (size_t i = 0; i < inliers->indices.size (); ++i)
	std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
												<< cloud->points[inliers->indices[i]].y << " "
												<< cloud->points[inliers->indices[i]].z << std::endl;

*/


	//... populate cloud
	pcl::visualization::CloudViewer viewer ("PcdViewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}

}




int KinectOpenCV()
{
	
	cv::Mat imgDepth;
	cv::Mat imgColor;
	cv::VideoCapture capture;
	
	capture.open( CV_CAP_OPENNI );


	const int minDistance = 400; // mm
	float b = capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_BASELINE ); // mm
	float F = capture.get( CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH ); // pixels

	for(;;)
	{
		//if( !capture.isOpened() )
		//	break;

		if( !capture.grab() )
			break;
		//
		capture.retrieve( imgDepth, CV_CAP_OPENNI_DEPTH_MAP );
		capture.retrieve( imgColor, CV_CAP_OPENNI_BGR_IMAGE );

		imshow( "imgDepth", imgDepth );
		imshow( "imgColor", imgColor );
		
		if( cv::waitKey( 30 ) == 27 )
			break;
	}

	return 1;
}




int OctreeTest()
{
	
	srand ((unsigned int) time (NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = 10;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
	}

	cloud->width = 12;
	cloud->points.resize (cloud->width * cloud->height);

	cloud->points.push_back( pcl::PointXYZ(100, 0, 0) );
	cloud->points.push_back( pcl::PointXYZ(0, 20, 0) );


	float resolution = 4.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();

	pcl::PointXYZ searchPoint;

	searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

	// Neighbors within voxel search

	std::vector<int> pointIdxVec;

	if (octree.voxelSearch (searchPoint, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint.x 
		<< " " << searchPoint.y 
		<< " " << searchPoint.z << ")" 
		<< std::endl;
              
		for (size_t i = 0; i < pointIdxVec.size (); ++i)
			std::cout << "    " << cloud->points[pointIdxVec[i]].x 
					<< " " << cloud->points[pointIdxVec[i]].y 
					<< " " << cloud->points[pointIdxVec[i]].z << std::endl;
	}

	// K nearest neighbor search

	int K = 10;

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	std::cout << "K nearest neighbor search at (" << searchPoint.x 
	<< " " << searchPoint.y 
	<< " " << searchPoint.z
	<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
			std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// Neighbors within radius search

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x 
			<< " " << searchPoint.y 
			<< " " << searchPoint.z
			<< ") with radius=" << radius << std::endl;


	if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
			std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}

	bool bOccuppied = 0;
	pcl::PointXYZ ptQuery = pcl::PointXYZ( 10.f, 20.f, 30.f );
	bOccuppied = octree.isVoxelOccupiedAtPoint( ptQuery );

	ptQuery = pcl::PointXYZ( 20.f, 20.f, 20.f );
	bOccuppied = octree.isVoxelOccupiedAtPoint( ptQuery );

	bOccuppied = octree.isVoxelOccupiedAtPoint( cloud->points[0] );
	
	//pcl::visualization::createCube(


	// ---- ray-casting
	pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::AlignedPointTVector voxelList;
	voxelList.clear();

	Eigen::Vector3f vOrigin(0, 0, 0);
	Eigen::Vector3f vDirection(1, 0, 0);


	int nPoints = octree.getIntersectedVoxelCenters( vOrigin, vDirection, voxelList, 0 );
	printf("vector intersected %d voxels\n", nPoints);
	std::vector <int> k_indices;
	nPoints = octree.getIntersectedVoxelIndices(vOrigin, vDirection, k_indices);
	if( k_indices.size() > 0 )
		printf("vector intersected %d voxels, %d, %.3f\n", nPoints, k_indices[0], cloud->points[k_indices[0]].x );

	vDirection = Eigen::Vector3f(0, 1, 0);
 	voxelList.clear();
	nPoints = octree.getIntersectedVoxelCenters( vOrigin, vDirection, voxelList, 0 ); 
	printf("vector intersected %d voxels\n", nPoints);



	// ray-casting ----

	pcl::visualization::CloudViewer viewer ("PcdViewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}


	return 1;
}





class OctreeViewer
{
public:
  OctreeViewer (std::string &filename, double resolution) :
    viz ("Octree visualizator"), cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        displayCloud (new pcl::PointCloud<pcl::PointXYZ>()), octree (resolution), displayCubes(false),
        showPointsWithCubes (false), wireframe (true)
  {

    //try to load the cloud
    if (!loadCloud(filename))
      return;

    //register keyboard callbacks
    viz.registerKeyboardCallback(&OctreeViewer::keyboardEventOccurred, *this, 0);

    //key legends
    viz.addText("Keys:", 0, 170, 0.0, 1.0, 0.0, "keys_t");
    viz.addText("a -> Increment displayed depth", 10, 155, 0.0, 1.0, 0.0, "key_a_t");
    viz.addText("z -> Decrement displayed depth", 10, 140, 0.0, 1.0, 0.0, "key_z_t");
    viz.addText("d -> Toggle Point/Cube representation", 10, 125, 0.0, 1.0, 0.0, "key_d_t");
    viz.addText("x -> Show/Hide original cloud", 10, 110, 0.0, 1.0, 0.0, "key_x_t");
    viz.addText("s/w -> Surface/Wireframe representation", 10, 95, 0.0, 1.0, 0.0, "key_sw_t");

    //set current level to half the maximum one
    displayedDepth = static_cast<int> (floor (octree.getTreeDepth() / 2.0));
    if (displayedDepth == 0)
      displayedDepth = 1;

    //show octree at default depth
    extractPointsAtLevel(displayedDepth);

    //reset camera
    viz.resetCameraViewpoint("cloud");

    //run main loop
    run();

  }

private:
  //========================================================
  // PRIVATE ATTRIBUTES
  //========================================================
  //visualizer
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;

  pcl::visualization::PCLVisualizer viz;
  //original cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //displayed_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
  //octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
  //level
  int displayedDepth;
  //bool to decide if we display points or cubes
  bool displayCubes, showPointsWithCubes, wireframe;
  //========================================================

  /* \brief Callback to interact with the keyboard
   *
   */
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
  {

    if (event.getKeySym() == "a" && event.keyDown())
    {
      IncrementLevel();
    }
    else if (event.getKeySym() == "z" && event.keyDown())
    {
      DecrementLevel();
    }
    else if (event.getKeySym() == "d" && event.keyDown())
    {
      displayCubes = !displayCubes;
      update();
    }
    else if (event.getKeySym() == "x" && event.keyDown())
    {
      showPointsWithCubes = !showPointsWithCubes;
      update();
    }
    else if (event.getKeySym() == "w" && event.keyDown())
    {
      if(!wireframe)
        wireframe=true;
      update();
    }
    else if (event.getKeySym() == "s" && event.keyDown())
    {
      if(wireframe)
        wireframe=false;
      update();
    }
  }

  /* \brief Graphic loop for the viewer
   *
   */
  void run()
  {
    while (!viz.wasStopped())
    {
      //main loop of the visualizer
      viz.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    }
  }

  /* \brief Helper function that read a pointcloud file (returns false if pbl)
   *  Also initialize the octree
   *
   */
  bool loadCloud(std::string &filename)
  {
    std::cout << "Loading file " << filename.c_str() << std::endl;
    //read cloud
    if (pcl::io::loadPCDFile(filename, *cloud))
    {
      std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
      return false;
    }

    //remove NaN Points
    std::vector<int> nanIndexes;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
    std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

    //create octree structure
    octree.setInputCloud(cloud);
    //update bounding box automatically
    octree.defineBoundingBox();
    //add points in the tree
    octree.addPointsFromInputCloud();
    return true;
  }

  /* \brief Helper function that draw info for the user on the viewer
   *
   */
  void showLegend(bool showCubes)
  {
    char dataDisplay[256];
    sprintf(dataDisplay, "Displaying data as %s", (showCubes) ? ("CUBES") : ("POINTS"));
    viz.removeShape("disp_t");
    viz.addText(dataDisplay, 0, 60, 1.0, 0.0, 0.0, "disp_t");

    char level[256];
    sprintf(level, "Displayed depth is %d on %d", displayedDepth, octree.getTreeDepth());
    viz.removeShape("level_t1");
    viz.addText(level, 0, 45, 1.0, 0.0, 0.0, "level_t1");

    viz.removeShape("level_t2");
    sprintf(level, "Voxel size: %.4fm [%zu voxels]", sqrt(octree.getVoxelSquaredSideLen(displayedDepth)),
            displayCloud->points.size());
    viz.addText(level, 0, 30, 1.0, 0.0, 0.0, "level_t2");

    viz.removeShape("org_t");
    if (showPointsWithCubes)
      viz.addText("Displaying original cloud", 0, 15, 1.0, 0.0, 0.0, "org_t");
  }

  /* \brief Visual update. Create visualizations and add them to the viewer
   *
   */
  void update()
  {
    //remove existing shapes from visualizer
    clearView();

    //prevent the display of too many cubes
    bool displayCubeLegend = displayCubes && static_cast<int> (displayCloud->points.size ()) <= MAX_DISPLAYED_CUBES;

    showLegend(displayCubeLegend);

	//viz.setCameraPosition(-10000.0, 0.0, 0.0, -1.0, 0.0, 0.0 );

    if (displayCubeLegend)
    {
      //show octree as cubes
      showCubes(sqrt(octree.getVoxelSquaredSideLen(displayedDepth)));
      if (showPointsWithCubes)
      {
        //add original cloud in visualizer
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
        viz.addPointCloud(cloud, color_handler, "cloud");
      }
    }
    else
    {
      //add current cloud in visualizer
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(displayCloud,"z");
      viz.addPointCloud(displayCloud, color_handler, "cloud");
    }
  }

  /* \brief remove dynamic objects from the viewer
   *
   */
  void clearView()
  {
    //remove cubes if any
    vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();
    while (renderer->GetActors()->GetNumberOfItems() > 0)
      renderer->RemoveActor(renderer->GetActors()->GetLastActor());
    //remove point clouds if any
    viz.removePointCloud("cloud");
  }

  /* \brief Create a vtkSmartPointer object containing a cube
   *
   */
  vtkSmartPointer<vtkPolyData> GetCuboid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
  {
    vtkSmartPointer <vtkCubeSource> cube = vtkSmartPointer <vtkCubeSource>::New();
    cube->SetBounds(minX, maxX, minY, maxY, minZ, maxZ);
    return cube->GetOutput();
  }

  /* \brief display octree cubes via vtk-functions
   *
   */
  void showCubes(double voxelSideLen)
  {
    //get the renderer of the visualizer object
    vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();

    vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New();
    size_t i;
    double s = voxelSideLen / 2.0;
    for (i = 0; i < displayCloud->points.size(); i++)
    {

      double x = displayCloud->points[i].x;
      double y = displayCloud->points[i].y;
      double z = displayCloud->points[i].z;

      treeWireframe->AddInput(GetCuboid(x - s, x + s, y - s, y + s, z - s, z + s));
    }

    vtkSmartPointer<vtkActor> treeActor = vtkSmartPointer<vtkActor>::New();

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
    mapper->SetInput(treeWireframe->GetOutput());
    treeActor->SetMapper(mapper);

    treeActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    treeActor->GetProperty()->SetLineWidth(2);
    if(wireframe)
    {
      treeActor->GetProperty()->SetRepresentationToWireframe();
      treeActor->GetProperty()->SetOpacity(0.35);
    }
    else
      treeActor->GetProperty()->SetRepresentationToSurface();

    renderer->AddActor(treeActor);
  }

  /* \brief Extracts all the points of depth = level from the octree
   *
   */
  void extractPointsAtLevel(int depth)
  {
    displayCloud->points.clear();

    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it_end = octree.end();

    pcl::PointXYZ pt;
    std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
    double start = pcl::getTime ();

    for (tree_it = octree.begin(depth); tree_it!=tree_it_end; ++tree_it)
    {
      Eigen::Vector3f voxel_min, voxel_max;
      octree.getVoxelBounds(tree_it, voxel_min, voxel_max);

      pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
      pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
      pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
      displayCloud->points.push_back(pt);
    }

    double end = pcl::getTime ();
    printf("%zu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
           (end - start) / static_cast<double> (displayCloud->points.size ()));

    update();
  }

  /* \brief Helper function to increase the octree display level by one
   *
   */
  bool IncrementLevel()
  {
    if (displayedDepth < static_cast<int> (octree.getTreeDepth ()))
    {
      displayedDepth++;
      extractPointsAtLevel(displayedDepth);
      return true;
    }
    else
      return false;
  }

  /* \brief Helper function to decrease the octree display level by one
   *
   */
  bool DecrementLevel()
  {
    if (displayedDepth > 0)
    {
      displayedDepth--;
      extractPointsAtLevel(displayedDepth);
      return true;
    }
    return false;
  }

};



int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// initialize MFC and print and error on failure
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: change error code to suit your needs
			_tprintf(_T("Fatal Error: MFC initialization failed\n"));
			nRetCode = 1;
		}
		else
		{
			// TODO: code your application's behavior here.

			//PCL_readPCD();

			//SimpleOpenNIViewer v;
			//v.Initial();
			//v.runDepthImage();

			//OctreeTest();

			std::string cloud_path("octreelog2.pcd");
			OctreeViewer v(cloud_path, 2.0);
			//std::string cloud_path("octreelog.pcd");
			//OctreeViewer v(cloud_path, 0.02);
			

			//KinectOpenCV();
		}
	}
	else
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: GetModuleHandle failed\n"));
		nRetCode = 1;
	}

	return nRetCode;
}
