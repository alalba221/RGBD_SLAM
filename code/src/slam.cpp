#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// choose optimazation method
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

// read a Frane with particular index
FRAME readFrame( int index, ParameterReader& pd );
// estimate the motion 
double normofTransform( cv::Mat rvec, cv::Mat tvec );

// the relationship between two frames 
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME}; 

/* 
 * check 2 frames, 
 * if its a good match and 2 vertices are neither in the loop, then add the vetex into the graph and a new edge
 * if these 2 vertices are in a loop, then just add en edge between them
 */ 
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );

// check the nearest loop 
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );

// check random loop 
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );

int main( int argc, char** argv )
{
    // 
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // Put all the key frame here
    vector< FRAME > keyframes; 
    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 
    FRAME lastFrame = readFrame( currIndex, pd ); // prev frame
    //FRAME currFrame = readFrame( currIndex, pd ); // prev frame
	 //  we always compare the current frame with the last frame
    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSTIC_PARAMETERS camera = getDefaultCamera();
    //computeKeyPointsAndDesp( currFrame, detector, descriptor );
	computeKeyPointsAndDesp( lastFrame, detector, descriptor );
    //PointCloud::Ptr cloud = image2PointCloud( currFrame.rgb, currFrame.depth, camera );
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
    /******************************* 
    // NEW: the initialization of G2O
    *******************************/
    // intialize Linear solver
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    g2o::SparseOptimizer globalOptimizer;  // this is what we finally got
    globalOptimizer.setAlgorithm( solver ); 
    // dont output information
    globalOptimizer.setVerbose( false );
    
    // add vertex to the globalOptimizer
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //
    v->setFixed( true ); //The first Vertex
    globalOptimizer.addVertex( v );
    
    //keyframes.push_back( currFrame );
	keyframes.push_back( lastFrame );
    
    double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");
    
    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // read current Frame
        computeKeyPointsAndDesp( currFrame, detector, descriptor ); //extract features
        CHECK_RESULT result = checkKeyframes( keyframes.back(), currFrame, globalOptimizer ); //compare currFrame and the last Key frame
        switch (result) // take different strategies according to the result
        {
        case NOT_MATCHED:
            //mismatched, skip over
            cout<<RED"Not enough inliers."<<endl;
            break;
        case TOO_FAR_AWAY:
            // too far, maybe wrong matched
            cout<<RED"Too far away, may be an error."<<endl;
            break;
        case TOO_CLOSE:
			// too close, cannot be a key frame
            cout<<YELLOW"Too close, not a keyframe"<<endl;
            break;
        case KEYFRAME:
            cout<<GREEN"This is a new keyframe"<<endl;
            // not too close nor too far
            /**
             * This is important!!
             * This is important!!
             * This is important!!
             * (very important so I've said three times!)
             */
            // close loop
            if (check_loop_closure)
            {
                checkNearbyLoops( keyframes, currFrame, globalOptimizer );
                checkRandomLoops( keyframes, currFrame, globalOptimizer );
            }
            keyframes.push_back( currFrame );
            
            break;
        default:
            break;
        }
        
    }
	//Opimize all edges
    cout<<RESET"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //set step numbers
    globalOptimizer.save( "./result_after.g2o" );
    cout<<"Optimization done."<<endl;

    // joint the point cloud
    cout<<"saving the point cloud map..."<<endl;
    PointCloud::Ptr output ( new PointCloud() ); //global map
    PointCloud::Ptr tmp ( new PointCloud() );

    pcl::VoxelGrid<PointT> voxel; // adjust the resolution of the map

    double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=0; i<keyframes.size(); i++)
    {
        // get a frame from g2o
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        Eigen::Isometry3d pose = vertex->estimate(); //pose of this frame after optimazing
        PointCloud::Ptr newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //convert to point cloud
        // filter
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );

        // add point cloud to global map
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );
    //save
    pcl::io::savePCDFile( "./result.pcd", *tmp );
    
    cout<<"Final map is saved."<<endl;
    return 0;
}

FRAME readFrame( int index, ParameterReader& pd )
{
    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");
    
    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    f.frameID = index;
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSTIC_PARAMETERS camera = getDefaultCamera();
    
	// compare frame1 and frame2

    RESULT_OF_PNP result = estimateMotion( f1, f2, camera );
    if ( result.inliers < min_inliers ) //number of inliers canot meet our standard
        return NOT_MATCHED;
    // calculate if the motion is too large
	
	// if ‖Δt‖+min(2π−‖r‖,‖r‖) becomes larger than max_norm
	// we can say the 2 frames are not contiuous frames,which means we got mismatch 
	// got wrong
    double norm = normofTransform(result.rvec, result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }
	
	// too close
 	if ( norm <= keyframe_threshold )
		return TOO_CLOSE; 

	
    // add an edge between this vertex and last frame
    // For vertex
    // only need to set the vertex id
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // For Edge
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // the ids of the 2 vertices of this edge
    edge->setVertex( 0, opti.vertex(f1.frameID ));
    edge->setVertex( 1, opti.vertex(f2.frameID ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    // information matrix
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    // edge->setMeasurement( T );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);
    


    return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int nearby_loops = atoi( pd.getData("nearby_loops").c_str() );
    
    // check current frame with the nearest key frames 
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // randomly pick some key frames
    
    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}
