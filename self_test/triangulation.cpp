#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // used in opencv2 
using namespace std;
using namespace cv;

struct TrackedFeature
{
  int id = -1;
  double x = -1.0;
  double y = -1.0;
//   double ts = -1;
};

//read three lines from the output_result, i.e. one spatiotemporal window - or one frame.
istream &read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp);

void feature_match (vector<TrackedFeature> &feature_list_1, 
                    vector<TrackedFeature> &feature_list_2,
                    vector<Point2f> &points1,
                    vector<Point2f> &points2);

void pose_estimation_2d2d (vector<Point2f> &points1,
                           vector<Point2f> &points2,
                           Mat& R, Mat& t );

void triangulation ( 
    const vector<Point2f> &points1,
    const vector<Point2f> &points2, 
    const Mat& R, const Mat& t, 
    vector< Point3d >& points );

// 像素坐标转相机归一化坐标
Point2f pixel2cam( const Point2d& p, const Mat& K );

int main ( int argc, char** argv )
{
    ifstream fin("/home/albert/workSpace/data/output_result_day_long2.txt");
    vector<TrackedFeature> feature_list_1, feature_list_2;
    double timestamp_1 = -1.0, timestamp_2 = -1.0;
    read_frame(fin, feature_list_1, timestamp_1);
    read_frame(fin, feature_list_2, timestamp_2);
    vector<Point2f> points1;
    vector<Point2f> points2;
    feature_match(feature_list_1,feature_list_2,points1,points2);
    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( points1, points2, R, t );
    //-- 三角化
    vector<Point3d> points;
    triangulation( points1, points2, R, t, points );
    
    //-- 验证三角化点与特征点的重投影关系
    Mat K = ( Mat_<double> ( 3,3 ) << 194.8, 0, 170.2, 0, 194.8, 127.0, 0, 0, 1);
    for ( int i=0; i<points1.size(); i++ )
    {
        Point2d pt1_cam = pixel2cam( points1[i], K );
        Point2d pt1_cam_3d(
            points[i].x/points[i].z, 
            points[i].y/points[i].z 
        );
        
        cout<<"point in the first camera frame: "<<pt1_cam<<endl;
        cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<endl;
        
        // 第二个图
        Point2f pt2_cam = pixel2cam( points2[i], K );
        Mat pt2_trans = R*( Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
        pt2_trans /= pt2_trans.at<double>(2,0);
        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
        cout<<endl;
    }
    
    return 0;
}

void feature_match (vector<TrackedFeature> &feature_list_1, 
                    vector<TrackedFeature> &feature_list_2,
                    vector<Point2f> &points1,
                    vector<Point2f> &points2)
{
    //-- 把匹配点转换为vector<Point2f>的形式
    Point2f temp_point;
    int match_counter = 0;
    
    for(int i=0; i<feature_list_1.size(); i++)
    {
        for(int j=0; j<feature_list_2.size(); j++)
        {
            if(feature_list_1[i].id==feature_list_2[j].id)
            {
                temp_point.x = feature_list_1[i].x;
                temp_point.y = feature_list_1[i].y;
                points1.push_back(temp_point);
                temp_point.x = feature_list_2[j].x;
                temp_point.y = feature_list_2[j].y;
                points2.push_back(temp_point);
                break;
            }
        }
    }
}

void pose_estimation_2d2d (vector<Point2f> &points1,
                           vector<Point2f> &points2, 
                           Mat& R,  Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 194.8, 0, 170.2, 0, 194.8, 127.0, 0, 0, 1 );

    for(int i=0;i<points1.size();i++)
    {
        cout<<"matched points:\n"<<points1[i].x<<"  "<<points2[i].x<<endl;
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 170.2, 127.0 );	//相机光心, TUM dataset标定值
    double focal_length = 194.8;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
    
}

void triangulation ( 
    const vector<Point2f> &points1,
    const vector<Point2f> &points2, 
    const Mat& R, const Mat& t, 
    vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );
    
    Mat K = ( Mat_<double> ( 3,3 ) << 194.8, 0, 170.2, 0, 194.8, 127.0, 0, 0, 1);
    vector<Point2f> pts_1, pts_2;
    for (int  i=0;i<points1.size();i++ )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( points1[i], K) );
        pts_2.push_back ( pixel2cam( points2[i], K) );
    }
    
    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
    
    // 转换成非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        Point3d p (
            x.at<float>(0,0), 
            x.at<float>(1,0), 
            x.at<float>(2,0) 
        );
        points.push_back( p );
    }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}

//read three lines from the output_result, i.e. one spatiotemporal window - or one frame.
istream &read_frame(istream &fin, vector<TrackedFeature> &feature_list, double &timestamp)
{
    TrackedFeature temp_Feature;
    string line;
    // cout<<"Start reading frame (three lines)"<<endl;

    for(int line_counter=0;line_counter<3;line_counter++)
    {
        getline(fin, line);
        istringstream iss(line);
        if(line_counter%3 == 0)//id row
        {
            iss>>timestamp;
            while(iss>>temp_Feature.id)
            {
                // iss>>temp_Feature.id;
                feature_list.push_back(temp_Feature);
            }
        }
        else if(line_counter%3 == 1)//x row
        {
            double temp = -1;
            double temp_x = -1;
            int position = 0;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                // return nullptr;
            }
            while(iss>>temp_x)
            {
                feature_list.at(position).x = temp_x;
                position++;
            }
        }
        else if(line_counter%3 == 2)//y row
        {
            double temp = -1;
            double temp_y = -1;
            int position = 0;
            iss>>temp;
            if(temp!=timestamp)
            {
                cout<<"ts="<<temp<<", timestamp is wrong!"<<endl;
                // return 0;
            }
            while(iss>>temp_y)
            {
                feature_list.at(position).y = temp_y;
                position++;
            }
        }
    }
    return fin;
}