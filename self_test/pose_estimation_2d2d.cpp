#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // use this if in OpenCV2 
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


void pose_estimation_2d2d (
    vector<TrackedFeature> &feature_list_1,
    vector<TrackedFeature> &feature_list_2,
    Mat& R, Mat& t );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

int main ( int argc, char** argv )
{
    ifstream fin("/home/albert/workSpace/data/output_result_day_long2.txt");
    vector<TrackedFeature> feature_list_1, feature_list_2;
    double timestamp_1 = -1.0, timestamp_2 = -1.0;
    read_frame(fin, feature_list_1, timestamp_1);
    read_frame(fin, feature_list_2, timestamp_2);
    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( feature_list_1, feature_list_2, R, t );

    //-- 验证E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
                t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
                -t.at<double> ( 1,0 ),     t.at<double> ( 0,0 ),      0 );

    cout<<"t^R="<<endl<<t_x*R<<endl;

    //-- 验证对极约束
    // Mat K = ( Mat_<double> ( 3,3 ) << 194.8, 0, 170.2, 0, 194.8, 127.0, 0, 0, 1 );
    // for ( DMatch m: matches )
    // {
    //     Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
    //     Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
    //     Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
    //     Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
    //     Mat d = y2.t() * t_x * R * y1;
    //     cout << "epipolar constraint = " << d << endl;
    // }
    return 0;
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


void pose_estimation_2d2d (vector<TrackedFeature> &feature_list_1, 
                           vector<TrackedFeature> &feature_list_2, 
                           Mat& R,  Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 194.8, 0, 170.2, 0, 194.8, 127.0, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;
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