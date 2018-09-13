#ifndef __SAVE_PATHPLANNING_HPP__
#define __SAVE_PATHPLANNING_HPP__

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// define data struct
typedef struct
{
    bool    IsInsideObstacle;//轮廓包围着的区域是障碍物，其外周是非障碍物，IsInsideObstacle=true；反之，轮廓包围着的区域是非障碍物，其外周是障碍物，IsInsideObstacle=false
    int     pointNum;//轮廓点序列实际点数
    Point   pt[256];//一般轮廓边沿点不会超过256个
    bool    IsConvextPoint[256];//判断对应序列的点是否为凸点。如果点是凸点，IsConvextPoint[i]=true；否者，IsConvextPoint[i]=false
    float   distance[256];  //有序端点两点之间的线段长度。第一个点和第二个点为第一个长度，依次类推。从最后一个点和第一个点间的长度为最后一个长度
} ContourPoints;

typedef struct
{
    bool    IsInsideObstacle; //轮廓包围着的区域是障碍物，其外周是非障碍物,IsInsideObstacle=true；反之；轮廓包围着的区域是障碍物，其外周是障碍物IsInsideObstacle=false
    int     pointNum;     //轮廓点序列实际点数
    int     ptIndex[256]; //一般轮廓边沿点不会超过256个，存储凸点对应于原始轮廓点序列的序号
    Point   pt[256];//一般轮廓边沿点不会超过256个
    float   distance[256]; //有序端点两点之间的线段长度。第一个点和第二个点为第一个长度，依次类推。从最后一个点和第一个点间的长度为最后一个长度
}  ConvexPoints;

typedef struct
{
     bool   IspointInGroove; //目标点在凹槽，IspointInGroove=true；
     bool   IsDirectFront;   //可以直接到达前点，IsDirectFront=true；
     bool   IsDirectBehind;  //可以直接到达后点，IsDirectBehind=true；
     int    ptFrontIndex;   //如果不存在凹槽，其表示目标点到达凸点没有障碍物时的两个凸点中，序号较小的一个凸点序号；如果有凹槽，则是比凹槽内的点序号小的凸点序号；
     int    ptBehindIndex;    //如果不存在凹槽，其表示目标到达凸点没有障碍物时的两个凸点中，序号较大的一个凸点序号；如果有凹槽，则是比凹槽内的点序号大的凸点序号；
}  PointInGroove;


void initContourPoints(ContourPoints& profile,vector<Point> pList);
bool isAllPointsSameSideWithLine(vector<Point> pList,int index1, int index2);
void findContourConvex(vector<Point> pList,ContourPoints& profile);
void initConvexPoints(ConvexPoints &convexPolygon,ContourPoints outerProfile);
int findNearestPointToGoal(ContourPoints& outerProfile,Point goal);
bool isGoalInContour(ContourPoints& outerProfile,Point test);
bool isGoalLineIntersectWithContour(ContourPoints& outerProfile, Point goalStart, Point goalEnd);
void judgePointInGroove(ContourPoints& outerProfile, PointInGroove& pointInGrooveA, Point goal, int goalNearestIndex);
void chooseBetterPath(ConvexPoints& convexPolygon,PointInGroove& pointInGrooveStart,Point goalStart,
                      PointInGroove& pointInGrooveEnd,Point goalEnd, vector<Point>& betterPath);
#endif
