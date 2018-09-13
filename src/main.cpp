#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>

#include "PathPlanning.hpp"

using namespace std;
using namespace cv;


int main()
{
    Mat globalMap = imread("../PathPlanTest1.png", 0);

    //cout<<"globalMap:"<<globalMap.channels()<<",size:"<<globalMap.size<<endl;

    // define outline vertex
    //vector constuct function: https://blog.csdn.net/tiaohua/article/details/4803994
    Point pointsArray[8] = {Point(67,278), Point(308,278),Point(308,75),Point(248,75),
                            Point(248,147),Point(135,147),Point(135,75),Point(67,75)};
    vector<Point> outline(&pointsArray[0],&pointsArray[8]);

    //draw these vertex and mark their orders
    //cout<<"outline:"<<outline.size()<<endl<<outline<<endl;
    for(int i=0;i<outline.size();i++){
        circle(globalMap,outline[i],2,Scalar(0,0,255),1);
        ostringstream indexText;
        indexText << i;
        putText(globalMap,indexText.str(),outline[i],cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0,0,255 ), 1);
    }


    // define start and end goal points
    Point goalStart(20, 320);
    //Point goalEnd(150,120);//(100, 50)outside    (150,120) inside groove
    //Point goalEnd(90,60);
    //Point goalEnd(290,60);  //***
    Point goalEnd(320,100);

    //draw the goal points
    circle(globalMap,goalStart,2,Scalar(0,0,255),1.5);
    putText(globalMap,String("S"),goalStart,cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0,0,255 ), 1.5);
    circle(globalMap,goalEnd,2,Scalar(0,0,255),1.5);
    putText(globalMap,String("E"),goalEnd,cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0,0,255 ), 1.5);

    // find convex outline from the raw point set
    ContourPoints outerProfile;
    initContourPoints(outerProfile,outline);
    findContourConvex(outline,outerProfile);//update its convex, better if findConvexVertex(outerProfile)

    //test if marked successfully
    for(int i=0; i<outerProfile.pointNum;i++){
        cout<<"i="<<i<<",pt="<<outerProfile.pt[i]<<",dis="<<outerProfile.distance[i]
            <<",convex="<<outerProfile.IsConvextPoint[i]<<endl;
    }

    // define ConvexPolygon, and initialize it with ContourPoints above
    ConvexPoints convexPolygon;
    initConvexPoints(convexPolygon,outerProfile);
    for(int i=0; i<convexPolygon.pointNum;i++){
        cout<<"i="<<i<<",index="<<convexPolygon.ptIndex[i]<<",pt="<<convexPolygon.pt[i]<<",dis="<<convexPolygon.distance[i]<<endl;
    }

    // seek the point which is nearest to the goal (containing start or end point)
    int goalStartNearestIndex = findNearestPointToGoal(outerProfile,goalStart);
    int goalEndNearestIndex = findNearestPointToGoal(outerProfile,goalEnd);
    cout<<"Nearest Point Index Start:"<<goalStartNearestIndex<<",End:"<<goalEndNearestIndex<<endl;

    // judge if goal goal inside or outside of the contour
    bool isGoalStartInContour = isGoalInContour(outerProfile,goalStart);
    bool isGoalEndInContour   = isGoalInContour(outerProfile,goalEnd);
    cout<<"Is in Contour start:"<<isGoalStartInContour<<",end:"<<isGoalEndInContour<<endl;

    if(isGoalStartInContour | isGoalEndInContour){//both or one goal point in obstance
        //act up-->move-->down
        //plan finished

    } else{// both are outside of contour
        // judge the line of goal points intersect with the contour
        bool isIntersect = isGoalLineIntersectWithContour(outerProfile,goalStart,goalEnd);
        cout<<"isGoalLineIntersectWithContour:"<<isIntersect<<endl;

        if(!isIntersect){
            //walk directly between goalStart and goalEnd
            //plan finished
        } else {
            PointInGroove pointInGrooveStart,pointInGrooveEnd;
            judgePointInGroove(outerProfile, pointInGrooveStart, goalStart, goalStartNearestIndex);
            judgePointInGroove(outerProfile, pointInGrooveEnd, goalEnd, goalEndNearestIndex);

            vector<Point> betterPath;
            chooseBetterPath(convexPolygon,pointInGrooveStart,goalStart,pointInGrooveEnd,goalEnd,betterPath);

            // draw the better path
            for(int i=0; i<betterPath.size(); i++){
                circle(globalMap,betterPath[i],7,Scalar(0,0,255),2);
//                ostringstream indexText;
//                indexText << "L"<<i;
//                putText(globalMap,indexText.str(),betterPath[i],cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,0,0), 1);

            }
        }
    }

    imshow("globalMap",globalMap);

    waitKey(0);
}



