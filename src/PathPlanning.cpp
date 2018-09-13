#include "PathPlanning.hpp"

using namespace std;
using namespace cv;

//ContourPoints outerProfile = {true,0};
//ContourPoints internalProfile = {false,0};

//anti-clockwise, p1,p2,p3...p(n-1)

//initialize ContourPoints its pointNum, pt, distance
void initContourPoints(ContourPoints& profile,vector<Point> pList){

    profile.IsInsideObstacle=true;//caution:outer or internal contour
    int pointNum = pList.size();
    profile.pointNum = pointNum;
    Point temp;
    for(int i=0; i<pointNum-1; i++){
        profile.IsConvextPoint[i]=false;
        profile.pt[i] = pList[i];
        temp = pList[i+1]-pList[i];
        profile.distance[i] = sqrt(temp.x*temp.x + temp.y*temp.y);
    }
    //when i=pointNum-1
    profile.pt[pointNum-1] = pList[pointNum-1];
    temp = pList[0]- pList[pointNum-1];
    profile.distance[pointNum-1] = sqrt(temp.x*temp.x + temp.y*temp.y);

}

// update ContourPoints its IsConvextPoint, mark its convext or groove
// find outside convex contour from the raw points
void findContourConvex(vector<Point> pList,ContourPoints& profile){

    int startIndex = pList.size()-1;
    int endIndex = 0;
    bool IsStartPointConvex = false;

    for(int i=0; i< pList.size(); i++) {
        bool isSameSide = isAllPointsSameSideWithLine(pList,startIndex,endIndex);
        //cout<<"i="<<i<<",isSameSide:"<<isSameSide<<endl;
        if(isSameSide){
            profile.IsConvextPoint[startIndex]=true;
            profile.IsConvextPoint[endIndex]=true;
            IsStartPointConvex=true;
            startIndex=endIndex;
            endIndex=endIndex+1;
        }else{
            if(IsStartPointConvex){
                endIndex=endIndex+1;
            }else{
                startIndex=endIndex;
                endIndex=endIndex+1;
            }
        }
    }
}

bool isAllPointsSameSideWithLine(vector<Point> pList,int index1, int index2){
    Point v12 = pList[index2] - pList[index1];  //ref vector
    for(int i = 0; i< pList.size()-1;i++){      //v12 no need cross with itself
        index2++;
        if(index2 == pList.size()-1){
            index2 = 0;
        }
        int index3 = index2;
        Point v13 = pList[index3] - pList[index1];
        int sCross = v12.x*v13.y - v12.y*v13.x;//v12 x v13
        if(sCross > 0){     //means the point abides in other side of v12 (anti-clockwise, yaxis opposite)
            return false;
        }
    }
    return true;
}

void initConvexPoints(ConvexPoints &convexPolygon,ContourPoints outerProfile){
    convexPolygon.IsInsideObstacle = true;
    int num = outerProfile.pointNum;
    int index = 0;
    for(int i=0; i<num; i++){
        if(outerProfile.IsConvextPoint[i]){
            convexPolygon.ptIndex[index] = i;
            convexPolygon.pt[index] = outerProfile.pt[i];
            index++;
        }
    }
    convexPolygon.pointNum = index;

    // solve distances
    int pointNum = convexPolygon.pointNum;
    Point temp;
    for(int i=0; i<pointNum-1; i++){
        temp = convexPolygon.pt[i+1]-convexPolygon.pt[i];
        convexPolygon.distance[i] = sqrt(temp.x*temp.x + temp.y*temp.y);
    }
    //when i=pointNum-1
    temp = convexPolygon.pt[0]- convexPolygon.pt[pointNum-1];
    convexPolygon.distance[pointNum-1] = sqrt(temp.x*temp.x + temp.y*temp.y);
}


int findNearestPointToGoal(ContourPoints& outerProfile, Point goal){
    Point temp;
    float distance;
    temp = outerProfile.pt[0]-goal;
    float nearestDistance = sqrt(temp.x*temp.x + temp.y*temp.y);
    int nearestIndex = 0;
    for(int i=1; i<outerProfile.pointNum; i++){
        temp = outerProfile.pt[i]-goal;
        distance = sqrt(temp.x*temp.x + temp.y*temp.y);
        if(distance < nearestDistance){
            nearestDistance = distance;
            nearestIndex = i;
        }
    }
    return nearestIndex;
}


bool isGoalInContour(ContourPoints& outerProfile,Point test){
    bool isInContour = false;
    for(int i=0,j=outerProfile.pointNum-1; i<outerProfile.pointNum;j=i++){//point order (k,k+1)
        float slopeInv = (outerProfile.pt[j].x-outerProfile.pt[i].x)/(outerProfile.pt[j].y-outerProfile.pt[i].y);
        if(((outerProfile.pt[i].y>test.y) != (outerProfile.pt[j].y>test.y)) &&
                (test.x < (slopeInv *(test.y-outerProfile.pt[i].y) + outerProfile.pt[i].x) ))
            isInContour = !isInContour;
    }
    return isInContour;

}

// judge of which side p3 is in segment p1p2
inline float direction(Point p1, Point p2, Point p3) {
    Point v1 = p2-p1;
    Point v2 = p3-p1;
    float crossProduct = v1.x * v2.y - v2.x*v1.y; //v1 x v2

    return crossProduct;
}

inline bool isSegmentsIntersect(Point p1, Point p2, Point p3, Point p4){
    float d1 = direction(p1, p2, p3);
    float d2 = direction(p1, p2, p4);
    float d3 = direction(p3, p4, p1);
    float d4 = direction(p3, p4, p2);
    if(d1*d2<0 && d3*d4<0)
        return true;
    else
        return false;
}

bool isGoalLineIntersectWithContour(ContourPoints& outerProfile, Point goalStart, Point goalEnd){// not including vertex
    bool isIntersect = false;
    for(int i=0, j=outerProfile.pointNum-1; i<outerProfile.pointNum; j=i++){
        isIntersect = isSegmentsIntersect(outerProfile.pt[j],outerProfile.pt[i],goalStart,goalEnd);
        if(isIntersect)
            return true;
    }
    return false;
}

int getIndexGoalWalkingOutGroove(ContourPoints& outerProfile,Point goal, Point goalNearestIndex){

}

inline int getAntiCloseWiseIndexIfConvex(ContourPoints& outerProfile, Point goal, int goalNearestIndex){
    // use last 3 points to detect the index  (now using all points)
    bool isIntersect = false;
    int endIndex = goalNearestIndex;
    int NUM = outerProfile.pointNum;

    for(int i=0; i<NUM; i++){
        endIndex++;
        for(int j=goalNearestIndex; j<endIndex;j++){
            isIntersect = isSegmentsIntersect(outerProfile.pt[j%NUM],outerProfile.pt[(j+1)%NUM],goal,outerProfile.pt[endIndex%NUM]);
            if(isIntersect)
                return (endIndex-1)%NUM;
        }
    }

    return goalNearestIndex;
}

inline int getCloseWiseIndexIfConvex(ContourPoints& outerProfile, Point goal, int goalNearestIndex){
    bool isIntersect = false;
    int endIndex = goalNearestIndex;
    int NUM = outerProfile.pointNum;

    for(int i=0; i<NUM; i++){
        endIndex--;
        for(int j=goalNearestIndex; j>endIndex;j--){
            isIntersect = isSegmentsIntersect(outerProfile.pt[(j+NUM)%NUM],outerProfile.pt[(j-1+NUM)%NUM],goal,outerProfile.pt[(endIndex+NUM)%NUM]);
            if(isIntersect)
                return (endIndex+NUM+1)%NUM;
        }
    }

    return goalNearestIndex;
}


inline int getAntiCloseWiseIndexIfGroove(ContourPoints& outerProfile, Point goal, int goalNearestIndex){
    int NUM = outerProfile.pointNum;
    int index = goalNearestIndex;

    for(int i=0; i<NUM; i++){
        int thisIndex = (index++)%NUM;
        if(outerProfile.IsConvextPoint[thisIndex]){
            return thisIndex;
        }
    }

    return goalNearestIndex;
}


inline int getCloseWiseIndexIfGroove(ContourPoints& outerProfile, Point goal, int goalNearestIndex){
    int NUM = outerProfile.pointNum;
    int index = goalNearestIndex;

    for(int i=0; i<NUM; i++){
        int thisIndex = (NUM+ (index--))%NUM;
        if(outerProfile.IsConvextPoint[thisIndex]){
            return thisIndex;
        }
    }

    return goalNearestIndex;
}

void judgePointInGroove(ContourPoints& outerProfile, PointInGroove& pointInGrooveA, Point goal, int goalNearestIndex){

    bool isConvex = outerProfile.IsConvextPoint[goalNearestIndex];
    cout<<"Nearest Point:"<<goalNearestIndex<<","<<isConvex<<endl;

    if(isConvex){ //if nearest point is convex
     // anti-closewise, from nearest point
        int indexAntiCloseWise = getAntiCloseWiseIndexIfConvex(outerProfile, goal, goalNearestIndex);
        cout<<"indexAntiCloseWise:"<<indexAntiCloseWise<<endl;
        int indexCloseWise = getCloseWiseIndexIfConvex(outerProfile, goal, goalNearestIndex);
        cout<<"indexCloseWise:"<<indexCloseWise<<endl;

        pointInGrooveA.ptFrontIndex = indexAntiCloseWise;
        pointInGrooveA.ptBehindIndex = indexCloseWise;

        if(pointInGrooveA.ptBehindIndex == pointInGrooveA.IsDirectFront){
            pointInGrooveA.IspointInGroove = true;
        }
        if(pointInGrooveA.IspointInGroove){
            pointInGrooveA.IsDirectFront  = true;
            pointInGrooveA.IsDirectBehind = true;
        }

    } else {// if nearest point is groove
        pointInGrooveA.IspointInGroove = true;

    // closewise, from nearest point
       int indexAntiCloseWise = getAntiCloseWiseIndexIfGroove(outerProfile, goal, goalNearestIndex);
       cout<<"indexAntiCloseWise:"<<indexAntiCloseWise<<endl;
       int indexCloseWise = getCloseWiseIndexIfGroove(outerProfile, goal, goalNearestIndex);
       cout<<"indexCloseWise:"<<indexCloseWise<<endl;

       pointInGrooveA.ptFrontIndex = indexAntiCloseWise;
       pointInGrooveA.ptBehindIndex = indexCloseWise;

       pointInGrooveA.IsDirectFront  = true;
       pointInGrooveA.IsDirectBehind = true;

       // 根据目标点和最近凸点的连线是否切割凹槽内的点；如果没有切割，则为可直连
    }
}

void findIndexInConvexFromContour(ConvexPoints& convexPolygon, int inVal, int& outVal){

    int NUM = convexPolygon.pointNum;
    for(int i=0; i<NUM; i++){
        if(inVal == convexPolygon.ptIndex[i]){
            outVal = i;
            break;
        }
    }
}

void chooseBetterPath(ConvexPoints& convexPolygon,PointInGroove& pointInGrooveStart,Point goalStart,
                      PointInGroove& pointInGrooveEnd,Point goalEnd, vector<Point>& betterPath){
    //compute shorter path and this is the better path
    int NUM = convexPolygon.pointNum;

    //1. anti-clockwise walk from start to end
    int ptFrontIndexStart = 0;
    int ptBehindIndexEnd = 0;
    findIndexInConvexFromContour(convexPolygon,pointInGrooveStart.ptFrontIndex, ptFrontIndexStart);
    findIndexInConvexFromContour(convexPolygon,pointInGrooveEnd.ptBehindIndex, ptBehindIndexEnd);
    vector<Point> pListAnticlockwise;
    int firstIndex;
    int secondIndex = ptFrontIndexStart;
    cout<<"secondIndex:"<<secondIndex<<endl;
    //pListAnticlockwise.push_back(firstIndex);
    pListAnticlockwise.push_back(goalStart);
    pListAnticlockwise.push_back(convexPolygon.pt[secondIndex]);
    Point temp = convexPolygon.pt[secondIndex] - goalStart;
    float distanceAntiClockWise = sqrt(temp.x*temp.x + temp.y*temp.y);

    for(int i=0; i< NUM; i++){
        if(secondIndex == ptBehindIndexEnd){
            break;
        }
        firstIndex = (ptFrontIndexStart + i + NUM)%NUM;
        secondIndex= (ptFrontIndexStart + i + NUM + 1)%NUM;
        pListAnticlockwise.push_back(convexPolygon.pt[secondIndex]);
        temp = convexPolygon.pt[secondIndex] - convexPolygon.pt[firstIndex];
        distanceAntiClockWise += sqrt(temp.x*temp.x + temp.y*temp.y);

    }

    firstIndex = ptBehindIndexEnd;
    pListAnticlockwise.push_back(goalEnd);
    temp = goalEnd - convexPolygon.pt[firstIndex];
    distanceAntiClockWise += sqrt(temp.x*temp.x + temp.y*temp.y);
    cout<<"AntiClockWise PATH:"<<pListAnticlockwise<<endl<<"distanceAntiClockWise:"<<distanceAntiClockWise<<endl;



    //2. clockwise walk from start to end
    int ptBehindIndexInConvexStart = 0;
    int ptFrontIndexEnd = 0;
    findIndexInConvexFromContour(convexPolygon,pointInGrooveStart.ptBehindIndex, ptBehindIndexInConvexStart);
    findIndexInConvexFromContour(convexPolygon,pointInGrooveEnd.ptFrontIndex, ptFrontIndexEnd);
    vector<Point> pListClockwise;
    firstIndex = 0;
    secondIndex = ptBehindIndexInConvexStart;
    cout<<"secondIndex:"<<secondIndex<<endl;
    //pListAnticlockwise.push_back(firstIndex);
    pListClockwise.push_back(goalStart);
    pListClockwise.push_back(convexPolygon.pt[secondIndex]);
    temp = convexPolygon.pt[secondIndex] - goalStart;
    float distanceClockWise = sqrt(temp.x*temp.x + temp.y*temp.y);

    for(int i=0; i< NUM; i++){
        if(secondIndex == ptFrontIndexEnd){
            break;
        }
        firstIndex = (ptBehindIndexInConvexStart - i + NUM)%NUM;
        secondIndex= (ptBehindIndexInConvexStart - i + NUM - 1)%NUM;
        pListClockwise.push_back(convexPolygon.pt[secondIndex]);
        temp = convexPolygon.pt[secondIndex] - convexPolygon.pt[firstIndex];
        distanceClockWise += sqrt(temp.x*temp.x + temp.y*temp.y);

    }

    firstIndex = ptFrontIndexEnd;
    pListClockwise.push_back(goalEnd);
    temp = goalEnd - convexPolygon.pt[firstIndex];
    distanceClockWise += sqrt(temp.x*temp.x + temp.y*temp.y);
    cout<<"ClockWise PATH:"<<pListClockwise<<endl<<"distanceClockWise:"<<distanceClockWise<<endl;


    //3. judge
    if(distanceAntiClockWise >= distanceClockWise){
        betterPath = pListClockwise;
    } else {
        betterPath = pListAnticlockwise;
    }
    cout<<"betterPath:"<<betterPath<<endl;

}
