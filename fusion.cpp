#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include<iostream>
#include <string>
#include <thread>
#include <stdio.h>
#include <algorithm>

#include "fus_api.h"

int main()
{
    cv::Mat vis, ir, fus, h, irr;
    //2
    // std::vector<cv::Point2f> irPts{cv::Point2f(96,290), cv::Point2f(96,181), cv::Point2f(186,181), cv::Point2f(227,176), cv::Point2f(283,299), cv::Point2f(337,198), cv::Point2f(523,242)};
    // std::vector<cv::Point2f> visPts{cv::Point2f(550,538), cv::Point2f(550,413),  cv::Point2f(694,413), cv::Point2f(760,405), cv::Point2f(852,547), cv::Point2f(941,432), cv::Point2f(1239,479)};

    //3
    // std::vector<cv::Point2f> irPts{cv::Point2f(74,277), cv::Point2f(74,170), cv::Point2f(163,170), cv::Point2f(205,162), cv::Point2f(262,286), cv::Point2f(315,184), cv::Point2f(498,230)};
    // std::vector<cv::Point2f> visPts{cv::Point2f(303,513), cv::Point2f(303,325),  cv::Point2f(517,325), cv::Point2f(619,315), cv::Point2f(757,527), cv::Point2f(889,353), cv::Point2f(1333,427)};

    //4
    std::vector<cv::Point2f> irPts{cv::Point2f(78,280), cv::Point2f(78,170), cv::Point2f(170,170), cv::Point2f(209,165), cv::Point2f(266,288), cv::Point2f(319,187), cv::Point2f(504,231)};
    std::vector<cv::Point2f> visPts{cv::Point2f(103,503), cv::Point2f(103,257),  cv::Point2f(395,257), cv::Point2f(523,243), cv::Point2f(709,521), cv::Point2f(881,291), cv::Point2f(1473,387)};
    // std::vector<cv::Point2f> visPts{cv::Point2f(96+20,290), cv::Point2f(96+20,181), cv::Point2f(523+20,242), cv::Point2f(587+20,223)};
    h = findHomography(irPts,visPts);

    std::cout<<h<<std::endl;

    cv::Rect roi = cv::Rect(407,217,1003,561);

    
    vis = cv::imread("/home/zpwang/data/4/0.png");
    ir = cv::imread("/home/zpwang/data/4/1.png");
    // cv::imshow("1", ir);
    // cv::waitKey(0);

    cv::warpPerspective(ir, ir, h, cv::Size(1920,1080));
    // cv::warpPerspective(ir, ir, h, cv::Size(640,512));

    cv::imwrite("1rrrr.png", ir);
    // cv::Mat mask = ir;
    // ir.copyTo(vis, mask);
    // cv::imwrite("1.png", vis);
    // return 0;
    cv::addWeighted(vis(roi), 0.3, ir(roi), 0.7, 0, fus);
    fus.copyTo(vis(roi));

    cv::imwrite("1.png", vis);
    return 0;

    cv::VideoCapture camCap("../cam.mp4");
    cv::VideoCapture mapCap("../map.mp4");

    cv::Mat m_camImg, m_mapImg, m_fusionRet, m_final;

    FUS_init(m_camImg, m_mapImg);


    while(1)
    {
        camCap >> m_camImg;
        mapCap >> m_mapImg;
        auto rs = m_fusionRet;

        FUS_process(m_camImg, m_mapImg, rs);

        cv::imshow("1", rs);
        cv::waitKey(10);
    }

    return 0;
}


// bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints,const std::vector<cv::KeyPoint>& trainKeypoints,
// float reprojectionThreshold,std::vector<cv::DMatch>& matches,cv::Mat& homography)
// {
// 	const int minNumberMatchesAllowed = 8;
// 	if (matches.size() < minNumberMatchesAllowed)
// 		return false;
// 	// Prepare data for cv::findHomography  
// 	std::vector<cv::Point2f> srcPoints(matches.size());
// 	std::vector<cv::Point2f> dstPoints(matches.size());
// 	for (size_t i = 0; i < matches.size(); i++)
// 	{
// 		srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
// 		dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
// 		//srcPoints[i] = trainKeypoints[i].pt;
// 		//dstPoints[i] = queryKeypoints[i].pt;
// 	}
// 	// Find homography matrix and get inliers mask  
// 	std::vector<unsigned char> inliersMask(srcPoints.size());
// 	homography = cv::findHomography(srcPoints,dstPoints,CV_FM_RANSAC,reprojectionThreshold,inliersMask);
// 	std::vector<cv::DMatch> inliers;
// 	for (size_t i = 0; i<inliersMask.size(); i++)
// 	{
// 		if (inliersMask[i])
// 			inliers.push_back(matches[i]);
// 	}
// 	matches.swap(inliers);
// 	return matches.size() > minNumberMatchesAllowed;
// }

//  void drawLine(cv::Mat &img, //要标记直线的图像
//      std::vector<cv::Vec2f> lines,   //检测的直线数据
//      double rows,   //原图像的行数（高）
//      double cols,  //原图像的列数（宽）
//      cv::Scalar scalar,  //绘制直线的颜色
//      int n  //绘制直线的线宽
//  )
//  {
//      cv::Point pt1, pt2;
//      for (size_t i = 0; i < lines.size(); i++)
//      {
//          float rho = lines[i][0];  //直线距离坐标原点的距离
//          float theta = lines[i][1];  //直线过坐标原点垂线与x轴夹角
//          double a = cos(theta);  //夹角的余弦值
//          double b = sin(theta);  //夹角的正弦值
//          double x0 = a*rho, y0 = b*rho;  //直线与过坐标原点的垂线的交点
//          double length = std::max(rows, cols);  //图像高宽的最大值
//                                            //计算直线上的一点
//          pt1.x = cvRound(x0 + length  * (-b));
//          pt1.y = cvRound(y0 + length  * (a));
//          //计算直线上另一点
//          pt2.x = cvRound(x0 - length  * (-b));
//          pt2.y = cvRound(y0 - length  * (a));
//          //两点绘制一条直线
//          cv::line(img, pt1, pt2, scalar, n);
//      }
//  }

//  int addAlpha(cv::Mat& src, cv::Mat& dst)
// {
// 	if (src.channels() == 4)
// 	{
// 		return -1;
// 	}
// 	else if (src.channels() == 1)
// 	{
// 		cv::cvtColor(src, src, cv::COLOR_GRAY2RGB);
// 	}
	
// 	dst = cv::Mat(src.rows, src.cols, CV_8UC4);
 
// 	std::vector<cv::Mat> srcChannels;
// 	std::vector<cv::Mat> dstChannels;
// 	//分离通道
// 	cv::split(src, srcChannels);
 
// 	dstChannels.push_back(srcChannels[0]);
// 	dstChannels.push_back(srcChannels[1]);
// 	dstChannels.push_back(srcChannels[2]);

//     cv::Mat alpha = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
//     alpha.setTo(200);
// 	//添加透明度通道
// 	dstChannels.push_back(alpha);
// 	//合并通道
// 	cv::merge(dstChannels, dst);
 
// 	return 0;
// }

// int main()
// {
//     std::cout << cv::getBuildInformation() << std::endl;

//     cv::Mat camImg = cv::imread("/home/zpwang/data/hyxt/cam.png");
//     cv::Mat mapImg = cv::imread("/home/zpwang/data/hyxt/map2.png");

//     cv::Mat grayMapImage, grayCamImage, edgeCam, edgeMap, result_g, result_G;
//     cv::cvtColor(mapImg, grayMapImage, cv::COLOR_BGR2GRAY);
//     cv::cvtColor(camImg, grayCamImage, cv::COLOR_BGR2GRAY);

    
//     // cv::GaussianBlur(grayMapImage, grayMapImage, cv::Size(5, 5), 3, 3);
//     // cv::imwrite("grayMapImage.png", grayMapImage);
//     // cv::Canny(grayMapImage, edge, 300, 580, 5, false);
//     // cv::imwrite("canny-map.png", edge);
//     // cv::threshold(edge, edge, 200, 255, cv::THRESH_BINARY);
//     // // cv::blur(grayMapImage, edge, cv::Size(3, 3));
//     // // cv::imwrite("edge1.png", edge);
//     // // cv::Canny(edge, edge, 3, 9, 3);
//     // cv::imwrite("edge-map.png", edge);

//     // cv::GaussianBlur(grayCamImage, grayCamImage, cv::Size(5, 5), 3, 3);
//     // cv::imwrite("grayCamImage.png", grayCamImage);
//     // cv::Canny(grayCamImage, edge, 300, 580, 5, false);
//     // cv::imwrite("canny-cam.png", edge);
//     // cv::threshold(edge, edge, 200, 255, cv::THRESH_BINARY);
//     // // cv::blur(grayMapImage, edge, cv::Size(3, 3));
//     // // cv::imwrite("edge1.png", edge);
//     // // cv::Canny(edge, edge, 3, 9, 3);
//     // cv::imwrite("edge-cam.png", edge);

//     // cv::GaussianBlur(grayMapImage,result_g,cv::Size(3,3),5,0);//高斯滤波
//     // cv::Laplacian(result_g,result_G,CV_16S,3,1,0);
//     // cv::convertScaleAbs(result_G,result_G);
//     // cv::imwrite("result_G_map.png", result_G);

//     // cv::GaussianBlur(grayCamImage,result_g,cv::Size(3,3),5,0);//高斯滤波
//     // cv::Laplacian(result_g,result_G,CV_16S,3,1,0);
//     // cv::convertScaleAbs(result_G,result_G);
//     // cv::imwrite("result_G_cam.png", result_G);

//     //camImg = camImg(cv::Rect(214, 0, 1912, 1080));

//     cv::Mat tmp;
//     // addAlpha(camImg, tmp);

//     // cv::imwrite("aa.png", tmp);

//     // cv::resize(tmp, tmp, cv::Size(1280,720));

//     // cv::resize(camImg, camImg, cv::Size(1024,576));
//     // cv::resize(mapImg, mapImg, cv::Size(1280,720));

//     auto mapSz = mapImg.size();
//     cv::resize(camImg, camImg, mapSz);

//     cv::Mat ret;
//     cv::addWeighted(camImg, 0.5, mapImg, 0.5, 0, ret);
//     cv::imwrite("ret.png", ret);

//     // cv::cvtColor(mapImg, mapImg,  cv::COLOR_BGR2BGRA);
//     // tmp.copyTo(mapImg(cv::Rect(0,0,1280,720)));

//     // cv::imwrite("ret.png", mapImg);

//     // return 0;

//     cv::Mat img;

//     img = mapImg;
//     // img = camImg;

//     // cv::GaussianBlur(img, img, cv::Size(5, 5), 3, 3);
//     cv::Canny(camImg, edgeCam, 80, 180, 3, false);
//     cv::threshold(edgeCam, edgeCam, 170, 255, cv::THRESH_BINARY);

//     cv::Canny(mapImg, edgeMap, 80, 180, 3, false);
//     cv::threshold(edgeMap, edgeMap, 170, 255, cv::THRESH_BINARY);
 
//     //用不同的累加器进行检测直线
//     std::vector<cv::Vec2f> lines1, lines2;
//     // cv::HoughLines(edge, lines1, 1, CV_PI / 180, 50, 0, 0);
//     // cv::HoughLines(edge, lines2, 1, CV_PI / 180, 250, 0, 0);
 
//      //在原图像中绘制直线
//     cv::Mat img1, img2;
//     // img.copyTo(img1);
//     // img.copyTo(img2);
//     //  drawLine(img1, lines1, edge.rows, edge.cols, cv::Scalar(255), 2);
//     //  drawLine(img2, lines2, edge.rows, edge.cols, cv::Scalar(255), 2);
 
//     //  //显示图像
//     //  imshow("edge", edge);
//     //  imshow("img", img);
//     //  imshow("img1", img1);
//     //  imshow("img2", img2);
//     //  cv::waitKey(0);
//     //  return 0;

//       //利用渐进概率式霍夫变换提取直线
//      std::vector<cv::Vec4i> linesP1, linesP2;
//      cv::HoughLinesP(edgeMap, linesP1, 1, CV_PI / 180, 150, 270, 20);  //
//      cv::HoughLinesP(edgeCam, linesP2, 1, CV_PI / 180, 150, 250, 20);  //cam
 
//      //绘制两个点连接最大距离10直线检测结果
//     //  cv::Mat img1;
//      mapImg.copyTo(img1);
//      for (size_t i = 0; i < linesP1.size(); i++)
//      {
//          cv::line(img1, cv::Point(linesP1[i][0], linesP1[i][1]),
//              cv::Point(linesP1[i][2], linesP1[i][3]), cv::Scalar(255), 3);
//      }
 
//      //绘制两个点连接最大距离30直线检测结果
//     //  cv::Mat img2;
//      camImg.copyTo(img2);
//      for (size_t i = 0; i < linesP2.size(); i++)
//      {
//          cv::line(img2, cv::Point(linesP2[i][0], linesP2[i][1]),
//              cv::Point(linesP2[i][2], linesP2[i][3]), cv::Scalar(255), 3);
//      }
 
//      //显示图像
//     //  cv::imwrite("edge.png", edge);
//      cv::imwrite("mapLine.png", img1);
//      cv::imwrite("camLine.png", img2);
//     //  cv::imshow("img1", img1);
//     //  cv::imshow("img2", img2);
//     //  cv::waitKey(0);
//      return 0;
 

//     // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
//     // cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
//     // std::vector<cv::KeyPoint>  kps1, kps2;
//     // cv::Mat desc1, desc2;
//     // sift->detectAndCompute(camImg, cv::Mat(), kps1, desc1);
//     // sift->detectAndCompute(mapImg, cv::Mat(), kps2, desc2);

//     // cv::Ptr<cv::BFMatcher> bfmatcher = cv::BFMatcher::create(cv::NORM_L2, true);
//     // std::vector<cv::DMatch> matches;
//     // bfmatcher->match(desc1, desc2, matches);

//     // cv::Mat homo;
//     // refineMatchesWithHomography(kps1, kps2, 0.5, matches, homo);

//     // // 4) draw and show
//     // cv::Mat img_matches;
//     // cv::drawMatches(camImg, kps1, mapImg, kps2, matches, img_matches);
//     // cv::imshow("BFMatcher", img_matches);

//     // cv::waitKey();

    

//     // cv::Mat ret;
//     // cv::addWeighted(camImg, 0.5, mapImg, 0.5, 0, ret);

//     // cv::imwrite("1.png", camImg);
//     // cv::imwrite("2.png", mapImg);
//     // cv::imwrite("3.png", ret);


//     // cv::imshow("1", ret);
//     // cv::waitKey(0);
// }