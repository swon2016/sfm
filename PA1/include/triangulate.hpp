#ifndef TRIANGULATE_HPP
#define TRIANGULATE_HPP
#include "frame_list.hpp"

class Triangulate{
private:
public:
    // void traingulate(cv::Mat& Rt1, cv::Mat& Rt2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2){
    //     // std::vector<cv::Point3f> = 
    //     cv::Mat Hom4D;
    //     cv::triangulatePoints(Rt1, Rt2, pts1, pts2, Hom4D);
    // }

    static void triangulate( cv::Mat& matr1, cv::Mat& matr2,
                                const cv::Mat& points1, const cv::Mat& points2,
                                cv::Mat& points_4D )
    {
        // cv::Mat points1_ = points1.t();
        // cv::Mat points2_ = points2.t();  //should be 2xn size
        cv::Mat points1_ = points1;
        cv::Mat points2_ = points2;  //should be 2xn size

        // std::cout<<"points row "<<points1_.rows<<" col "<<points1_.cols<<std::endl;
        // std::cout<<"points4D "<<points_4D<<std::endl;

        points_4D.create(4, points1_.cols, CV_64F);
        cv::Mat* projMatr1 = &matr1;
        cv::Mat* projMatr2 = &matr2;
        cv::Mat* projPoints1 = &points1_;
        cv::Mat* projPoints2 = &points2_;
        cv::Mat* points4D = &points_4D;

        if( projMatr1 == 0 || projMatr2 == 0 || projPoints1 == 0 || projPoints2 == 0 || points4D == 0) std::cout<<"error"<<std::endl;

        int numPoints = projPoints1->cols;

        if( numPoints < 1 ) std::cout<<"Number of points must be more than zero"<<std::endl;
        if( projPoints2->cols != numPoints || points4D->cols != numPoints ) std::cout<<"Number of points must be the same"<<std::endl;
        if( projPoints1->rows != 2 || projPoints2->rows != 2)std::cout<<"Number of proj points coordinates must be == 2"<<std::endl;
        if( points4D->rows != 4 )std::cout<<"Number of world points coordinates must be == 4"<<std::endl;
        if( projMatr1->cols != 4 || projMatr1->rows != 3 || projMatr2->cols != 4 || projMatr2->rows != 3) std::cout<<"Size of projection matrices must be 3x4"<<std::endl;
       
        // preallocate SVD matrices on stack
        cv::Matx<double, 4, 4> matrA;
        cv::Matx<double, 4, 4> matrU;
        cv::Matx<double, 4, 1> matrW;
        cv::Matx<double, 4, 4> matrV;

        cv::Mat* projPoints[2] = {projPoints1, projPoints2};
        cv::Mat* projMatrs[2] = {projMatr1, projMatr2};

        /* Solve system for each point */
        for( int i = 0; i < numPoints; i++ )/* For each point */
        {
            /* Fill matrix for current point */
            for( int j = 0; j < 2; j++ )/* For each view */
            {
                double x,y;
                x = (*projPoints[j]).at<double>(0,i);
                y = (*projPoints[j]).at<double>(1,i);
                for( int k = 0; k < 4; k++ )
                {
                    matrA(j*2+0, k) = x * (*projMatrs[j]).at<double>(2,k) - (*projMatrs[j]).at<double>(0,k);
                    matrA(j*2+1, k) = y * (*projMatrs[j]).at<double>(2,k) - (*projMatrs[j]).at<double>(1,k);
                }
            }
            // Solve system for ith point
            cv::SVD::compute(matrA, matrW, matrU, matrV);
            
            (*points4D).at<double>(0,i)=matrV(3,0);/* X */
            (*points4D).at<double>(1,i)=matrV(3,1);/* Y */
            (*points4D).at<double>(2,i)=matrV(3,2);/* Z */
            (*points4D).at<double>(3,i)=matrV(3,3);/* W */
        }
    }



};

// TODO: BA .. maybe for photometry loss and reprojection error



#endif