#ifndef RANSAC_E_HPP
#define RANSAC_E_HPP

#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include "frame_list.hpp"
#include "triangulate.hpp"
#include <opencv2/core.hpp>
#include <cstdlib> //std::rand(), std::srand()

// #include "feature.hpp"
// #include <opencv2/core.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d.hpp>
// #include <opencv2/calib3d.hpp>

class Frame;

class Pose{
private:
    cv::Mat E;
    cv::Mat R;
    cv::Mat t;

    cv::Mat K;
    cv::Point2f getpp(){
        cv::Point2d pp(K.at<float>(0,2), K.at<float>(1,2));
        return pp;
    }
    float getfocal(){
        return (K.at<float>(0,0)+K.at<float>(1,1)) / 2.0;
    }
    // cv::Mat& getK(){
    //     return K;
    // }

public:
    Pose(cv::Mat K){ this->K = K; }

    std::pair<cv::Mat, cv::Mat> getpose(std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2, std::vector<cv::DMatch> match, cv::Size size, cv::Mat& points4D){
       //===================1======================
        // std::vector<cv::Point2f> pts1, pts2;

        // for(int i = 0; i < match.size(); i++){
        //     pts1.push_back(kpts1[match[i].queryIdx].pt);
        //     pts2.push_back(kpts2[match[i].trainIdx].pt);
        // }
        //===================1======================

        //===================2======================
        std::vector<double> pts1, pts2;

        for(int i = 0; i < match.size(); i++){
            pts1.push_back(kpts1[match[i].queryIdx].pt.x);
            pts1.push_back(kpts1[match[i].queryIdx].pt.y);
            
            pts2.push_back(kpts2[match[i].trainIdx].pt.x);
            pts2.push_back(kpts2[match[i].trainIdx].pt.y);
        }
        //===================2====================== 

        std::cout<<"pts len: " << pts1.size() << " " << pts2.size() <<std::endl;
        // std::cout<<pts1[0].x<<" "<<pts1[0].y<<" "<<pts2[0].x<<" "<<pts2[0].y<<" "<<std::endl;

        // E = cv::findEssentialMat(pts2, pts1, getfocal(), getpp(), cv::RANSAC, 0.999, 0.1);
        
        double threshold = 0.1;  //mimic conventional.. parameter
        threshold /= (K.at<double>(0,0)+K.at<double>(1,1))/2;
        E = this->ransac_E(pts2, pts1, threshold);
        std::cout<<"ransac_e done E: "<<E<<std::endl;

        recoverpose(E, pts2, pts1, this->R, this->t, points4D);
        // cv::recoverPose(this->E, pts2, pts1, this->R, this->t, getfocal(), getpp());
        
        return {R, t};
    }

    cv::Mat ransac_E(std::vector<double>& pts1, std::vector<double>& pts2, int threshold){
        cv::Mat points1, points2;
        cv::Mat(pts1).convertTo(points1, CV_64F);
        cv::Mat(pts2).convertTo(points2, CV_64F);
        
        points1 = points1.reshape(1, pts1.size()/2);
        points2 = points2.reshape(1, pts2.size()/2);        
        std::cout<<"pts1: "<<points1.rows<<" "<<points1.cols<<std::endl;
        std::cout<<"pts2: "<<points2.rows<<" "<<points2.cols<<std::endl;
 
        // now form nx2 matrix

        double fx = K.at<double>(0,0);
        double fy = K.at<double>(1,1);
        double cx = K.at<double>(0,2);
        double cy = K.at<double>(1,2);
        
        // std::cout<<points1.col(0)<<std::endl; //compare
        points1.col(0) = (points1.col(0) - cx) / fx;   //for x
        points1.col(1) = (points1.col(1) - cy) / fy;   //for y
        points2.col(0) = (points2.col(0) - cx) / fx;   //for x
        points2.col(1) = (points2.col(1) - cy) / fy;   //for y
        // now points are camera centered rays! 
        // points for epipolar constraints

        cv::Mat E = ransac(points1, points2, 5, threshold);

        return E;
        // std::cout<<points1.col(0)<<std::endl; //compare
    }

    cv::Mat fivepoint(const cv::Mat& points1, const cv::Mat& points2){
        int n = points1.rows;
        // std::cout<<n<<std::endl;
        // std::cout<<"pts1: "<<points1.rows<<" "<<points1.cols<<std::endl;
        // std::cout<<"pts2: "<<points2.rows<<" "<<points2.cols<<std::endl;

        //========================================
        // cv::Mat B(3, 3, CV_64F);
        // B.at<double>(0,0) = 2 ;
        // B.at<double>(0,1) = -1 ;
        // B.at<double>(0,2) = 1 ;
        // B.at<double>(1,0) = 1 ;
        // B.at<double>(1,1) = 1 ;
        // B.at<double>(1,2) = 1 ;
        // B.at<double>(2,0) = 4 ;
        // B.at<double>(2,1) = -2 ;
        // B.at<double>(2,2) = 2 ;

        // cv::Mat U, W, Vt;
        // cv::SVD::compute(B, W, U, Vt, cv::SVD::FULL_UV);
        
        // std::cout<<"B, U, W, Vt size: "<<B.size()<<" "<<U.size()<<" "<<W.size()<<" "<<Vt.size()<<" "<<std::endl;
        // std::cout<<"B, U, W, Vt: "<<B<<std::endl<<U<<std::endl<<W<<std::endl<<Vt<<std::endl;
        // std::cout<<"======================================================================================"<<std::endl;

        //========================================



        cv::Mat A(n, 9, CV_64F);
        //AE = 0 A: nx9 matrix
        //form A (u, v) and (u', v') pairs
        
        A.col(0) = points1.col(0).mul( points2.col(0) );  //uu'
        A.col(1) = points1.col(1).mul( points2.col(0) );  //vu'
        A.col(2) = points2.col(0) * 1.0;                  //u'
        A.col(3) = points1.col(0).mul( points2.col(1) );  //uv'
        A.col(4) = points1.col(1).mul( points2.col(1) );  //vv'
        A.col(5) = points2.col(1) * 1.0;                  //v'
        A.col(6) = points1.col(0) * 1.0;                  //u
        A.col(7) = points1.col(1) * 1.0;                  //v
        A.col(8) = 1.0;                                   //1
        std::cout<<"A size: "<<A.size()<<" "<<std::endl;
        std::cout<<"A row, col: "<<A.rows<<" "<<A.cols<<" "<<std::endl;

        cv::Mat U, W, Vt;
        cv::SVD::compute(A, W, U, Vt, cv::SVD::FULL_UV);
        // std::cout<<"A, U, W, Vt size: "<<A.size()<<" "<<U.rows<<U.cols<<" "<<W.size()<<" "<<Vt.size()<<" "<<std::endl;
        // std::cout<<"W is "<<W.size()<<" "<<std::endl;
        // std::cout<<"got Vt: "<<Vt.size()<<std::endl;

        cv::Mat V = cv::Mat(Vt.t()); //make V beacause we have V^t = V transpose
        // std::cout<<"got V: "<<cv::Mat(V.col(8)).size()<<std::endl;
        cv::Mat E = cv::Mat(V.col(8)).clone().reshape(1, 3);     //make E out out last column of V
       
        // force rank 2
        cv::SVD::compute(E, W, U, Vt, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);
        // std::cout<<"E, U, W, Vt size: "<<E.size()<<" "<<U.size()<<" "<<W.size()<<" "<<Vt.size()<<" "<<std::endl;

        //rank = 2 constraint
        cv::Mat W_ = cv::Mat::zeros(3, 3, CV_64F);
        W_.at<double>(0,0) = W.at<double>(0,0);
        W_.at<double>(1,1) = W.at<double>(1,1);
        W_.at<double>(2,2) = 0;
        E = U*W_*Vt;
        
        return E;
    }

    cv::Mat ransac(cv::Mat& pts1, cv::Mat& pts2, int modelparam, int threshold){
        // std::cout<<"pts1: "<<pts1.rows<<" "<<pts1.cols<<std::endl;
        // std::cout<<"pts2: "<<pts2.rows<<" "<<pts2.cols<<std::endl;

        cv::Mat best_E;

        if(pts1.size() != pts2.size()) std::cout<< "Wrong point pairs estimating EMat" << std::endl;
        int num_pair;
        if(pts1.cols == 2) 
            num_pair = pts1.rows;
        
        if(modelparam == num_pair){ //if we got 5 point pairs, just make E
            best_E = fivepoint(pts1, pts2);
        }
        else{  //if we have many point pairs, repeat( take 5 pair -> make E -> test how well it fits for all point pairs)
            //random sampling here

            cv::Mat sample_points1;
            cv::Mat sample_points2;
            cv::Mat E;
            int best_cnt = 0;
            
            // std::vector<int> inliercount;
            for(int iter = 0; iter<1000; iter++){
                getsamples(pts1, pts2, sample_points1, sample_points2, modelparam);
                std::cout<<"got samples, iter: "<<iter<<std::endl;

                E = fivepoint(sample_points1, sample_points2);

                //compute how may inliers here
                int inlier_cnt = findinlier(pts1, pts2, E, threshold);
                std::cout<<"inlier of this E: "<<inlier_cnt<<std::endl;

                if( inlier_cnt >= best_cnt ){
                    best_E = E.clone();
                    best_cnt = inlier_cnt;
                }
            }
        }
        return best_E;
    }

    void getsamples(const cv::Mat& pts1, const cv::Mat& pts2, cv::Mat& sample_pts1, cv::Mat& sample_pts2, int num_sample){
        srand((unsigned int)time(NULL)); //seed값으로 현재시간 부여 
        int max_idx = pts1.rows;
        std::cout<<"get sample points with max idx: "<<max_idx<<std::endl;


        // int idxs[num_sample];
        int rand_idx;

        // const int *m1ptr = pts1.ptr<int>();
        // const int *m2ptr = pts2.ptr<int>();

        sample_pts1.create(num_sample, 2, CV_64F);
        sample_pts2.create(num_sample, 2, CV_64F); //5x2 matrix

        // int *sp1ptr = sample_pts1.ptr<int>();
        // int *sp2ptr = sample_pts2.ptr<int>();

        // 난수 % n = 난수의 범위 0~(n-1)
        for (int i = 0; i < num_sample; i++){
            // idxs[i] = rand() % max_idx;
            rand_idx = rand() % max_idx;
            std::cout<<"rand_idx: "<<rand_idx<<std::endl;
            sample_pts1.row(i) = pts1.row(rand_idx);
            sample_pts2.row(i) = pts2.row(rand_idx);
        }

    }

    int findinlier(const cv::Mat& pts1, const cv::Mat& pts2, const cv::Mat& model, double threshold){
        // pts nx2 matrix. 

        cv::Matx33d E(model);
        double err;  //using epipolar constraint

        int cnt_inlier = 0;
        double tot_err = 0;
        
        for(int i = 0; i < pts1.rows ; i++ ){
            // cv::Vec3f p1(pts1.at<float>(i,1))
            cv::Vec3d p1(pts1.at<float>(i,0), pts1.at<float>(i,1), 1);
            cv::Vec3d p2(pts2.at<float>(i,0), pts2.at<float>(i,1), 1);
            cv::Vec3d Ep1 = E*p1; 
            cv::Vec3d Etp2 = E.t()*p2;

            double p2tEp1 = p2.dot(Ep1); // should be 0 idealy

            err = (double)(p2tEp1 / (Ep1[0]*Ep1[0] + Ep1[1]*Ep1[1] + Etp2[0]*Etp2[0] + Etp2[1]*Etp2[1]));
            
            if(err <= threshold){
                cnt_inlier +=1;
                tot_err+=err;
            } 
        }

        return cnt_inlier;
    }



    void decomposeE( cv::Mat& E, cv::Mat& out_R1, cv::Mat& out_R2, cv::Mat& out_t )
    {
        cv::Mat D, U, Vt;
        cv::SVD::compute(E, D, U, Vt);

        if (determinant(U) < 0) U *= -1.;
        if (determinant(Vt) < 0) Vt *= -1.;

        cv::Mat W = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        W.convertTo(W, E.type());
        std::cout<<"0000000000000000"<<std::endl;

        cv::Mat R1, R2, t;
        R1 = U * W * Vt;
        R2 = U * W.t() * Vt;
        t = U.col(2) * 1.0;
        std::cout<<"3333333333333333333"<<std::endl;

        R1.copyTo(out_R1);
        R2.copyTo(out_R2);
        t.copyTo(out_t);
    }

    int recoverpose(cv::Mat E, std::vector<double>& pts1, std::vector<double>& pts2,  //input pts are inpixel coords
                    cv::Mat& out_R, cv::Mat& out_t, cv::Mat& out_triangulatedPoints, double distanceThresh = 50)
    {   
        std::cout<<"recover pose start"<<std::endl;

        cv::Mat points1, points2;
        cv::Mat(pts1).convertTo(points1, CV_64F);
        cv::Mat(pts2).convertTo(points2, CV_64F);
        
        points1 = points1.reshape(1, pts1.size()/2);
        points2 = points2.reshape(1, pts2.size()/2);        
        // now form nx2 matrix

        std::cout<<"pts1: "<<points1.rows<<" "<<points1.cols<<std::endl;
        std::cout<<"pts2: "<<points2.rows<<" "<<points2.cols<<std::endl;
        int npoints = points1.rows;

        
        double fx = K.at<double>(0,0);
        double fy = K.at<double>(1,1);
        double cx = K.at<double>(0,2);
        double cy = K.at<double>(1,2);
        
        // std::cout<<points1.col(0)<<std::endl; //compare
        points1.col(0) = (points1.col(0) - cx) / fx;   //for x
        points1.col(1) = (points1.col(1) - cy) / fy;   //for y
        points2.col(0) = (points2.col(0) - cx) / fx;   //for x
        points2.col(1) = (points2.col(1) - cy) / fy;   //for y
        // now points are camera centered rays! 

        points1 = points1.t();
        points2 = points2.t();

        cv::Mat R1, R2, t;
        decomposeE(E, R1, R2, t);
        // std::cout<<"11111111111111111"<<std::endl;

        cv::Mat P0 = cv::Mat::eye(3, 4, R1.type());
        cv::Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
        P1.colRange(0,3) = R1 * 1.0; P1.col(3) = t * 1.0;
        P2.colRange(0,3) = R2 * 1.0; P2.col(3) = t * 1.0;
        P3.colRange(0,3) = R1 * 1.0; P3.col(3) = -t * 1.0;
        P4.colRange(0,3) = R2 * 1.0; P4.col(3) = -t * 1.0;
        // std::cout<<"2222222222222222"<<std::endl;

        // Do the cheirality check.
        std::vector<cv::Mat> allTriangulations(4);
        cv::Mat Q;

        Triangulate::triangulate(P0, P1, points1, points2, Q);
        Q.copyTo(allTriangulations[0]);
        cv::Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask1 = (Q.row(2) < distanceThresh) & mask1;
        Q = P1 * Q;
        mask1 = (Q.row(2) > 0) & mask1;
        mask1 = (Q.row(2) < distanceThresh) & mask1;

        Triangulate::triangulate(P0, P2, points1, points2, Q);
        Q.copyTo(allTriangulations[1]);
        cv::Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask2 = (Q.row(2) < distanceThresh) & mask2;
        Q = P2 * Q;
        mask2 = (Q.row(2) > 0) & mask2;
        mask2 = (Q.row(2) < distanceThresh) & mask2;

        Triangulate::triangulate(P0, P3, points1, points2, Q);
        Q.copyTo(allTriangulations[2]);
        cv::Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask3 = (Q.row(2) < distanceThresh) & mask3;
        Q = P3 * Q;
        mask3 = (Q.row(2) > 0) & mask3;
        mask3 = (Q.row(2) < distanceThresh) & mask3;

        Triangulate::triangulate(P0, P4, points1, points2, Q);
        Q.copyTo(allTriangulations[3]);
        cv::Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask4 = (Q.row(2) < distanceThresh) & mask4;
        Q = P4 * Q;
        mask4 = (Q.row(2) > 0) & mask4;
        mask4 = (Q.row(2) < distanceThresh) & mask4;

        mask1 = mask1.t();
        mask2 = mask2.t();
        mask3 = mask3.t();
        mask4 = mask4.t();
        
        out_R.create(3, 3, R1.type());
        out_t.create(3, 1, t.type());

        int good1 = countNonZero(mask1);
        int good2 = countNonZero(mask2);
        int good3 = countNonZero(mask3);
        int good4 = countNonZero(mask4);
        std::cout<<"goodcount"<<good1<<good2<<good3<<good4<<std::endl;

        if (good1 >= good2 && good1 >= good3 && good1 >= good4)
        {
            allTriangulations[0].copyTo(out_triangulatedPoints);
            R1.copyTo(out_R);
            t.copyTo(out_t);

            return good1;
        }
        else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
        {
            allTriangulations[1].copyTo(out_triangulatedPoints);
            R2.copyTo(out_R);
            t.copyTo(out_t);
            
            return good2;
        }
        else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
        {
            allTriangulations[2].copyTo(out_triangulatedPoints);
            t = -t;
            R1.copyTo(out_R);
            t.copyTo(out_t);
            
            return good3;
        }
        else
        {
            allTriangulations[3].copyTo(out_triangulatedPoints);
            t = -t;
            R2.copyTo(out_R);
            t.copyTo(out_t);
            
            return good4;
        }
    }
};


#endif