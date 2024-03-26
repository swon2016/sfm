#ifndef FRAME_LIST_HPP
#define FRAME_LIST_HPP
#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <opencv2/opencv.hpp>
#include <vector>
#include "feature.hpp"
#include "relative_pose.hpp"

class Frame{
private:
    int frameid;
    Frame* prev;
    Frame* next;

    // cv::Mat tracked_desc;
    // std::vector<cv::KeyPoint> tracked_kpts;

public:

    cv::Mat E = cv::Mat::zeros(3,3, CV_64F);
    cv::Mat R = cv::Mat::eye(3,3, CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1, CV_64F);

    // cv::Mat glob_R = cv::Mat::zeros(3,3, CV_32F);
    // cv::Mat glob_t = cv::Mat::zeros(3,1, CV_32F);

    cv::Mat desc;
    std::vector<cv::KeyPoint> kpts;
    std::vector<cv::DMatch> match_to_prev;
    std::vector<cv::Point2f> tracked_kpts_to_prev;
    // cv::Mat points4D;

    Frame(){
        this->prev = NULL;
        this->next = NULL;
        //NULL for head and tail
    }
    
    Frame(std::vector<cv::KeyPoint> kpts, cv::Mat desc){ 
        this->kpts = kpts;
        this->desc = desc;
    }
    Frame* getprev(){ return prev; }
    Frame* getnext(){ return next; }
    void setprev(Frame* prev_fr){ this->prev = prev_fr;}
    void setnext(Frame* next_fr){ this->next = next_fr;}
    void setmatch(std::vector<cv::DMatch> match){this->match_to_prev = match;}
};

class FrameList{ //must be called once
private:
    Frame* head;
    Frame* tail;
    Feature* feature;
    Pose* pose;

    int cnt;
    // cv::Mat K;
public:
    FrameList(cv::Mat K){
        head = new Frame;
        tail = new Frame;    //pointer to pointer
        head->setnext(tail);
        tail->setprev(head);

        cnt = 0;
        // this->K = K;

        feature = new Feature;
        pose = new Pose(K);
    }

    void insert_back(cv::Mat im, cv::Mat& points4D){
        std::cout<<"insert back start"<<std::endl;

        std::pair<std::vector<cv::KeyPoint>, cv::Mat> extraction = feature->computekpts(im);
        Frame* frame = new Frame(extraction.first, extraction.second);
        
        Frame* oldlast = tail->getprev();
        frame->setprev(oldlast);
        frame->setnext(tail);
        tail->setprev(frame);
        oldlast->setnext(frame);
        cnt += 1;
        //connected lists

        std::cout<<"frame added to list"<<std::endl;

        if(frame->getprev() != head ){
            frame->setmatch(feature->goodmatch(frame->desc, frame->getprev()->desc));
            std::cout<<"set match done"<<std::endl;

            std::pair<cv::Mat, cv::Mat> posepair = pose->getpose(frame->kpts, frame->getprev()->kpts, frame->match_to_prev, im.size(), points4D);
            std::cout<<"getpose done"<<std::endl;
            cv::Mat tmp_R = posepair.first;
            cv::Mat tmp_t = posepair.second;
            // std::cout<<"11111"<<std::endl;
            
            frame->t = frame->getprev()->t + frame->getprev()->R * tmp_t;
            frame->R = tmp_R * frame->getprev()->R;
            std::cout<<"2222222"<<std::endl;
            // std::cout<<frame->R<<std::endl;
        }
    }

};



#endif //FRAME_LIST.HPP
