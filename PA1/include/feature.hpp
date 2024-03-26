#ifndef FEATURE_HPP
#define FEATURE_HPP
#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>

class Feature{ //must be called once

private:
    cv::Ptr<cv::Feature2D> sift = cv::SIFT::create(100);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, true);
      // Brute-Force matcher create method

    std::vector<cv::DMatch> matches;
    // std::vector<cv::DMatch> good_matches;

public:    
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> computekpts(cv::Mat im){
        std::vector<cv::KeyPoint> kpts;
        cv::Mat desc;

        sift->detectAndCompute(im, cv::Mat(), kpts, desc);

        // std::cout << "kpts length: " << kpts.size() << std::endl;
        return {kpts, desc};
    }

    std::vector<cv::DMatch> goodmatch(cv::Mat desc1, cv::Mat desc2){
        matcher->match(desc1, desc2, matches);	// Find the best match for each descriptor from a query set.
        // Matcher_SIFT->match(ReferDescriptor, TargetDescriptor, matches);
        
        sort(matches.begin(), matches.end());

        // drawing only good matches (dist less than 2*min_dist)
        // good_matches = matches[50];
        std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin()+50);

        return good_matches;
    }


};


#endif