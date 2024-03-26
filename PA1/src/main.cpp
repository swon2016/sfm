#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "frame_list.hpp"


// Function to pad a number with leading zeros to make it four digits
std::string make_four_digit(int num) {
    if (num < 10) {
        return "000" + std::to_string(num);
    } else if (num < 100) {
        return "00" + std::to_string(num);
    } else if (num < 1000) {
        return "0" + std::to_string(num);
    } else {
        return std::to_string(num);
    }
}


int main() {
    std::cout<<"main start"<<std::endl;
    std::string path = "/root/PA1/data_PA1/data/Data";
    std::ifstream intrinsic(path + "/K.txt");

    std::string line;
    cv::Mat K(3, 3, CV_64F);
    int i=0;
    while(getline(intrinsic, line)){
        std::stringstream ss(line);

        double token;
        int j=0;
        while(ss>>token){
            K.at<double>(i, j) = token;
            j++;
        }
        ss.clear();
        i++;
    }
   std::string plyfilePath = "test.ply";

	// write File
	std::ofstream writeFile(plyfilePath.data());
	
    FrameList frame_list = FrameList(K);

    const int num_files = 32;
    for (int i = 0; i < num_files; i++) {
        std::cout<<"============" << i << "============"<<std::endl;
        std::string filename = path + "/" + make_four_digit(i) + ".JPG";
        cv::Mat im = cv::imread(filename, 0);
        if(im.empty()) std::cout << "IMG NOT LOADED" << std::endl;
        cv::Mat points4D;
        frame_list.insert_back(im, points4D); //compute and saves kpts, desc
        std::cout<<"insert back done"<<std::endl;

        std::cout<<points4D.size()<<std::endl;
        if( writeFile.is_open() && i>0){
            for(int i =0; i<=points4D.size().width;i++){
                writeFile<<points4D.at<float>(0,i)/points4D.at<float>(3,i)<<" "<<points4D.at<float>(1,i)/points4D.at<float>(3,i)<<" "<<points4D.at<float>(2,i)/points4D.at<float>(3,i)<<"\n";}
            
        }

    }
    writeFile.close();
    return 0;
}

