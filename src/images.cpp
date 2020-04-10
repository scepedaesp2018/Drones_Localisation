#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>

#define pi 3.14159f
#define PERPEDICULAR_MARGIN 30  // min angle between lines for taking their crossing points
#define H_FILTER_CONST 0.2f     // lines selection: Algorithm takes [0.5-H_FILTER_CONST,0.5+H_FILTER_CONST]*lines.size();
#define BLUR_FILTER_SIZE 13     // Kernel size filter
#define ITER_FILT 3             // number of Gaussian filter repetition
#define PIXELS_PER_LINE 50      // how much pixels must have a detected right line

using namespace cv;

// Canny Filter Parameters
const int lowThreshold = 50;
const int MaxlowThreshold = 100;
// Information about the images localisation
const std::vector<std::string> folders = {"up","front","down"};
const std::vector<std::vector<int>> identifiants = {{2,3,4,5,10},{1,6,7,9,12,16,17,18},{8,11,13,19,20,21}};
//Frame size of video
int H = 42; //inicialized in a random value
int W = 42;

// Processing = Gaussian filtering + Canny edge detector
Mat process(Mat & src){
  Mat resized, detected_edges;
  cvtColor( src, src, COLOR_BGR2GRAY );
  resize(src,src,Size(0.2*src.cols,0.2*src.rows));
  resized=src.clone();
  for(int i = 0 ; i < ITER_FILT ; i++)
    GaussianBlur( resized, resized, Size(BLUR_FILTER_SIZE,BLUR_FILTER_SIZE),0,0);
  Canny( resized, detected_edges, lowThreshold, MaxlowThreshold, 3 );
return detected_edges;
}

/* Print : original image + edge detection result + lines detected + drone trancking point stimated
        src : Original image
        dst : Processed image
        lines : vector of detected lines
        points : vector of detected points
*/
void print_image(Mat & src, Mat & dst, std::vector<Vec2f> & lines, std::vector<Point> & points, std::string name){
  // every line is a vector of his values in Polar representation

  for( size_t i = 0; i < lines.size(); i++ )
  {
      float rho = lines[i][0], theta = lines[i][1];
      Point pt1, pt2;
      // Polar to cartesian conversion
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      // Getting point image: this method is taken from OpenCV exemples. dont change "1000"
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line(dst, pt1, pt2, Scalar(255,255,255), 1, LINE_AA); //OpenCV plot basic plot line fonction in image
  }
  Mat salida;
  hconcat(src,dst,salida); // hconcat : horiontal concatenation of 2 images
  // ploting  green points
  cvtColor(salida, salida, COLOR_GRAY2BGR);
  for(auto & p : points){
    circle(salida, {p.x, p.y} , 5, {0,255,0}, 3);
  }
  imshow(name,salida);
}

// Read an image from file name
Mat read_img(std::string name){
  Mat src;
  src = imread(name, IMREAD_COLOR ); // Load an image
  if( src.empty())
  {
    std::cout << "Could not open or find the image!\n" << std::endl;
  }
  return src;
}

// Return the minimal angle between 2 lines.
// a, b slope of the lines in rad // look theta meaning in HoughLines Documentation
float is_perpendicular(float a , float b){
  return min(abs(a-b),pi-abs(a-b));
}

// Find the crossing point of to lines in polar representation (rho,theta)
Point find_crossing(Vec2f & ap, Vec2f & bp){
  // Polar to cartesian line representation y = m x + b
  float m1,m2,b1,b2;
  Point crossing;
  m1 = -cos(ap[1])/sin(ap[1]);
  m2 = -cos(bp[1])/sin(bp[1]);
  b1 = ap[0]/sin(ap[1]);
  b2 = bp[0]/sin(bp[1]);
  // crossing point of two cartesian lines
  crossing.x = (b2-b1)/(m1-m2);
  crossing.y = m2*crossing.x + b2;
  return crossing;
}

// finds the crossing point between a line and a vector of lines
std::vector<Point> find_points(Vec2f & middle, std::vector<Vec2f> & lines){
  std::vector<Point> crossing;
  for (auto & l : lines){
    Point cruce = find_crossing(middle, l);
    crossing.push_back(cruce);
  }
  return crossing;
}

// Return the nearest point to the center of the image
std::vector<Point> cruces_filter(std::vector<Point> & cr){
    std::sort(cr.begin(), cr.end(), [](const Point & a, const Point & b) {return ((pow((H-a.y),2)+pow((W-a.x),2)) < (pow((H-b.y),2)+pow((W-b.x),2))); });
    std::vector<Point> cruces;
    cruces.push_back(cr[0]);
    return cruces;
}

int main( int argc, char** argv )
{

  Mat src,dst;

  for ( int f = 1 ; f < 3 ; f++ ){
    for(auto & num : identifiants[f]){
      // Looking for the image
      std::string name = ("../img/cutted/" + folders[f] + "/drons" + std::to_string(num) + ".jpg");
      src = imread(name);
      // Processing the image
      dst = process(src);
      //actualice its size global values
      H = dst.rows;
      W = dst.cols;

      std::vector<Vec2f> lines; // will hold the results of the lines detection
      HoughLines(dst, lines, 1, CV_PI/180, PIXELS_PER_LINE, 0, 0 );

      // getting the middle inclination value
      std::sort(lines.begin(), lines.end(), [](const Vec2f & a, const Vec2f & b) {return a[1] < b[1]; });
      if(lines.size()>0){
        Vec2f middle = lines[floor(lines.size()/2)];

        // looking for a perpedicular lines:
        bool cross = false;
        std::vector<Vec2f> perpendiculars; // if cross, look for a crossing
        std::vector<Vec2f> horizontals; // if not get the medians center-line

        for (auto & l : lines){
          float aux = is_perpendicular(middle[1], l[1]);
          if (aux > PERPEDICULAR_MARGIN*(pi/180)){
            cross = true;
            perpendiculars.push_back(l);
          }
        }

        // newl used for plotting
        std::vector<Vec2f> newl=perpendiculars;
        newl.push_back(middle);


        std::vector<Point> cruces; // will hold the results of the crossing point stimation
        if(cross){

          // Perpendiculars lines case
          cruces = find_points(middle,perpendiculars);
          if(cruces.size()>0){
            cruces = cruces_filter(cruces);
            std::string m = (folders[f] + " drone " + std::to_string(num));
            print_image(src,dst,newl,cruces,m);
          }
        }else{

          // Parallel lines case
          for(int i = floor(lines.size()*(0.5-H_FILTER_CONST)) ; i < floor(lines.size()*(0.5+H_FILTER_CONST)) ; i++){
            // We take just some lines near to the median
            horizontals.push_back(lines[i]);
          }

          // The middle point of a horiontal line is stimated at its crossing with the vertical center line.

          Vec2f vertical = {floor((float)dst.size().width/2),0.01}; // Vertical line in the center of the image (polar representation)
          cruces = find_points(vertical,horizontals);
          if(cruces.size()>0){
            cruces = cruces_filter(cruces);
            horizontals.push_back(vertical);
            std::string m = (folders[f] + " drone " + std::to_string(num));
            print_image(src,dst,horizontals,cruces,m);
          }
        }
      }
      waitKey(0);
    }
  }
  destroyAllWindows();
  return 0;
}
