#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>

#define pi 3.14159f
#define PERPEDICULAR_MARGIN 30    // min angle between lines for taking their crossing points
#define H_FILTER_CONST 0.2f       // lines selection: Algorithm takes [0.5-H_FILTER_CONST,0.5+H_FILTER_CONST]*lines.size();
#define BLUR_FILTER_SIZE 13       // Kernel Size filter
#define LENGTH_CROSS_FILTER 15     // Lenght of median filter buffer
#define ITER_FILT 1               // number of Gaussian filter repetition
#define PIXELS_PER_LINE 40        // how much pixels must have a detected right line

using namespace cv;

// Canny Filter Parameters
const int lowThreshold = 50;
const int MaxlowThreshold = 100;
//Frame size of video
int H = 42; //inicialized in a random value
int W = 42;
// Dron position
Point Dron_pos(0,0);


// buffer for saving the last drone positions for median filter
std::vector<Point> cruces_filter_buffer(LENGTH_CROSS_FILTER,{0,0});
int median_buffer_count = 0; //auxiliar for itering in cruces_filter_buffer

// Processing = Gaussian filtering + Canny edge detector
Mat process(Mat & src){
  Mat resized, detected_edges;
  cvtColor( src, src, COLOR_BGR2GRAY );
  resize(src,src,Size(0.2*src.cols,0.2*src.rows));
  resized=src.clone();
  for (int i = 0 ; i<ITER_FILT; i++)
    GaussianBlur( resized, resized, Size(BLUR_FILTER_SIZE,BLUR_FILTER_SIZE),0,0);resized;
  Canny( resized, detected_edges, lowThreshold, MaxlowThreshold, 3 );
return detected_edges;
}

/* Print : original image + edge detection result + lines detected + drone trancking point stimated
        src : Original image
        dst : Processed image
        lines : vector of detected lines
        points : vector of detected points
*/
void print_image(Mat & src, Mat & dst, std::vector<Vec2f> & lines){
  // every line is a vector of his values in Polar representation

  Mat withoutlin=dst.clone();
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
  hconcat(src,dst,salida);// hconcat : horiontal concatenation of 2 images
  // ploting  green points
  cvtColor(salida, salida, COLOR_GRAY2BGR);
  circle(salida, {Dron_pos.x, Dron_pos.y} , 5, {0,255,0}, 3);
  imshow("Drones detections",salida);
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

  // Save the nearest point (drone stimated position) in a buffer
  cruces_filter_buffer[median_buffer_count++]=cr[0];
  // median_buffer_count is an auxiliar counter
  if(median_buffer_count == LENGTH_CROSS_FILTER) median_buffer_count =0; // force counter to conversion

  // Sorting buffer for median filter
  std::sort(cruces_filter_buffer.begin(), cruces_filter_buffer.end(), [](const Point & a, const Point & b) {return (a.x + a.y) < (b.x+b.y); });

  std::vector<Point> cruces;
  cruces.push_back(cruces_filter_buffer[floor((float)LENGTH_CROSS_FILTER/2)]); // Get median value
  return cruces;
}

void pose_update(std::vector<Point> P){
    Dron_pos = P[0];
}

int main( int argc, char** argv )
{
  // Open video as a imput stream
  VideoCapture cap("../dron.mp4");
  
  if(!cap.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }

  Mat src,dst;
  cap >> src; // get initial frame
  // Processing the image
  dst = process(src);
  //actualice its size global values
  H = dst.rows;
  W = dst.cols;
    
  while(1){
  
    // Capture frame-by-frame
    cap >> src;

    // If the frame is empty, break immediately
    if (src.empty())
      break;
    
    // Processing the image
    dst = process(src);


    std::vector<Vec2f> lines; // will hold the results of the detection
    HoughLines(dst, lines, 1, CV_PI/180, PIXELS_PER_LINE, 0, 0 ); // runs the actual detection

    // getting the middle inclination value
    std::sort(lines.begin(), lines.end(), [](const Vec2f & a, const Vec2f & b) {return a[1] < b[1]; });
    char c=(char)waitKey(25);
    if(c==27)
      break;
    
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
      std::vector<Vec2f> newl = perpendiculars;
      newl.push_back(middle);

      std::vector<Point> cruces;// will hold the results of the crossing point stimation

      if(cross){

        // Perpendiculars lines case
        cruces = find_points(middle,perpendiculars);
        if(cruces.size()>0){
          cruces = cruces_filter(cruces);
          pose_update(cruces);
          print_image(src,dst,newl);
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
          pose_update(cruces);
          horizontals.push_back(vertical);
          print_image(src,dst,horizontals);
        }
      }
    }else{
        std::vector<Vec2f> auxl;
        print_image(src,dst,auxl);
    }
  }

  // relase input stream
  cap.release();

  // Closes all the frames
  destroyAllWindows();

  return 0;
}
