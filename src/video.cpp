#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>

#define pi 3.14159
#define PERPEDICULAR_MARGIN 30
#define H_FILTER_CONST 0.2f
#define BLUR_FILTER_SIZE 13
#define LENGTH_CROSS_FILTER 15
#define ITER_FILT 3
#define PIXELS_PER_LINE 50

using namespace cv;

const int lowThreshold = 50;
const int MaxlowThreshold = 100;
//Frame size of video
int H = 42;
int W = 42;


std::vector<Point> cruces_filter_buffer(LENGTH_CROSS_FILTER,{0,0});
int iter_filt=3;

Mat tratar(Mat & src, int filter_size, int iter_filt){
  Mat resized,src_gray, blured, detected_edges;
  cvtColor( src, src, COLOR_BGR2GRAY );
  resize(src,src,Size(0.2*src.cols,0.2*src.rows));
  resized=src.clone();
  for (int i = 0 ; i<ITER_FILT; i++)
    GaussianBlur( resized, resized, Size(filter_size,filter_size),0,0);
  blured = resized;
  Canny( blured, detected_edges, lowThreshold, MaxlowThreshold, 3 );
return detected_edges;
}

void print_image(Mat & src, Mat & dst, std::vector<Vec2f> & lines, std::vector<Point> & points){
  Mat withoutlin=dst.clone();
  for( size_t i = 0; i < lines.size(); i++ )
  {
      float rho = lines[i][0], theta = lines[i][1];
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line(dst, pt1, pt2, Scalar(255,255,255), 1, LINE_AA);
  }
  Mat salida;
  hconcat(src,dst,salida);
  cvtColor(salida, salida, COLOR_GRAY2BGR);
  for(auto & p : points){
    circle(salida, {p.x, p.y} , 5, {0,255,0}, 3);
  }
  imshow("Drones detections",salida);
}

Mat read_img(std::string name){
  Mat src;
  src = imread(name, IMREAD_COLOR ); // Load an image
  if( src.empty())
  {
    std::cout << "Could not open or find the image!\n" << std::endl;
  }
  return src;
}

float comparison(float a , float b){
  return min(abs(a-b),180-abs(a-b));
}

std::vector<Point> line2Point(Vec2f line){
  float rho = line[0], theta = line[1];
  Point pt1, pt2;
  double a = cos(theta), b = sin(theta);
  double x0 = a*rho, y0 = b*rho;
  pt1.x = cvRound(x0 + 1000*(-b));
  pt1.y = cvRound(y0 + 1000*(a));
  pt2.x = cvRound(x0 - 1000*(-b));
  pt2.y = cvRound(y0 - 1000*(a));
  std::vector<Point> points = {pt1,pt2};
  return points;
}

Point find_crossing(Vec2f & ap, Vec2f & bp){
  float m1,m2,b1,b2;
  Point crossing;
  m1 = -cos(ap[1])/sin(ap[1]);
  m2 = -cos(bp[1])/sin(bp[1]);
  b1 = ap[0]/sin(ap[1]);
  b2 = bp[0]/sin(bp[1]);
  crossing.x = (b2-b1)/(m1-m2);
  crossing.y = m2*crossing.x + b2;
  return crossing;
}

std::vector<Point> find_points(Vec2f & middle, std::vector<Vec2f> & lines){
  std::vector<Point> crossing;
  for (auto & l : lines){
    Point cruce = find_crossing(middle, l);
    crossing.push_back(cruce);
  }
  return crossing;
}

std::vector<Point> cruces_filter(std::vector<Point> & cr){
  std::sort(cr.begin(), cr.end(), [](const Point & a, const Point & b) {return ((pow((H-a.y),2)+pow((W-a.x),2)) < (pow((H-b.y),2)+pow((W-b.x),2))); });

  cruces_filter_buffer[iter_filt++]=cr[0];

  std::sort(cruces_filter_buffer.begin(), cruces_filter_buffer.end(), [](const Point & a, const Point & b) {return (a.x + a.y) < (b.x+b.y); });
  if(iter_filt == LENGTH_CROSS_FILTER) iter_filt =0;
  std::vector<Point> cruces;
  cruces.push_back(cruces_filter_buffer[floor((float)LENGTH_CROSS_FILTER/2)]);
  return cruces;
}

int main( int argc, char** argv )
{

  VideoCapture cap("../dron.mp4");
  if(!cap.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }

  Mat src,dst;
  cap >> src;

  dst = tratar(src,BLUR_FILTER_SIZE,3);
  H = dst.rows;
  W = dst.cols;

  while(1){
    // Capture frame-by-frame
    cap >> src;

    // If the frame is empty, break immediately
    if (src.empty())
      break;

    dst = tratar(src,BLUR_FILTER_SIZE,3);

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
        float aux = comparison(middle[1], l[1]);
        if (aux > PERPEDICULAR_MARGIN*(pi/180)){
          cross = true;
          perpendiculars.push_back(l);
        }
      }

      std::vector<Vec2f> newl=perpendiculars;
      newl.push_back(middle);

      std::vector<Point> cruces;

      if(cross){
        cruces = find_points(middle,perpendiculars);
        if(cruces.size()>0){
          cruces = cruces_filter(cruces);
          print_image(src,dst,newl,cruces);
        }
      }else{
        for(int i = floor(lines.size()*(0.5-H_FILTER_CONST)) ; i < floor(lines.size()*(0.5+H_FILTER_CONST)) ; i++){
          horizontals.push_back(lines[i]);
        }
        //std::cout << dst.size() << std::endl;
        Vec2f vertical = {floor((float)dst.size().width/2),0.01};
        cruces = find_points(vertical,horizontals);
        if(cruces.size()>0){
          cruces = cruces_filter(cruces);
          horizontals.push_back(vertical);
          print_image(src,dst,horizontals,cruces);
        }
      }
    }
  }

  cap.release();

  // Closes all the frames
  destroyAllWindows();

  waitKey(0);
  return 0;
}
