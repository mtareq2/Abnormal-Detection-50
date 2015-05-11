#include <stdio.h>
#include<stdlib.h>

#include <opencv/cv.h>
#include<vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "package_bgs/pt/PixelBasedAdaptiveSegmenter.h"
#include "package_tracking/BlobTracking.h"
#include "package_analysis/VehicleCouting.h"


using namespace cv;

void OffsetImage(Mat &image, cv::Scalar bordercolour, int xoffset, int yoffset);

int main(int argc, char **argv)
{
	//Total number of videos
	int const numberOfVideos = 11;
	int const numberOfCols = 10, numberOfRows = 5;
	int top, bottom, left, right;
	// resize_factor: 50% of original image
	//P.S: hat3'yr el width w el height hayt3'er f el dst msh hatl7zo f el destination
	int resize_factor = 100, width = 600, height = 240, key = 0, h, w;

	CvCapture *capture[numberOfVideos] = { 0 };
	IplImage *frame_aux[numberOfVideos];
	IplImage *frame[numberOfVideos];

	/* Background Subtraction Methods */
	IBGS *bgs[numberOfVideos];
	/* Blob Tracking */
	Mat img_blob[numberOfVideos];
	//vector<Mat> img_blob;
	BlobTracking* blobTracking[numberOfVideos];
	/* Vehicle Counting Algorithm */
	VehicleCouting* vehicleCouting[numberOfVideos];

	Mat img_input[numberOfVideos];
	Mat img_mask[numberOfVideos];
	Mat img[numberOfVideos];

	//Destination image: big img contains all videos frames. 5 rows 10 cols (10 imgs in each row) 
	Mat dst(Size(numberOfCols * width, numberOfRows * height), CV_8UC3);

	char * filename = new char[100];
	int  counter = -1, counter2;

	//loop numberOfVideos times initialize bgs and other AND get first frame for each video
	for (int it = 0; it < numberOfVideos; it++){
		++counter;
		bgs[it] = new PixelBasedAdaptiveSegmenter;
		blobTracking[it] = new BlobTracking;
		vehicleCouting[it] = new VehicleCouting;

		if (counter >= 6)
			counter = 0;

		sprintf(filename, "C:/Users/mohamedtarek/Desktop/GP Project/Dataset_ Myvideos/test%03d.mp4", counter);
		capture[it] = cvCaptureFromFile(filename);
		if (!capture[it]){
			printf("Cannot open video!");
			return 1;
		}

		frame_aux[it] = cvQueryFrame(capture[it]);
		frame[it] = cvCreateImage(cvSize((int)((frame_aux[it]->width*resize_factor) / 100), (int)((frame_aux[it]->height*resize_factor) / 100)), frame_aux[it]->depth, frame_aux[it]->nChannels);
		//frame[it] = frame_aux[it];
	}

	while (key != 'q')
	{
		h = 0; w = width; counter2 = -1;
		for (int it = 0; it < numberOfVideos; it++){

			frame_aux[it] = cvQueryFrame(capture[it]);
			if (!frame_aux[it]) break;

			//b n resize el frame_aux 3la ad el frame 3lshan yt7t f el frame
			cvResize(frame_aux[it], frame[it]);

			img[it].create(Size(width, height), CV_8UC3);

			++counter2;

			/*** PT Package (adapted from Hofmann) ***/
			img_input[it] = frame[it];
			// bgs->process(...) method internally shows the foreground mask image   
			bgs[it]->process(img_input[it], img_mask[it]);


			if (!img_mask[it].empty())
			{
				// Perform blob tracking
				blobTracking[it]->process(img_input[it], img_mask[it], img_blob[it]);

				if (img_blob[it].data){
					cv::resize(img_blob[it], img_blob[it], img_blob[0].size());
					img[it] = img_blob[it];

					// cout << "dst " << dst->width << " " << dst->height << "\nimg1 " << img1.width << " " << img1.height << "\nimg2 " << img2.width << " " << img2.height << endl;


					//if we reached the maximum width go to the next row
					if (it != 0 && it % 10 == 0){
						++h;
						counter2 = 0;
					}

					//size of the borders (top, bottom, left and right). We give them a value of 3% the size of src
					top = (int)(0.03*img[it].rows);
					bottom = (int)(0.03*img[it].rows);
					left = (int)(0.03*img[it].cols);
					right = (int)(0.03*img[it].cols);

					copyMakeBorder(img[it], img[it], top, bottom, left, right, BORDER_CONSTANT, Scalar(0,0,0));
					resize(img[it], img[it], Size(width, height));
					

					// Perform vehicle counting
					vehicleCouting[it]->setInput(img_blob[it]);
					vehicleCouting[it]->setTracks(blobTracking[it]->getTracks());
					vehicleCouting[it]->process();

					// Copy image to spesfic rectanle region(x,y,width,height) in the dst image 
					//Mat imgPanelRoi(dst, Rect(counter2*w, h*img[it].size().height, img[it].size().width, img[it].size().height));
					Mat imgPanelRoi(dst, Rect(counter2*w, h*height, width, height));

					//OffsetImage(img[it], Scalar(200,0,0), 0, 0);
					//img[it](Rect(0, 0, img[it].size().width, img[it].size().height)).setTo(Scalar(200, 0, 0));
					img[it].copyTo(imgPanelRoi);

					/*IplImage *destination = cvCreateImage(cvSize(1300, 650), dst->depth, dst->nChannels);
					cvResize(dst, destination);*/

					Mat destination;
					destination.create(Size(1300, 650), dst.type());
					resize(dst, destination, destination.size());
					imshow("destination", destination);
					imshow("dst", dst);


				}


			}// end if mask
			waitKey(5);
		} // end inner for loop

	} //end while

	delete vehicleCouting;
	delete blobTracking;
	delete bgs;

	cvDestroyAllWindows();

	for (int i = 0; i <= 49; i++)
		cvReleaseCapture(&capture[i]);

	return 0;
}

void OffsetImage(Mat &image, cv::Scalar bordercolour, int xoffset, int yoffset)
{
	Mat temp(image.rows + 2 * yoffset, image.cols + 2 * xoffset, image.type(), bordercolour);
	Mat roi(temp(cvRect(xoffset, yoffset, image.cols, image.rows)));
	image.copyTo(roi);
	image = temp.clone();
}