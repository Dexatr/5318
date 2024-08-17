/*
 *
 *  Example by Sam Siewert 
 *
 *  Updated 12/6/18 for OpenCV 3.1
 *
 *  This code captures video frames from a webcam, computes the difference 
 *  between consecutive frames, and displays the difference along with the 
 *  current and previous frames. The difference is calculated in grayscale, 
 *  and if the difference exceeds a certain threshold, a message is logged.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <syslog.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

// Buffers for displaying difference and time in the output window
char difftext[20];
char timetext[20];

int main(int argc, char** argv)
{
    // Matrix containers for video frames and differences
    Mat mat_frame, mat_gray, mat_diff, mat_gray_prev;  
    VideoCapture vcap;  // VideoCapture object to capture video from webcam
    unsigned int diffsum, maxdiff, framecnt = 0;  // Variables for difference computation and frame counting
    double percent_diff = 0.0, percent_diff_old = 0.0;
    double ma_percent_diff = 0.0, fcurtime = 0.0, start_fcurtime = 0.0;

    struct timespec curtime;

    // Record the start time
    clock_gettime(CLOCK_REALTIME, &curtime);
    start_fcurtime = (double)curtime.tv_sec + ((double)curtime.tv_nsec / 1000000000.0);

    // Open the default video device (usually the built-in webcam)
    if (!vcap.open(0)) 
    {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Opened default camera interface" << std::endl;
    }

    // Ensure the first frame is captured
    while (!vcap.read(mat_frame)) {
        std::cout << "No frame" << std::endl;
        cv::waitKey(33);  // Wait for 33 milliseconds before trying again
    }

    // Convert the first frame to grayscale
    cv::cvtColor(mat_frame, mat_gray, cv::COLOR_BGR2GRAY);

    // Initialize matrices for difference computation
    mat_diff = mat_gray.clone();  // Clone the grayscale frame to initialize the difference matrix
    mat_gray_prev = mat_gray.clone();  // Clone the grayscale frame to initialize the previous frame matrix

    // Calculate the maximum possible difference (used for percentage difference calculation)
    maxdiff = (mat_diff.cols) * (mat_diff.rows) * 255;

    while (1)
    {
        // Capture the next frame
        if (!vcap.read(mat_frame)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();  // Wait indefinitely if no frame is captured
        }
        else
        {
            framecnt++;  // Increment frame count
            clock_gettime(CLOCK_REALTIME, &curtime);  // Get the current time
            fcurtime = (double)curtime.tv_sec + ((double)curtime.tv_nsec / 1000000000.0) - start_fcurtime;  // Calculate elapsed time
        }

        // Convert the current frame to grayscale
        cv::cvtColor(mat_frame, mat_gray, cv::COLOR_BGR2GRAY);

        // Compute the absolute difference between the current and previous grayscale frames
        absdiff(mat_gray_prev, mat_gray, mat_diff);

        // Calculate the sum of differences (single channel sum)
        diffsum = (unsigned int)cv::sum(mat_diff)[0]; 

        // Calculate the percentage difference relative to the maximum possible difference
        percent_diff = ((double)diffsum / (double)maxdiff) * 100.0;

        // Calculate a moving average of the percentage difference
        if (framecnt < 3)
            ma_percent_diff = (percent_diff + percent_diff_old) / (double)framecnt;
        else
            ma_percent_diff = ((ma_percent_diff * (double)framecnt) + percent_diff) / (double)(framecnt + 1);

        // Log the percentage difference and other related information
        syslog(LOG_CRIT, "TICK: percent diff, %lf, old, %lf, ma, %lf, cnt, %u, change, %lf\n", percent_diff, percent_diff_old, ma_percent_diff, framecnt, (percent_diff - percent_diff_old));
        sprintf(difftext, "%8d", diffsum);  // Format the difference sum as a string
        sprintf(timetext, "%6.3lf", fcurtime);  // Format the current time as a string

        percent_diff_old = percent_diff;  // Update the old percentage difference

        // Display the difference and time information if the percentage difference exceeds 0.5%
        if (percent_diff > 0.5)
        {
            cv::putText(mat_diff, difftext, cv::Point(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, cv::LINE_AA);
            cv::putText(mat_diff, timetext, cv::Point(500, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, cv::LINE_AA);
        }

        // Show the current frame, the previous frame, and the difference between them
        cv::imshow("Clock Current", mat_gray);
        cv::imshow("Clock Previous", mat_gray_prev);
        cv::imshow("Clock Diff", mat_diff);

        // Wait for 100ms or until the user presses 'q' to quit
        char c = cv::waitKey(100);  // Sample rate
        if (c == 'q') break;

        // Update the previous frame to the current frame
        mat_gray_prev = mat_gray.clone();
    }

};
