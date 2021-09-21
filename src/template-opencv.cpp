/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"

#include "cluon-complete.cpp"

// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
 // Include the string stream library to use instead of buffer
#include <sstream>

int32_t main(int32_t argc, char ** argv) {
  int32_t retCode {
    1
  };
  // Parse the command line parameters as we require the user to specify some mandatory information on startup.
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
    (0 == commandlineArguments.count("name")) ||
    (0 == commandlineArguments.count("width")) ||
    (0 == commandlineArguments.count("height"))) {
    std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
    std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
    std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
    std::cerr << "         --width:  width of the frame" << std::endl;
    std::cerr << "         --height: height of the frame" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
  } else {
    // Extract the values from the command line parameters
    const std::string NAME {
      commandlineArguments["name"]
    };
    const uint32_t WIDTH {
      static_cast < uint32_t > (std::stoi(commandlineArguments["width"]))
    };
    const uint32_t HEIGHT {
      static_cast < uint32_t > (std::stoi(commandlineArguments["height"]))
    };
    const bool VERBOSE {
      commandlineArguments.count("verbose") != 0
    };

    // Attach to the shared memory.
    std::unique_ptr < cluon::SharedMemory > sharedMemory {
      new cluon::SharedMemory {
        NAME
      }
    };
    if (sharedMemory && sharedMemory -> valid()) {
      std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory -> name() << " (" << sharedMemory -> size() << " bytes)." << std::endl;

      // Interface to a running OpenDaVINCI session where network messages are exchanged.
      // The instance od4 allows you to send and receive messages.
      cluon::OD4Session od4 {
        static_cast < uint16_t > (std::stoi(commandlineArguments["cid"]))
      };

      opendlv::proxy::GroundSteeringRequest gsr;
      std::mutex gsrMutex;
      auto onGroundSteeringRequest = [ & gsr, & gsrMutex](cluon::data::Envelope && env) {
        // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
        // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
        std::lock_guard < std::mutex > lck(gsrMutex);
        gsr = cluon::extractMessage < opendlv::proxy::GroundSteeringRequest > (std::move(env));
        //std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
      };

      od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

      // HSV values for blue
      int minHueBlue = 102;
      int maxHueBlue = 150;
      int minSatBlue = 88;
      int maxSatBlue = 165;
      int minValueBlue = 43;
      int maxValueBlue = 222;

      //HSV values for yellow
      int minHueYellow = 0;
      int maxHueYellow = 42;
      int minSatYellow = 75;
      int maxSatYellow = 221;
      int minValueYellow = 170;
      int maxValueYellow = 255;

      int frameCounter = 0; // used to count starting frames
      int frameSampleSize = 5; // initial number of frames used to determine direction

      int identifiedShape = 60; // pixel size used to determine cones
      int yellowConeExists = 0; // flag to check if blue cones have been detected

      // Variables for steering angle calculation
      float steeringWheelAngle = 0.0;
      float steeringMax = 0.3;
      float steeringMin = -0.3;
      int carDirection = -1; // left car direction is negative (counterclockwise), default value

      // Variables for turning
      float carTurnR = 0.025;
      float carTurnL = -0.025;

      cv::Mat rightContourImage; // stores image when identifying car direction

      // Vectors used for storing cone contours 
      std::vector < std::vector < cv::Point > > contours;
      std::vector < cv::Vec4i > hierarchy;

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning()) {

        // Increase the frameCounter variable to get our sample frames for carDirection
        frameCounter++;

        // OpenCV data structure to hold an image.
        cv::Mat img;

        // Wait for a notification of a new frame.
        sharedMemory -> wait();

        // Lock the shared memory.
        sharedMemory -> lock(); {
          // Copy the pixels from the shared memory into our own data structure.
          cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory -> data());
          img = wrapped.clone();
        }

        std::pair < bool, cluon::data::TimeStamp > sTime = sharedMemory -> getTimeStamp(); // Saving current time in sTime var

        // Convert TimeStamp obj into microseconds
        uint64_t sMicro = cluon::time::toMicroseconds(sTime.second);

        //Shared memory is unlocked
        sharedMemory -> unlock();

        // Defining images for later use
        cv::Mat hsvRightImg;
        cv::Mat hsvCenterImg;
        cv::Mat detectRightImg;
        cv::Mat detectCenterImg;

        // loop runs until frame counter is greater than the sample size of 5, used to determine direction (counterclockwise, clockwise etc...)
        if (frameCounter < frameSampleSize) {
          // Operation to find yellow cones in HSV image

          // Defining the regions of interest for the right
          cv::Rect regionOfInterestRight = cv::Rect(415, 265, 150, 125);

          // Creating image with the defined regions of interest
          cv::Mat imageWithRegionRight = img(regionOfInterestRight);

          // Converts the imageWithRegionRight image to HSV values and stores the result in hsvRightImg
          cv::cvtColor(imageWithRegionRight, hsvRightImg, cv::COLOR_BGR2HSV);

          // Applying our defined HSV values as thresholds to hsvRightImg to create a new detectRightImg
          cv::inRange(hsvRightImg, cv::Scalar(minHueYellow, minSatYellow, minValueYellow), cv::Scalar(maxHueYellow, maxSatYellow, maxValueYellow), detectRightImg);

          //Applying Gaussian blur to detectRightImg
          cv::GaussianBlur(detectRightImg, detectRightImg, cv::Size(5, 5), 0);

          //Applying dilate and erode to detectLeftImg to remove holes from foreground
          cv::dilate(detectRightImg, detectRightImg, 0);
          cv::erode(detectRightImg, detectRightImg, 0);

          // The below will find the contours of the cones in detectRightImg and store them in the contours vector
          cv::findContours(detectRightImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          // Creates a mat object of the same size as detectRightImg used for storing the drawn contours
          rightContourImage = cv::Mat::zeros(detectRightImg.rows, detectRightImg.cols, CV_8UC3);

          // Loops over the contours vector
          for (unsigned int i = 0; i < contours.size(); i++) {

            // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
            if (cv::contourArea(contours[i]) > identifiedShape) {

              // Draws the contour of the cone on the image
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(rightContourImage, contours, i, colour, -1, 8, hierarchy);

              // Set yellowConeExists flag to 1 to indicate that we have found a flag
              yellowConeExists = 1;

              //If yellow cones are detected, that means the car direction is clockwise and the carDirection must be set as 1
              if (yellowConeExists == 1) {
                carDirection = 1;
              }
            }
          }
          // Frame counter printed for testing purposes
          // std::cout << "frame counter" << frameCounter;
        }

        // If frameCounter is larger than or equal to frameSampleSize
        if (frameCounter >= frameSampleSize) {

          // Defining the regions of interest for both centre 
          cv::Rect regionOfInterestCentre = cv::Rect(200, 245, 230, 115);

          // Creating images with the defined regions of interest
          cv::Mat imageWithRegionCentre = img(regionOfInterestCentre);

          // Converts the imageWithRegionCentre image to HSV values and stores the result in hsvCenterImg
          cv::cvtColor(imageWithRegionCentre, hsvCenterImg, cv::COLOR_BGR2HSV);

          // Applying our defined HSV values as thresholds to hsvCenterImg to create a new detectCenterImg
          cv::inRange(hsvCenterImg, cv::Scalar(minHueBlue, minSatBlue, minValueBlue), cv::Scalar(maxHueBlue, maxSatBlue, maxValueBlue), detectCenterImg);

          //Applying Gaussian blur to detectCenterImg
          cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);

          //Applying dilate and erode to detectCenterImg to remove holes from foreground
          cv::dilate(detectCenterImg, detectCenterImg, 0);
          cv::erode(detectCenterImg, detectCenterImg, 0);

          // The below will find the contours of the cones in detectLeftImg and store them in the contours vector
          cv::findContours(detectCenterImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

          // Creates a mat object of the same size as detectCenterImg used for storing the drawn contours
          cv::Mat blueContourImage = cv::Mat::zeros(detectCenterImg.rows, detectCenterImg.cols, CV_8UC3);

          int blueConeCenter = 0; // Flag for whether blue cones are detected in the image

          // Loops over the contours vector
          for (unsigned int i = 0; i < contours.size(); i++) {

            // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
            if (cv::contourArea(contours[i]) > identifiedShape) {
              // Draws the contour of the cone on the image
              cv::Scalar colour(255, 255, 0);
              cv::drawContours(blueContourImage, contours, i, colour, -1, 8, hierarchy);

              // If the current steeringWheelAngle is more than to steeringMin AND less than to steeringMax 
              if (steeringWheelAngle > steeringMin && steeringWheelAngle < steeringMax) {

                // If a blue cone has not been detected yet AND car direction is clockwise
                if (blueConeCenter != 1 && carDirection == 1) {
                  // Set blueConeCenter as 1 because it has detected a cone 
                  blueConeCenter = 1;

                  // Turn right when a blue cone is detected, to steer away from the cone
                  steeringWheelAngle = steeringWheelAngle - carTurnR;
                  //std::cout << "line 288 " << steeringWheelAngle << std::endl;

                } // If a blue cone has not been detected yet AND car direction is counterclockwise
                else if (blueConeCenter != 1 && carDirection == -1) {
                  // Set blueConeCenter as 1 because it has detected a cone 
                  blueConeCenter = 1;

                  // Turn left when a blue cone is detected, to steer away from the cone
                  steeringWheelAngle = steeringWheelAngle - carTurnL;
                  //std::cout << "line 298 " << steeringWheelAngle << std::endl;
                }

              } // If the current steering angle is less than steeringMin or more than steeringMax 
              else {
                // Set steeringWheelAngle to 0 (go straight, no new steering angle provided by driver)
                blueConeCenter = 1;
                steeringWheelAngle = 0.0;
                //std::cout << "line 306 " << steeringWheelAngle << std::endl;
              }

            }
          }
          // Pop up window used for testing 
          // If verbose is included in the command line, a window showing only the blue contours will appear
          if (VERBOSE) {
            cv::imshow("Blue Contours", blueContourImage);
            cv::waitKey(1);
          }

          // If a blue cone hasn't been detected, we check for yellow cones
          if (blueConeCenter != 1) {

            // Converts the imageWithRegionCentre image to HSV values and stores the result in hsvCenterImg
            cv::cvtColor(imageWithRegionCentre, hsvCenterImg, cv::COLOR_BGR2HSV);

            // Applying our defined HSV values as thresholds to hsvCenterImg to create a new detectCenterImg
            cv::inRange(hsvCenterImg, cv::Scalar(minHueYellow, minSatYellow, minValueYellow), cv::Scalar(maxHueYellow, maxSatYellow, maxValueYellow), detectCenterImg);

            //Applying Gaussian blur to detectCenterImg
            cv::GaussianBlur(detectCenterImg, detectCenterImg, cv::Size(5, 5), 0);

            //Applying dilate and erode to detectCenterImg to remove holes from foreground
            cv::dilate(detectCenterImg, detectCenterImg, 0);
            cv::erode(detectCenterImg, detectCenterImg, 0);

            // The below will find the contours of the cones in detectLeftImg and store them in the contours vector
            cv::findContours(detectCenterImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            // Creates a mat object of the same size as detectCenterImg used for storing the drawn contours
            cv::Mat yellowContourImage = cv::Mat::zeros(detectCenterImg.rows, detectCenterImg.cols, CV_8UC3);

            int yellowConeCenter = 0; // Flag for whether yellow cones are detected in the image

            // Loops over the contours vector
            for (unsigned int i = 0; i < contours.size(); i++) {
              // If the current index of the vector has a contour area that is larger than the defined number of pixels in identifiedShape, we have a cone
              if (cv::contourArea(contours[i]) > identifiedShape) {
                // Draws the contour of the cone on the image
                cv::Scalar colour(255, 255, 0);
                cv::drawContours(yellowContourImage, contours, i, colour, -1, 8, hierarchy);

                // If the current steeringWheelAngle is more than steeringMin AND less than to steeringMax
                if (steeringWheelAngle > steeringMin && steeringWheelAngle < steeringMax) {

                  // If a yellow cone has not been detected yet AND car direction is clockwise
                  if (yellowConeCenter != 1 && carDirection == 1) {
                    // Set yellowConeCenter as 1 because it has detected a cone
                    yellowConeCenter = 1;

                    // Turn left when a yellow cone is detected, to steer away from the cone
                    steeringWheelAngle = steeringWheelAngle - carTurnL;
                    // std::cout << "line 368 " << steeringWheelAngle << std::endl;

                  } // If a yellow cone has not been detected yet AND car direction is counterclockwise 
                  else if (yellowConeCenter != 1 && carDirection == -1) {
                    // Set yellowConeCenter as 1 because it has detected a cone
                    yellowConeCenter = 1;

                    // Turn right when a yellow cone is detected, to steer away from the cone
                    steeringWheelAngle = steeringWheelAngle - carTurnR;
                    //std::cout << "line 378 " << steeringWheelAngle << std::endl;
                  }

                } // If the current steering angle is less than steeringMin or more than steeringMax
                else {
                  // Set steeringWheelAngle to 0 (go straight, no new steering angle provided by driver)
                  yellowConeCenter = 1;
                  steeringWheelAngle = 0.0;
                  //std::cout << "line 386 " << steeringWheelAngle << std::endl;
                }
              }
            }
            // Pop up window used for testing
            // If verbose is included in the command line, a window showing only the yellow contours will appear
            if (VERBOSE) {
              cv::imshow("Yellow Contours", yellowContourImage);
              cv::waitKey(1);
            }

            // If no blue or yellow cones have been detected
            if (yellowConeCenter == 0 && blueConeCenter == 0) {
              // If no cones are present, the steeringWheelAngle is set to 0
              steeringWheelAngle = 0.00;
              // std::cout << "line 401 " << steeringWheelAngle << std::endl;
            }
          }
        }

        // creates string stream input, optimized buffer, convert whatever is coming in as string
        std::ostringstream calcGroundSteering;
        std::ostringstream actualSteering;
        std::ostringstream timestamp;

        // putting values into stream
        calcGroundSteering << steeringWheelAngle;
        actualSteering << gsr.groundSteering();
        timestamp << sMicro;

        // creating strings for printing
        std::string time = " Time Stamp: ";
        std::string calculatedGroundSteering = "Calculated Ground Steering: ";
        std::string actualGroundSteering = " Actual Ground Steering: ";
        std::string groundSteeringAngle = std::to_string(steeringWheelAngle);

        // appending into one string to display
        calculatedGroundSteering.append(groundSteeringAngle);
        calculatedGroundSteering.append(calcGroundSteering.str());
        calculatedGroundSteering.append(actualGroundSteering);
        calculatedGroundSteering.append(actualSteering.str());
        calculatedGroundSteering.append(time);
        calculatedGroundSteering.append(timestamp.str());

        // Displays information on video
        cv::putText(img, //target image
          calculatedGroundSteering,
          cv::Point(1, 50),
          cv::FONT_HERSHEY_DUPLEX,
          0.35,
          CV_RGB(0, 250, 154));

        {
          std::lock_guard < std::mutex > lck(gsrMutex);
          std::cout << "group_16;" << sMicro << ";" << steeringWheelAngle << std::endl;
         // std::cout << sMicro << ";" << steeringWheelAngle << ";" << gsr.groundSteering() << " car direction: " << carDirection << std::endl;
        }

        // Displays debug window on screen
        if (VERBOSE) {
          cv::imshow("Debug", img);
          cv::waitKey(1);
        }

      }
    }
    retCode = 0;
  }
  return retCode;
}