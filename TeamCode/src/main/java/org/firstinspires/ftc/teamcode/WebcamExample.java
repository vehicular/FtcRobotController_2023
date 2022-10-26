/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@TeleOp
public class WebcamExample extends LinearOpMode
{
    OpenCvWebcam webcam;
    RedConeDectectionPipeline redPipeline;
    YellowPoleDetector yellowPipeline;
    
    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", 
                        hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        //redPipeline = new RedConeDectectionPipeline();
        //webcam.setPipeline(redPipeline);
    
        yellowPipeline = new YellowPoleDetector();
        webcam.setPipeline(yellowPipeline);
        
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    
        telemetry.addLine("Waiting for start");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            
            //telemetry.addData("Num contours found", redPipeline.getNumContoursFound());
            
            telemetry.addData("Pole Position", yellowPipeline.getLocation_leftright());
            telemetry.addData("top_left", yellowPipeline.top_left_x);
            telemetry.addData("top_right", yellowPipeline.top_right_x);
            telemetry.addData("bottom_left", yellowPipeline.bottom_left_x);
            telemetry.addData("bottom_right", yellowPipeline.bottom_right_x);
            
            telemetry.update();
    
            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }
    
            //YellowPoleDetector.PoleLocation location = yellowPipeline.getLocation();
            //if (location != YellowPoleDetector.PoleLocation.NONE) {
                // Move to the left / right
            //} else {
                // adjust claw, and drop the cone
            //}
            
            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
                * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
                * anyway). Of course in a real OpMode you will likely not want to do this.
                */
            sleep(100);
        }
    }
    
    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    // Good to dectect red cone
    static class RedConeDectectionPipeline extends TimestampedOpenCvPipeline //OpenCvPipeline
    {
        Point stageTextAnchor;
        Point timeTextAnchor;
        
        Scalar green = new Scalar(0,255,0,255);
        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 180;  // 102, 80
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
    
        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_CHAN_IDX = 1;//2;
    
        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12,12));//(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9));
    
        /*
         * Colors
         */
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        
        
        
    
        /*
         * Our working image buffers
         */
        Mat yCbCrChanMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();
    
    
        
    
        static class AnalyzedStone
        {
            StoneOrientation orientation;
            double angle;
        }
    
        enum StoneOrientation
        {
            UPRIGHT,
            NOT_UPRIGHT
        }
        
        ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
        // Volatile since accessed by OpMode thread w/o synchronization ?
        volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();
        
        
    
        int numContoursFound;
    
        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181,98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253,98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;
    
    
        int avg1, avg2, avg3;
    
        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        
        enum Stage
        {
            YCbCr_CHAN,
            THRESHOLD,
            MORPHED,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }
        
        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN;
        private Stage[] stages = Stage.values();
        
        
        @Override
        public void init(Mat mat)
        {
            timeTextAnchor = new Point(0, 15);
            stageTextAnchor = new Point(540, mat.height()-10);
            runtime.reset();
        }
        
        @Override
        public void onViewportTapped()
        {
        }
    
        ElapsedTime runtime = new ElapsedTime();
        
        @Override
        public Mat processFrame(Mat input, long captureTimeNanos)
        {
            if(runtime.seconds()>5)
            {
                int currentStageNum = stageToRenderToViewport.ordinal();
                int nextStageNum = currentStageNum + 1;
                if(nextStageNum >= stages.length)
                {
                    nextStageNum = 0;
                }
                stageToRenderToViewport = stages[nextStageNum];
                runtime.reset();
            }
            
            Imgproc.putText(input, String.format("Time: %d", captureTimeNanos),
                    timeTextAnchor, Imgproc.FONT_HERSHEY_PLAIN, 1, green, 1);
    
            Imgproc.putText(input, String.format("%s",stageToRenderToViewport.toString()),
                    stageTextAnchor, Imgproc.FONT_HERSHEY_PLAIN, 1, green, 1);
    
    
            
            internalStoneList.clear();
            
    
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }
    
            clientStoneList = new ArrayList<>(internalStoneList);
            
            //https://gist.github.com/razimgit/d9c91edfd1be6420f58a74e1837bde18
            
            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN:
                {
                    return yCbCrChanMat;
                }
                
                case THRESHOLD:
                {
                    return thresholdMat;
                }
                
                case MORPHED:
                {
                    return morphedThreshold;
                }
                
                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnPlainImageMat;
                }
                
                default: //RAW_IMAGE
                {
                    return input;
                }
            }
        }
    
        ArrayList<MatOfPoint> findContours(Mat input)
        {
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();
            //contoursList.clear(); // better to new it?
            
            // A list we'll be using to store the contours we find
            //ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        
            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, yCbCrChanMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChanMat, yCbCrChanMat, CB_CHAN_IDX); //2->red? 1->blue
        
            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(yCbCrChanMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
            morphMask(thresholdMat, morphedThreshold);
        
            // Ok, now actually look for the contours! We only look for external contours.
            //Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            
            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);
    
    
            return contoursList;
        }
        
        void morphMask(Mat input, Mat output)
        {
            /*
             * Apply some erosion and dilation for noise reduction
             */
            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);
        
            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }
    
        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        
        void analyzeContour(MatOfPoint contour, Mat input)
        {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
    
            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);
            
            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            //region1_Cb = yCbCrChanMat.submat(new Rect(region1_pointA, region1_pointB));
            //region2_Cb = yCbCrChanMat.submat(new Rect(region2_pointA, region2_pointB));
            //region3_Cb = yCbCrChanMat.submat(new Rect(region3_pointA, region3_pointB));
            
            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            //avg1 = (int) Core.mean(region1_Cb).val[0];
            //avg2 = (int) Core.mean(region2_Cb).val[0];
            //avg3 = (int) Core.mean(region3_Cb).val[0];
    
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines*/
    
            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines*/
    
            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines*/
    
    
            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);
        }
        
        public int getNumContoursFound()
        {
            return numContoursFound;
        }
    
    
    
        static void drawTagText(RotatedRect rect, String text, Mat mat)
        {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x-50,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1); // Font thickness
        }
    
        static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */
        
            Point[] points = new Point[4];
            rect.points(points);
        
            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    }
    
    
    static class YellowPoleDetector extends OpenCvPipeline {
        enum PoleLocation {
            LEFT,
            RIGHT,
            NONE
        }
        enum Stage
        {
            HSV_CHAN,
            THRESHOLD,
            MORPHED,
            EDGE,
            //HIERARCHY,
            //CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }
    
        private YellowPoleDetector.Stage stageToRenderToViewport = YellowPoleDetector.Stage.HSV_CHAN;
        private YellowPoleDetector.Stage[] stages = YellowPoleDetector.Stage.values();
        
        private int width; // width of the image
        int location_leftright = 0;
        int location_forwardback = 0;
    
        /*
         * Our working image buffers
         */
        Mat hSVChanMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat edgesMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();
    
    
        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6,6));//(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    
        static final int CB_CHAN_MASK_THRESHOLD = 150;
    
        // THESE NEED TO BE TUNED BASED ON YOUR DISTANCE FROM THE POLE
        private final double minContourArea = 300.0;
        private final double minContourPerimeter = 1000.0;
        private final double minContourWidth = 10.0;
        private final double minContourHeight = 100.0;
        
        public YellowPoleDetector() {
        
        }
        
        
        ElapsedTime runtime = new ElapsedTime();
    
        @Override
        public void init(Mat mat)
        {
            this.width = mat.width();
            runtime.reset();
        }
        
        @Override
        public Mat processFrame(Mat input) {
            
            if(runtime.seconds()>5)
            {
                int currentStageNum = stageToRenderToViewport.ordinal();
                int nextStageNum = currentStageNum + 1;
                if(nextStageNum >= stages.length)
                {
                    nextStageNum = 0;
                }
                stageToRenderToViewport = stages[nextStageNum];
                runtime.reset();
            }
            
            // "Mat" stands for matrix, which is basically the image that the detector will process
            // the input matrix is the image coming from the camera
            // the function will return a matrix to be drawn on your phone's screen
            
            // The detector detects regular stones. The camera fits two stones.
            // If it finds one regular stone then the other must be the Pole.
            // If both are regular stones, it returns NONE to tell the robot to keep looking
            
            // Make a working copy of the input matrix in HSV
            Imgproc.cvtColor(input, hSVChanMat, Imgproc.COLOR_RGB2HSV);
            
            // if something is wrong, we assume there's no Pole
            if (hSVChanMat.empty()) {
                location_leftright = 100;
                return input;
            }
            
            // We create a HSV range for yellow to detect regular stones
            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Scalar lowHSV = new Scalar(11.8, 161.7, 116.5); // lower bound HSV for yellow 20,100,100
            Scalar highHSV = new Scalar(30.3, 255.0, 255.0); // higher bound HSV for yellow 30,255,255
            
            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(hSVChanMat, lowHSV, highHSV, thresholdMat);
            //Imgproc.threshold(hSVChanMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
    
            morphMask(thresholdMat, morphedThreshold);
            // Use Canny Edge Detection to find edges
            // you might have to tune the thresholds for hysteresis
            Imgproc.Canny(morphedThreshold, edgesMat, 100, 300);
            
            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            // We then find the bounding rectangles of those contours
            List<MatOfPoint> contoursList = new ArrayList<>();
            
            //Imgproc.findContours(edgesMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(edgesMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
    
            // remove noisy contours
            List<MatOfPoint> outputContours = new ArrayList<>();
            filterContours(contoursList,outputContours, minContourArea, minContourPerimeter, minContourWidth, minContourHeight);
            //https://github.com/Epsilon10/SKYSTONE-CV/blob/19fc8b43450bd660614f579c4a368f628d192def/VisionPipeline.java#L150
            
            for(MatOfPoint contour : contoursList)
            {
                analyzeContour(contour, input);
            }
            
            /*MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contoursList.size()];
            Rect[] boundRect = new Rect[contoursList.size()];
            for (int i = 0; i < contoursList.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contoursList.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }*/
            
            // Iterate and check whether the bounding boxes
            // cover left and/or right side of the image
            double left_x = 420;
            double right_x = 520;
            
            double forward_diff_x = 20;
            
            if(contoursList.size() == 1)
            {
                Point[] points = contoursList.get(0).toArray();
                location_leftright = 0;
                
                if(points.length >= 4)
                {
                    top_left_x = points[0].x;
                    top_right_x = points[1].x;
                    bottom_left_x = points[2].x;
                    bottom_right_x = points[3].x;
                    
                    if(top_left_x < left_x)
                    {
                        location_leftright--;
                    }
                    if(bottom_left_x < left_x)
                    {
                        location_leftright--;
                    }
                    if(top_right_x < left_x)
                    {
                        location_leftright--;
                    }
                    if(top_right_x < left_x)
                    {
                        location_leftright--;
                    }
    
                    if(top_left_x > right_x)
                    {
                        location_leftright++;
                    }
                    if(bottom_left_x > right_x)
                    {
                        location_leftright++;
                    }
                    if(top_right_x > right_x)
                    {
                        location_leftright++;
                    }
                    if(top_right_x > right_x)
                    {
                        location_leftright++;
                    }
                }
            }
    
    /*
            input.copyTo(contoursOnPlainImageMat);
            for (int i = 0; i != boundRect.length; i++) {
                if (boundRect[i].x < left_x)
                    left = true;
                if (boundRect[i].x + boundRect[i].width > right_x)
                    right = true;
                
                // draw red bounding rectangles on mat
                // the mat has been converted to HSV so we need to use HSV as well
                //Imgproc.rectangle(hSVChanMat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
                Imgproc.rectangle(contoursOnPlainImageMat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
            }
    */
    
            switch (stageToRenderToViewport)
            {
                case HSV_CHAN:
                {
                    return hSVChanMat;
                }
    
                case THRESHOLD:
                {
                    return thresholdMat;
                }
                
                case MORPHED:
                    return morphedThreshold;
    
                case EDGE:
                {
                    return edgesMat;
                }
    
                //case CONTOURS_OVERLAYED_ON_FRAME:
                //{
                //    return contoursOnPlainImageMat;
                //}
    
                default: //RAW_IMAGE
                {
                    return input;
                }
            }
            
            //return input;
        }
    
        private int minX, minY = Integer.MAX_VALUE;
        private int maxX, maxY = -1 * Integer.MAX_VALUE;
        private void filterContours(List<MatOfPoint> contours, List<MatOfPoint> outputContours, double minContourArea, double minContourPerimeter, double minContourWidth,
                                    double minContourHeight)
        {
            //resetRectangle();
            Log.d("NumContours", contours.size() + "");
            for (MatOfPoint contour : contours)
            {
                Rect rect = Imgproc.boundingRect(contour);
                int x = rect.x;
                int y = rect.y;
                int w = rect.width;
                int h = rect.height;
    
                if (w < minContourWidth)
                    continue;
                if (rect.area() < minContourArea)
                    continue;
                if ((2 * w + 2 * h) < minContourPerimeter)
                    continue;
                if (h < minContourHeight)
                    continue;
                outputContours.add(contour);
    
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x + w > maxX) maxX = x + w;
                if (y + h > maxY) maxY = y + h;
            }
        }
        
        void analyzeContour(MatOfPoint contour, Mat input)
        {
            // Transform the contour to a different format
            Point[] points = new Point[4];
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
    
            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);
    
            rotatedRectFitToContour.points(points);
            //if(points.length >= 4)
            {
                top_left_x = points[0].x;
                top_right_x = points[1].x;
                bottom_left_x = points[2].x;
                bottom_right_x = points[3].x;
            }
        }
        
        public double top_left_x = 0;
        public double top_right_x = 0;
        public double bottom_left_x = 0;
        public double bottom_right_x = 0;
    
        void morphMask(Mat input, Mat output)
        {
            /*
             * Apply some erosion and dilation for noise reduction
             */
            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);
        
            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }
    
    
        static final Scalar RED = new Scalar(255, 0, 0);
        static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */
        
            Point[] points = new Point[4];
            rect.points(points);
        
            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    
        private Mat crop(Mat image, Point topLeftCorner, Point bottomRightCorner) {
            Rect cropRect = new Rect(topLeftCorner, bottomRightCorner);
            return new Mat(image, cropRect);
        }
        
        public int getLocation_leftright() {
            return this.location_leftright;
        }
    }
}