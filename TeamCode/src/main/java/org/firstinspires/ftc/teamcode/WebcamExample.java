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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
    StageSwitchingPipeline stageSwitchingPipeline;
    
    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        stageSwitchingPipeline = new StageSwitchingPipeline();
        webcam.setPipeline(stageSwitchingPipeline);
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
            
            telemetry.addData("Num contours found", stageSwitchingPipeline.getNumContoursFound());
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
    static class StageSwitchingPipeline extends TimestampedOpenCvPipeline //OpenCvPipeline
    {
        Point stageTextAnchor;
        Point timeTextAnchor;
        
        Scalar green = new Scalar(0,255,0,255);
        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 80;  // 102
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
    
        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_CHAN_IDX = 2;
    
        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    
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
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();
    
    
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
    
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
            YCbCr_CHAN2,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }
        
        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
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
            
            // hull
            
            // filter out small hulls
            
            // detect convex hull shape
            
            // object boundary rect
            
            // left, center, right position calculation
            
            //https://gist.github.com/razimgit/d9c91edfd1be6420f58a74e1837bde18
            
            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }
                
                case THRESHOLD:
                {
                    return thresholdMat;
                }
                
                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    ///Draw a simple box around the middle 1/2 of the entire frame
                    Imgproc.rectangle(
                            contoursOnPlainImageMat,
                            new Point(
                                    input.cols()/4,
                                    input.rows()/4),
                            new Point(
                                    input.cols()*(3f/4f),
                                    input.rows()*(3f/4f)),
                            new Scalar(0, 255, 0), 4);
                    return contoursOnPlainImageMat;
                }
                
                case RAW_IMAGE:
                {
                    return input;
                }
                
                default:
                {
                    return input;
                }
            }
        }
    
        ArrayList<MatOfPoint> findContours(Mat input)
        {
            contoursList.clear();
            
            // A list we'll be using to store the contours we find
            //ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        
            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, CB_CHAN_IDX);
        
            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            morphMask(thresholdMat, morphedThreshold);
        
            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            //Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
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
            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = yCbCrChan2Mat.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = yCbCrChan2Mat.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = yCbCrChan2Mat.submat(new Rect(region3_pointA, region3_pointB));
            
            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];
    
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
    
            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
    
            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
    
    
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
    }
}