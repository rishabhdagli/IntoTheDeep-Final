package org.firstinspires.ftc.teamcode.Detect;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


public class ObejectDectectionPipeline implements VisionProcessor, CameraStreamSource {

    //similar to an array list initialization
    // make the connection between why we need bit maps.

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private Bitmap bitmap;

    private Mat hsvMask = new Mat();

    public Point Center = new Point(0,0);

    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private Scalar lower = new Scalar(0, 100 , 0);
    private Scalar upper = new Scalar(30, 255, 255);


    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        //this line of code decides what we send to the phone
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public Bitmap getLastFrame() {
        return lastFrame.get();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);

        Imgproc.cvtColor(frame, hsvMask, Imgproc.COLOR_RGB2HSV);
        lower = new Scalar(ObjectDetectorOpmode.hueMin, ObjectDetectorOpmode.satMin, ObjectDetectorOpmode.valMin);
        upper = new Scalar(ObjectDetectorOpmode.hueMax, ObjectDetectorOpmode.satMax, ObjectDetectorOpmode.valMax);
        Core.inRange(hsvMask, lower, upper, hsvMask);
        Imgproc.GaussianBlur(hsvMask, hsvMask, new Size(11, 11), 0);


        switch (ObjectDetectorOpmode.state) {

            case (0): //Masking
                              /*
      1. converting the color space of the frame to HSV
      2. Apply Mask and Blur
      3. Convert the mat to a bit map
      4. send it to the atomic reference
      5. send to ftcdash
       */
                Utils.matToBitmap(hsvMask, bitmap);
                lastFrame.set(bitmap);


                break;

            case(1): //Camera viewing Largest Rect

                /*1. access the values of the static variables HSV from opmode
        2. create a new mat and make is instance variable
        3. convert that mat to hsv
        4. apply Core.Inrange with HSV values
        5. apply some blur and canny
        6.detect the largest contour
        7.draw abox put only show the box on frame
        8.convert frame with box to a bit map
        9.send to atomic reference.
       */


                contours.clear();

                Imgproc.GaussianBlur(hsvMask, hsvMask, new Size(11 + ObjectDetectorOpmode.blurIncrement*2, 11 + ObjectDetectorOpmode.blurIncrement*2), 0);
                Imgproc.Canny(hsvMask, hsvMask, 9, 9);

                Imgproc.findContours(hsvMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
                Imgproc.drawContours(hsvMask, contours, -1, new Scalar(255, 0, 0), 5);

                int widestX = 0, widestY = 0, widestW = 0, widestH = 0;

                for (MatOfPoint contour : contours) {
                    Rect box = Imgproc.boundingRect(contour);

                    int x = box.x;
                    int y = box.y;
                    int w = box.width;
                    int h = box.height;

                    //Imgproc.rectangle(frame, new Point(x, y), new Point(x + w, y + h), new Scalar(255, 0, 0), 2);


                    if (w > widestW) {
                        widestX = x;
                        widestY = y;
                        widestW = w;
                        widestH = h;
                    }


                }

                Imgproc.rectangle(frame, new Point(widestX, widestY), new Point(widestX+widestW, widestY+widestH),new Scalar(120,100,100), 5);
                Imgproc.line(frame,new Point(widestX + widestW/2.0,0),new Point(widestX + widestW/2.0,640),new Scalar(0,0,0),5);
                Imgproc.line(frame,new Point(0,widestY + widestH/2.0),new Point(640,widestY + widestH/2.0),new Scalar(0,0,0),5);

                Utils.matToBitmap(frame, bitmap);

                Center = new Point(widestX + widestW/2.0, widestY + widestH/2.0);

                lastFrame.set(bitmap);

                break;

        }//switch case end




        return null;

        } //process
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

}// End of the class




