package org.firstinspires.ftc.teamcode.PipelineTesting;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class JunctionDetection extends OpenCvPipeline {
    Mat mat = new Mat();

    static int width = 640;
    static int height = 480;
    static double[] degree = new double[]{0.0};
    public static double PERCENT_COLOR_THRESHOLD = 0.4;
    public static double highHSVred = 50;
    public static double highHSVblue = 250;
    public static double highHSVgreen = 250;
    public static double lowHSVred = 0;
    public static double lowHSVblue = 50;
    public static double lowHSVgreen = 50;
    static double multiplier = 2.55;
    public static int boxes = 60;
    Telemetry telemetry;

    public JunctionDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        double incrementX = (double) width/boxes;
        double incrementY = (double) height/boxes;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(lowHSVred/2, lowHSVblue*multiplier, lowHSVgreen*multiplier);
        Scalar highHSV = new Scalar(highHSVred/2, highHSVblue*multiplier, highHSVgreen*multiplier); //Hue values of halved in openCV

        Core.inRange(mat, lowHSV, highHSV, mat);
        Scalar colorStone = new Scalar(255, 0, 0);
        ArrayList<Double> degrees = new ArrayList<>();
        degrees.clear();
        degree[0] = 0;
        double currentDegree = -90;
        //Adding "boxes" boxes to identify the angle of the junction infront of the camera.
        for (double i = incrementX; i <= width; i += incrementX){
            currentDegree += 180.0/boxes;
            Rect temp = new Rect(
                    new Point(i - incrementX, 0),
                    new Point(i, height)
            );
            double tempArea = temp.area();
            double threshold = Core.sumElems(mat.submat(temp)).val[0] / tempArea / 255;
            if (threshold > PERCENT_COLOR_THRESHOLD) {
                degrees.add(currentDegree);
                degree[0] += currentDegree;
            }
            Imgproc.rectangle(mat, temp, colorStone);
        }

        if(degree[0] != 0){
            degree[0] /= (double) degrees.size();
        }


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        telemetry.addData("degree", degree[0]);
        telemetry.update();

        return mat;
    }

    public double getDegree() {
        return degree[0];
    }

}