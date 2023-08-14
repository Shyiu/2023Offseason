package org.firstinspires.ftc.teamcode.Pipeline;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

//for dashboard
@Config
public class JunctionDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    static int width = 800;
    static int height = 448;
    static double degree;
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public JunctionDetection(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        double incrementX = (double) 800/180;
        double incrementY = (double) 448/180;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        ArrayList<Double> degrees = new ArrayList<>();
        degrees.clear();
        double currentDegree = -90;
        //Adding 180 boxes to identify the angle of the junction infront of the camera.
        for (double i = incrementX; i < width; i += incrementX){
            currentDegree += 1;
            Rect temp = new Rect( new Point(i - incrementX, 0),
                    new Point(incrementX, height));
            double tempArea = temp.area();
            double threshold = Core.sumElems(mat.submat(temp)).val[0] / tempArea / 255;
            if (threshold > PERCENT_COLOR_THRESHOLD) {
                degrees.add(currentDegree);
            }
        }
        degree = degrees.stream()
                .mapToDouble(a -> a)
                .sum();
        degree /= degrees.size();


//
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
//
//        Scalar colorStone = new Scalar(255, 0, 0);
//        Scalar colorSkystone = new Scalar(0, 255, 0);
//
//        Imgproc.rectangle(mat, TARGET, location == Location.TARGET ? colorSkystone:colorStone);
//        Imgproc.rectangle(mat, TOP_TARGET, location == Location.UP ? colorSkystone:colorStone);

        return mat;
    }

    public double getDegree() {
        return degree;
    }

}