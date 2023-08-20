package org.firstinspires.ftc.teamcode.Pipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PipelineTesting.JunctionDetection;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//for dashboard
@Config
@Autonomous(name = "Junction Testing")
public class JunctionTesting extends LinearOpMode {
    OpenCvCamera camera;
    RobotConstants robot = new RobotConstants();
    public static double speed = .3;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, robot.junctionCamera), cameraMonitorViewId);

        JunctionDetection detector = new JunctionDetection(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 5);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addLine("what");
            telemetry.addData("Degree: ", detector.getDegree());
            telemetry.update();
        }
        camera.stopStreaming();
    }
}