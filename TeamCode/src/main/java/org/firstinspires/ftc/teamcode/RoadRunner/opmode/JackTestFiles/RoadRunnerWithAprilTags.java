package org.firstinspires.ftc.teamcode.RoadRunner.opmode.JackTestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "RoadRunner")
public class RoadRunnerWithAprilTags extends LinearOpMode {
    public static double DISTANCE = 10; // in
    double poseX = 0.0;
    double poseY = 0.0;
    double poseZ = 0.0;
    List<AprilTagDetection> currentDetections;
    public String aprilTagName = "UNDEFINED";
    public int latestID = -1;
    public boolean poseDetected = false;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive(hardwareMap);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE * 2)
                .build();
        Trajectory back = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();
        Trajectory right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        webcamName = getWebcamName("Webcam 1");
        aprilTagProcessor = createAprilTagProcessor();
        visionPortal = createVisionPortal(webcamName, aprilTagProcessor);
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(left);
        drive.followTrajectory(forward);
        drive.followTrajectory(right);
        drive.followTrajectory(back);
        drive.followTrajectory(left);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addLine("FR Power: " + drive.rightFront.getPower());
            telemetry.addLine("BR Power: " + drive.rightRear.getPower());
            telemetry.addLine("FL Power: " + drive.leftFront.getPower());
            telemetry.addLine("BL Power: " + drive.leftRear.getPower());
            currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                latestID = detection.id;
                aprilTagName = getAprilTagName(latestID);
                telemetry.addLine(String.valueOf(aprilTagName));
                poseX = detection.rawPose.x;
                poseY = detection.rawPose.y;
                poseZ = detection.rawPose.z;
                poseDetected = true;
            }
        }
    }
    public String getAprilTagName(int id){
        String position = "";
        switch (id){
            case 1:
                position = "BLUE_BACKDROP_LEFT";
                break;
            case 2:
                position = "BLUE_BACKDROP_CENTER";
                break;
            case 3:
                position = "BLUE_BACKDROP_RIGHT";
                break;
            case 4:
                position = "RED_BACKDROP_LEFT";
                break;
            case 5:
                position = "RED_BACKDROP_CENTER";
                break;
            case 6:
                position = "RED_BACKDROP_RIGHT";
                break;
            default:
                position = "UNDEFINED";
                break;
        }
        return position;
    }
    public WebcamName getWebcamName (String webcam_name){
        return hardwareMap.get(WebcamName.class, webcam_name);
    }
    public AprilTagProcessor createAprilTagProcessor () {
        return AprilTagProcessor.easyCreateWithDefaults();
    }
    public VisionPortal createVisionPortal (WebcamName webcamName, AprilTagProcessor processor){
        return VisionPortal.easyCreateWithDefaults(webcamName, processor);
    }
}