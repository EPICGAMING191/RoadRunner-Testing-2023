package org.firstinspires.ftc.teamcode.RoadRunner.opmode.JackTestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Autonomous
public class RoadRunnerInRegularOpMode extends OpMode {
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
    private boolean moved = false;

    private SampleMecanumDrive drive;
    private Trajectory right;
    private Trajectory forward;
    private Trajectory back;
    private Trajectory left;

    @Override
    public void init() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        forward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE * 2)
                .build();
        back = drive.trajectoryBuilder(new Pose2d())
                .back(DISTANCE)
                .build();
        left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();
        right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        webcamName = getWebcamName("Webcam 1");
        aprilTagProcessor = createAprilTagProcessor();
        visionPortal = createVisionPortal(webcamName, aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            latestID = detection.id;
            aprilTagName = getAprilTagName(latestID);
            telemetry.addLine(String.valueOf(aprilTagName));
            poseX = detection.rawPose.x;
            poseY = detection.rawPose.y;
            poseZ = detection.rawPose.z;
            poseDetected = true;
            moveUsingID(drive,detection.id, left, right, forward, back);
        }
        //if(moved == false){
            //drive.followTrajectory(left);
            //drive.followTrajectory(forward);
            //drive.followTrajectory(right);
            //drive.followTrajectory(back);
            //drive.followTrajectory(left);
            //moved = true;
        //}

    }

    @Override
    public void loop() {
        visionPortal.stopStreaming();
    }

    public String getAprilTagName(int id) {
        String position = "";
        switch (id) {
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

    public WebcamName getWebcamName(String webcam_name) {
        return hardwareMap.get(WebcamName.class, webcam_name);
    }

    public AprilTagProcessor createAprilTagProcessor() {
        return AprilTagProcessor.easyCreateWithDefaults();
    }

    public VisionPortal createVisionPortal(WebcamName webcamName, AprilTagProcessor processor) {
        return VisionPortal.easyCreateWithDefaults(webcamName, processor);
    }
    public void moveUsingID(SampleMecanumDrive drive, int id, Trajectory left, Trajectory right, Trajectory forward, Trajectory back){
        switch (id){
            case 1:
            case 2:
            case 3:
                drive.followTrajectory(right);
                break;
            case 4:
            case 5:
            case 6:
                drive.followTrajectory(left);
            default:
                break;
        }
    }
}

