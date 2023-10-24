package org.firstinspires.ftc.teamcode.RoadRunner.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "RoadRunner")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 10; // in

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
        }
    }
}