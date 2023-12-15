package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AttemptAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));



        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        Trajectory D12 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(50, 50), Math.toRadians(90))
                .build();

        drive.followTrajectory(D12);
//        telemetry.addData("Finished first move, pos : " , )
        sleep(2000);
        drive.followTrajectory((traj));
        sleep(2000);
        drive.followTrajectory(D12);
//        telemetry.addData("Finished first move, pos : " , )
        sleep(2000);
        drive.followTrajectory((traj));
        sleep(2000);drive.followTrajectory(D12);
//        telemetry.addData("Finished first move, pos : " , )
        sleep(2000);
        drive.followTrajectory((traj));
        sleep(2000);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(D12.end(), true)
//                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
//                        .build()
//        );

    }
}
