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

    private double correctionInch = (95 / 60); // kms (Still untuned even with StraightTest, nothing is accurate)

    private void clawToggle(Boolean open) {
        if (open) {
            // open claw
        } else {
            // close claw
        }
    }

    private void armPosition(int deg) {
        //
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();


        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));


        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .forward(25 * correctionInch)
                .build();
        drive.followTrajectory(traj);
        sleep(2000);
        clawToggle(true); // open claw
//        armPosition(15); toggle hover mode
        sleep(2000);
        clawToggle(false); // close claw

        sleep(2000);
//        traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d())


//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();
//
//        Trajectory D12 = drive.trajectoryBuilder(new Pose2d())
//                .splineToConstantHeading(new Vector2d(50, 50), Math.toRadians(90))
//                .build();
//
//        drive.followTrajectory(D12);
////        telemetry.addData("Finished first move, pos : " , )
//        sleep(2000);
//        drive.followTrajectory((traj));
//        sleep(2000);
//        drive.followTrajectory(D12);
////        telemetry.addData("Finished first move, pos : " , )
//        sleep(2000);
//        drive.followTrajectory((traj));
//        sleep(2000);drive.followTrajectory(D12);
////        telemetry.addData("Finished first move, pos : " , )
//        sleep(2000);
//        drive.followTrajectory((traj));
//        sleep(2000);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(D12.end(), true)
//                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
//                        .build()
//        );

    }
}
