package org.firstinspires.ftc.teamcode.drive.opmode;

//import android.graphics.Color
//import com.qualcomm.robotcore.hardware.DcMotorEx

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "InconsistentForward", group = "")
public class InconsistentForward extends LinearOpMode {

    private DcMotor rightRear, leftRear, rightFront, leftFront, thrower, armAngle;

    private CRServo armLength;

    double overallDistanceModifier = 1.0;
    double accelerationRate = 0.01;
    double startRate = 0.02;
    double stopRate = 0.02;
    double theTurnAdjustor = 6.45;
    private Servo armWrist;
    private Servo theClaw;
//    val lengthMin: Double = -0.41
//    val lengthMax: Double = 0.19

    double lengthMin = -10.0;
    double lengthMax = 10.0;

    double stickX = 0.0;
    double stickY = 0.0;
    double speedFactor = 0.0;
    double pivot = 0.0;
    boolean suspendMotion = false;
    boolean susClicked = false;
    double susAngle = 0.0;
    boolean clawOn = false;
    boolean clawSwitch = false;

    private void moveForward(double distance, double initSpeed) {
        // frontLeft, frontRight, rearLeft, rearRight
        double distanceRatio = overallDistanceModifier;
        double actualDistance = distance * distanceRatio;
        double halfsies = actualDistance / 2;
        double pathingDistance;
        double distanceLeft;
        double speed = 0;

        int[] initialMotorPositions = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};
        double lowestPosition = 0.0;
        Boolean running = true;

        while (running) {
            pathingDistance = rightRear.getCurrentPosition() - initialMotorPositions[3];
            distanceLeft = actualDistance - pathingDistance;
            // old
            if (pathingDistance <= halfsies) {
                if (speed < initSpeed) {
                    speed = (startRate + (accelerationRate * pathingDistance));
                } else {
                    speed = initSpeed;
                }
            } else {
                if (((distanceLeft * accelerationRate) + stopRate) < speed) {
                    speed = ((distanceLeft * accelerationRate) + stopRate);
                } if (speed < stopRate) {
                    speed = stopRate;
                }
            }

            running = false;
            if (leftFront.getCurrentPosition() < (initialMotorPositions[0] + actualDistance)) {
                leftFront.setPower(speed);
                running = true;
            }
            if (rightFront.getCurrentPosition() < (initialMotorPositions[1] + actualDistance)) {
                rightFront.setPower(speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() < (initialMotorPositions[2] + actualDistance)) {
                leftRear.setPower(speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() < (initialMotorPositions[3] + actualDistance)) {
                rightRear.setPower(speed);
                running = true;
            }


        }
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

    }

    private void moveReverse(double distance, double speed) {
        // frontLeft, frontRight, rearLeft, rearRight
        double distanceRatio = overallDistanceModifier;
        distance = -distance;
        int[] initialMotorPositions = {leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftRear.getCurrentPosition(),rightRear.getCurrentPosition()};
        Boolean running = true;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() > (initialMotorPositions[0] + (distance * distanceRatio))) {
                leftFront.setPower(-speed);
                running = true;
            }
            if (rightFront.getCurrentPosition() > (initialMotorPositions[1] + (distance * distanceRatio))) {
                rightFront.setPower(-speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() > (initialMotorPositions[2] + (distance * distanceRatio))) {
                leftRear.setPower(-speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() > (initialMotorPositions[3] + (distance * distanceRatio))) {
                rightRear.setPower(-speed);
                running = true;
            }

        }
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

    }

    private void turnRight(double angle, double speed) {

        double turnRatio = theTurnAdjustor;
        int[] initialMotorPosition = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};
        boolean running = true;
        double offset = angle * turnRatio;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() < (offset + initialMotorPosition[0])) {
                leftFront.setPower(speed);
                running = true;
            }
            if (rightFront.getCurrentPosition() > (-offset + initialMotorPosition[1])) {
                rightFront.setPower(-speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() < (offset + initialMotorPosition[2])) {
                leftRear.setPower(speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() > (-offset + initialMotorPosition[3])) {
                rightRear.setPower(-speed);
                running = true;
            }
        }

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

    }

    private void turnLeft(double angle, double speed) {

        double turnRatio = theTurnAdjustor + 0.25;
        int[] initialMotorPosition = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};
        boolean running = true;
        double offset = angle * turnRatio;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() > (-offset + initialMotorPosition[0])) {
                leftFront.setPower(-speed);
                running = true;
            }
            if (rightFront.getCurrentPosition() < (offset + initialMotorPosition[1])) {
                rightFront.setPower(speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() > (-offset + initialMotorPosition[2])) {
                leftRear.setPower(-speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() < (offset + initialMotorPosition[3])) {
                rightRear.setPower(speed);
                running = true;
            }
        }

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

    }

    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        armAngle = hardwareMap.dcMotor.get("ANGLE0");
        armLength = hardwareMap.crservo.get("AL0");
        armWrist = hardwareMap.servo.get("AW1");
        theClaw = hardwareMap.servo.get("TC2");
        thrower = hardwareMap.dcMotor.get("THROWER");

        waitForStart();
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection((DcMotorSimple.Direction.REVERSE));

        thrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Double startingPosition = 0.0;
        armAngle.setTargetPosition(armAngle.getCurrentPosition()+250);
        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        moveForward(150,0.2);
        sleep(25000);
        moveForward(50,0.4);

    }

}
