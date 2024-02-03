package org.firstinspires.ftc.teamcode.drive.opmode;

//import android.graphics.Color
//import com.qualcomm.robotcore.hardware.DcMotorEx

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Mashle", group = "")
public class Mashle extends LinearOpMode {

    private DcMotor rightRear, leftRear, rightFront, leftFront, thrower, armAngle;

    private CRServo armLength;
    private int armAngleNumber;
    private double armLengthNumber, wristNumber, clawNumber;

    private final double overallDistanceModifier = 17.4438;
    private final double accelerationRate = 0.0005;
//    private final double deAccelerationRate = 0.0002;
    private final double startRate = 0.08;
    private final double stopRate = 0.07;
    private final double theTurnAdjustor = 6.4;
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

    private void moveForward(double inputGoalDistance, double maxSpeed) {
        // frontLeft, frontRight, rearLeft, rearRight
        double goalDistance = overallDistanceModifier * inputGoalDistance;
        double halfOfGoalDistance = goalDistance / 2;
        double distanceLeft;
        double speed = 0;
        double distanceTravelled;

        int[] initialMotorPositions = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};
        double lowestPosition = 0.0;
        boolean running = true;

        while (running) {
            armControl();
            distanceTravelled = rightRear.getCurrentPosition() - initialMotorPositions[3];
            distanceLeft = goalDistance - distanceTravelled;
            // old
            if (distanceLeft >= halfOfGoalDistance) {
                if (speed < maxSpeed) {
                    speed = (startRate + (accelerationRate * distanceTravelled));
                } else {
                    speed = maxSpeed;
                }
            } else {

                if (((distanceLeft * accelerationRate) + stopRate) < speed) {
                    if ((distanceLeft*accelerationRate) + stopRate > stopRate) {
                        speed = ((distanceLeft*accelerationRate) + stopRate);
                    } else {
                        speed = (stopRate);
                    }
                }
            }

            running = false;
            if (leftFront.getCurrentPosition() < (initialMotorPositions[0] + goalDistance)) {
                leftFront.setPower(speed);
                running = true;
                armControl();
            } else {
                leftFront.setPower(0.0);
            }
            if (rightFront.getCurrentPosition() < (initialMotorPositions[1] + goalDistance)) {
                rightFront.setPower(speed);
                running = true;
            } else {
                rightFront.setPower(0.0);
            }
            if (leftRear.getCurrentPosition() < (initialMotorPositions[2] + goalDistance)) {
                leftRear.setPower(speed);
                running = true;
            } else {
                leftRear.setPower(0.0);
            }
            if (rightRear.getCurrentPosition() < (initialMotorPositions[3] + goalDistance)) {
                rightRear.setPower(speed);
                running = true;
            } else {
                rightRear.setPower(0.0);
            }
            armControl();


        }
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

    }

    private void armControl() {
        armAngle.setTargetPosition(armAngleNumber);
        if (armAngle.getCurrentPosition()<armAngleNumber) armAngle.setPower(0.1);
        if (armAngle.getCurrentPosition()>armAngleNumber) armAngle.setPower(-0.2);
//        armLength.setPower(armLengthNumber);
        armWrist.setPosition(wristNumber);
        theClaw.setPosition(clawNumber);
    }

    private void moveReverse(double distance, double initSpeed) {
        // frontLeft, frontRight, rearLeft, rearRight

        double distanceRatio = overallDistanceModifier;
        double actualDistance = distance * distanceRatio;
        double halfsies = actualDistance / 2;
        double pathingDistance;
        double distanceLeft;
        double speed = 0;

        int[] initialMotorPositions = {leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftRear.getCurrentPosition(),rightRear.getCurrentPosition()};
        boolean running = true;
        while (running){
            armControl();
            pathingDistance = rightRear.getCurrentPosition() - initialMotorPositions[3];
            distanceLeft = actualDistance - pathingDistance;
            // old
            if (distanceLeft >= halfsies) {
                if (speed < initSpeed) {
                    speed = (startRate + (accelerationRate * -pathingDistance));
                } else {
                    speed = initSpeed;
                }
            } else {
                if (((distanceLeft * accelerationRate) + stopRate) < speed) {
                    speed = (((pathingDistance-actualDistance) * accelerationRate) + stopRate);
                } if (speed < stopRate) {
                    speed = stopRate;
                }
            }

            running = false;
            if (leftFront.getCurrentPosition() > (initialMotorPositions[0] + (-distance * distanceRatio))) {
                leftFront.setPower(-speed);
                running = true;
            }
            if (rightFront.getCurrentPosition() > (initialMotorPositions[1] + (-distance * distanceRatio))) {
                rightFront.setPower(-speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() > (initialMotorPositions[2] + (-distance * distanceRatio))) {
                leftRear.setPower(-speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() > (initialMotorPositions[3] + (-distance * distanceRatio))) {
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
            armControl();
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
            armControl();
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

//    private void setClaw(boolean open, double angle) {
//
//        //
//
//    }
//
//    private void setClaw(boolean open) {
//
//        //
//
//    }
//
//    private void setArm(double length, double wristAngle) {
//
//        if (length >= -1 && length <= -0.355) {
//
//        }
//
//    }

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
        thrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        armAngleNumber = 277;
        clawNumber = 0.0;
        wristNumber = .61;
        for ( int i = 0; i < 400; i++ ) {
            armControl();
            sleep(1);
        } armAngle.setPower(0);

        armAngleNumber = 277;
        wristNumber = 0.847;
        for ( int i = 0; i < 100; i++ ) {
            armControl();
            sleep(1);
        } armAngle.setPower(0);

        armAngleNumber = -370;
        clawNumber = 0.0;
        for ( int i = 0; i < 250; i++ ) {
            armControl();
            sleep(1);
        } armAngle.setPower(0);

        waitForStart();
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection((DcMotorSimple.Direction.REVERSE));


        moveForward(80,0.6); // 32in

        clawNumber = 1.0;
        for ( int i = 0; i < 50; i++ ) {
            armControl();
            sleep(1);
        } armAngle.setPower(0);

        armAngleNumber = 1000;
        for ( int i = 0; i < 1000; i++ ) {
            armControl();
            sleep(1);
        } armAngle.setPower(0);



//        armLengthNumber = armLength
        armControl();

//        armAngleNumber = 1000;
//        wristNumber = .86;
//        clawNumber = 1.0;

        armControl();
        moveReverse(40,0.6); // 16in
        sleep(500);
        turnRight(95,0.2);
        sleep(500);
        moveForward(40,0.4);
        turnLeft(93,0.2);
        sleep(500);
        moveForward(120,0.6);
        turnLeft(93,0.2);
        sleep(500);
        moveForward(292, 0.4); // 120in

//>>>>>>> 189afb40d929945503e3cc787b66983b1c5bc56b

//        sleep(25000);
//        moveForward(50,0.4);

    }

}
