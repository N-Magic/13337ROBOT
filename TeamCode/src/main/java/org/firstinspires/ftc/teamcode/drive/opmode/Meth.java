package org.firstinspires.ftc.teamcode.drive.opmode;

//import android.graphics.Color
//import com.qualcomm.robotcore.hardware.DcMotorEx

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Meth", group = "")
public class Meth extends LinearOpMode {

    private DcMotor rightRear, leftRear, rightFront, leftFront, thrower, armAngle;


    private CRServo armLength;
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



    private void moveForward(double distance, double speed) {
        // frontLeft, frontRight, rearLeft, rearRight
        double distanceRatio = 1.0;
        int[] initialMotorPositions = {leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftRear.getCurrentPosition(),rightRear.getCurrentPosition()};
        double lowestPosition = 0.0;
        double effectiveDistance = distance * distanceRatio;
        Boolean running = true;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() < effectiveDistance) {
                leftFront.setPower(speed);
                running = true;
            }
            if (rightFront.getCurrentPosition() < effectiveDistance) {
                rightFront.setPower(speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() < effectiveDistance) {
                leftRear.setPower(speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() < effectiveDistance) {
                rightRear.setPower(speed);
                running = true;
            }

        }
        leftFront.setPower(0.0);
        rightFront.setPower((0.0));
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
        moveForward((1225),(0.2));

        sleep(30000);

//        while (rightRear.getCurrentPosition() < (startingPosition + 1000)) {
//            rightFront.setPower(0.1);
//            leftFront.setPower(0.1);
//            rightRear.setPower(0.1);
//            leftRear.setPower(0.1);
//        }

//        while (rightFront < )

    }

}
