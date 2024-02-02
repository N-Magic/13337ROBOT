package org.firstinspires.ftc.teamcode.drive;

//import android.graphics.Color
//import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor
//import org.firstinspires.ftc.robotcore.external.JavaUtil
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "XDRIVE", group = "")
public class XDRIVE extends LinearOpMode {

    private DcMotor rightRear, leftRear, rightFront, leftFront, thrower, armAngle;


    private CRServo armLength;
    private Servo armWrist;
    private Servo theClaw;
//    val lengthMin: Double = -0.41
//    val lengthMax: Double = 0.19

    double lengthMin = -.93;
    double lengthMax = -.35;


    double stickX = 0.0;
    double stickY = 0.0;
    double speedFactor = 0.0;
    double pivot = 0.0;
    boolean suspendMotion = false;
    boolean susClicked = false;
    double susAngle = 0.0;
    boolean clawOn = false;
    boolean clawSwitch = false;


    private void moveCon() {

        stickX = gamepad1.left_stick_x / 2.0;
        stickY = gamepad1.left_stick_y / 2.0;
        speedFactor = (gamepad1.right_trigger + 0.0);
        if (speedFactor <= 0.3) {
            speedFactor = 0.3;
        }
        pivot = 0.0;
        if (gamepad1.right_stick_x >= 0.15 || gamepad1.right_stick_x <= -0.15) {
            pivot = ((gamepad1.right_stick_x + 0.0) * speedFactor);
        } else {
            pivot = 0.0;
        }


        rightFront.setPower((((stickX + stickY) * speedFactor) + pivot));
        rightRear.setPower((((-stickX + stickY) * speedFactor) + pivot));
        leftFront.setPower((((-stickX - stickY) * speedFactor) + pivot));
        leftRear.setPower((((stickX - stickY) * speedFactor) + pivot));



    }


    private void armCon() {

        if (suspendMotion) {
            if (clawOn) {
                if (armAngle.getCurrentPosition() < (susAngle + 120)) {
                    armAngle.setPower(0.6);
                } else {
                    armAngle.setPower(0.0);
                }
            } else {
                if (armAngle.getCurrentPosition() < susAngle) {
                    armAngle.setPower(0.2);
                } else {
                    armAngle.setPower(0.0);
                }
            }
        } else {
            armAngle.setPower((gamepad2.right_stick_y / 2.0));
        }

        if (gamepad2.right_bumper) {
            if (!susClicked) {
                susAngle = (armAngle.getCurrentPosition() + 0.0);
                suspendMotion = !suspendMotion;
                susClicked = true;
            }
        } else {
            susClicked = false;
        }

        telemetry.addData("ARM ANGLE", armAngle.getCurrentPosition());
        telemetry.addData("SUSPEND", suspendMotion);
        telemetry.addData("SUS CLICKED", susClicked);
        telemetry.addData("SUSANGLE", susAngle);

    }


    private void clawCon() {

        if (gamepad2.left_bumper) {
            if (!clawSwitch) {
                clawOn = !clawOn;
                clawSwitch = true;
            }
        } else {
            clawSwitch = false;
        }

        theClaw.setDirection(Servo.Direction.REVERSE);
        theClaw.setPosition(theClaw.getPosition() + ((gamepad2.left_stick_x / 300.0)));

        telemetry.addData("Claw Direction", theClaw.getDirection());
        telemetry.addData("Claw Position", theClaw.getPosition());
        telemetry.addData("NA   Hover", clawOn);
        telemetry.addData("NA   Hover switch", clawSwitch);

    }


    private void extensionCon() {

        double extension = armLength.getPower() + (gamepad2.left_stick_y / 350.0);
        if ((extension >= lengthMin) && (extension <= lengthMax)) {
            armLength.setPower(extension);
        }

        telemetry.addData("ARM LENGTH", armLength.getPower());

    }


    private void wristCon() {

        armWrist.setPosition(armWrist.getPosition() + ((gamepad2.right_stick_x / 350.)));

        telemetry.addData("WRIST ANGLE", armWrist.getPosition());

    }

    private void throwerCon() {

        if (gamepad1.left_bumper) {
            thrower.setPower(.1);
        } else if (gamepad1.right_bumper) {
            thrower.setPower(-.1);
        } else {
            thrower.setPower(0.0);
        }
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

        thrower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            moveCon();
            armCon();
            wristCon();
            extensionCon();
            clawCon();
            throwerCon();
            telemetry.update();

        }

    }

}
