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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "XDRIVE", group = "")
public class XDRIVE extends LinearOpMode {

    private DcMotor rightRear, leftRear, rightFront, leftFront, thrower, armAngle;
    private OpenCvCamera camera;
    private CRServo armLength;
    private Servo armWrist;
    private Servo theClaw;
//    val lengthMin: Double = -0.41
//    val lengthMax: Double = 0.19

    double lengthMin = -.93;

    private boolean middle = false;
    public boolean right = false;
    double lengthMax = -.35;

    private int location = -1;

    private int width = 1280;

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
        armLength.setPower(extension);

        telemetry.addData("ARM LENGTH", armLength.getPower());
        telemetry.addData("ARM LENGTH - extensionVar", extension);

    }


    private void wristCon() {

        armWrist.setPosition(armWrist.getPosition() + ((gamepad2.right_stick_x / 350.)));
        armWrist.setPosition(armWrist.getPosition() + ((gamepad2.right_stick_x / 300.0)));
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webby"), cameraMonitorViewId);
        camera.setPipeline(new SamplePipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, 720, OpenCvCameraRotation.UPRIGHT);
            }

             @Override
             public void onError(int errorCode) {  }
        });

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
            telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
            telemetry.addData("location", location);
            telemetry.addData("middle", middle);
            telemetry.addData("right", right);
            telemetry.update();
        } camera.stopStreaming();

    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
//            Imgproc.rectangle(
//                    input,
//                    new Point(
//                            input.cols() / 4,
//                            input.rows() / camera
//                    new Point(
//                            input.cols() * (3f / 4f),
//                            input.rows() * (3f / 4f)),
//                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            if (mat.empty()) {
                location = 0;
                return input;
            }

            Scalar low = new Scalar(107, 77, 31);
            Scalar high = new Scalar(112, 84, 35);
            Mat thresh = new Mat();

            Core.inRange(mat, low, high, thresh);

            Mat edges = new Mat();
            Imgproc.Canny(thresh, edges, 100, 300);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }

            double left_x = 0.30 * width;
            double right_x = 0.70 * width;
            middle = false;
            right = false;
            for (int i = 0; i != boundRect.length; i++) {
                if (boundRect[i].x < left_x) {
                    middle = true;
                }
                if (boundRect[i].x * boundRect[i].width > right_x) {
                    right = true;
                }

                Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.9));
            }

            if (!middle) location = 1;
            else if (!right) location = 2;
            else location = 3;

            return mat;
        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *

             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                camera.pauseViewport();
            } else {
                camera.resumeViewport();
            }
        }
    }

}
