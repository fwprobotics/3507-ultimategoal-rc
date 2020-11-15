package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Auto", group = "autonomous")
public class AutoBase extends LinearOpMode
{

    // CV Stuff
    public static int four_rings = 140;
    public static int one_ring = 128;
    public static int x = 181;
    public static int y = 98;
    OpenCvCamera webcam;
    RingDeterminationPipeline pipeline;
    RingPositions ringPos;

    // Actuators
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Arm arm;

    // Chad
    Integer cpr = 537;
    Integer gearratio = 1;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter);
    Double bias = 1.0;//adjust until your robot goes 20 inches
    Double meccyBias = 1.05;//change to adjust only strafing movement
    Double conversion = cpi * bias;
    Boolean exit = false;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    // Ring possibilities
    public enum RingPositions {
        ZERO, // A
        ONE, // B
        FOUR // C
    }

    @Config
    public static class AutoConstants
    {
        public static double fast_speed = 0.6;
        public static double slow_speed = 0.4;
        public static double turn_speed = 0.3;
    }

    @Config
    public static class ZeroAutoConstants
    {

        public static double a_forward_to_zone = 59.2;
        public static double b_turn_to_zone = -90;
        public static double c_drive_to_zone = 19.8;
        public static double d_turn_face_second_wobble = -90;
        public static double e_drive_to_second_wobble = 39.8;
        public static double f_turn_face_zone_2 = 165;
        public static double g_drive_to_zone_2 = 34.4;
        public static double h_strafe_park = 51.6;

        public static double z_wobble_backup = -3;
    }

    //@Config
    public static class OneAutoConstants
    {
        public static double a_forward_to_zone = 72;
        public static double b_strafe_from_zone = -27.2;
        public static double c_forward_to_wobble_2 = 50.2;
        public static double d_turn_to_face_zone_2 = -175;
        public static double e_forward_to_zone_2 = 51.2;
    }

    //@Config
    public static class FourAutoConstants
    {
        public static double a_forward_to_zone = 81.4;
        public static double b_strafe_to_zone = -32.0;
        public static double c_tap_up_to_zone = 9.8;
        public static double d_turn_to_face_wobbler = 175;
        public static double e_forward_to_wobbler_2 = 69;
        public static double f_turn_to_face_zone_2 = -175;
        public static double g_drive_up_to_zone_2 = 67.2;
        public static double h_back_to_park = -23.8;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initGyro();

        frontleft = hardwareMap.dcMotor.get("frontLeftDrive");
        frontright = hardwareMap.dcMotor.get("frontRightDrive");
        backleft = hardwareMap.dcMotor.get("backLeftDrive");
        backright = hardwareMap.dcMotor.get("backRightDrive");
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = new Arm(Arm.armRunMode.AUTONOMOUS, this, hardwareMap, telemetry);

        // Setting up CV
//        initCV();

        waitForStart();

        if (isStopRequested()) return;

        // MANUAL OVERRIDE FOR NOW
//         detectRings();
        ringPos = RingPositions.ZERO;

        /*

        BEGIN MOVEMENT

        */


        switch (ringPos) {

            case ZERO: // A

                // Drive forwards to zone
                moveToPosition(ZeroAutoConstants.a_forward_to_zone, AutoConstants.fast_speed);

                // Turn to face zone
                turnWithGyro(ZeroAutoConstants.b_turn_to_zone, AutoConstants.turn_speed);

                sleep(300);

                // Drive up to zone
                moveToPosition(ZeroAutoConstants.c_drive_to_zone, AutoConstants.slow_speed);

                sleep(300);

                // DROP WOBBLE
                // arm.autoDrive(100, 0.8);

                // Back up from wobble a bit
                moveToPosition(ZeroAutoConstants.z_wobble_backup, AutoConstants.fast_speed);

                // Bring wobble back down to ready position
                // arm.autoDrive(100, -0.8)

                // Turn to face second wobble
                turnWithGyro(ZeroAutoConstants.d_turn_face_second_wobble, AutoConstants.turn_speed);

                // Drive up to second wobble
                moveToPosition(ZeroAutoConstants.e_drive_to_second_wobble, AutoConstants.fast_speed);


                // GRAB SECOND WOBBLE
                // arm.autoDrive(100, 0.8)


                // Turn back to face zone
                turnWithGyro(ZeroAutoConstants.f_turn_face_zone_2, AutoConstants.turn_speed);

                moveToPosition(ZeroAutoConstants.g_drive_to_zone_2, AutoConstants.fast_speed); // Drive up to zone


                // DROP SECOND WOBBLE
                // arm.autoDrive(100, 0.8)

                sleep(300);

                // Back up a bit
                moveToPosition(ZeroAutoConstants.z_wobble_backup, AutoConstants.fast_speed);

                strafeToPosition(ZeroAutoConstants.h_strafe_park, AutoConstants.slow_speed); // Strafe onto line to park

                break;

            case ONE: // B

                // Drive forward to zone
                moveToPosition(OneAutoConstants.a_forward_to_zone, AutoConstants.fast_speed);

                sleep(300);

                // Drop wobble
                //arm.autoDrive(100, -0.8);

                // POSSIBLE - add tiny backup


                // Strafe away from zone to line up with second wobble
                strafeToPosition(OneAutoConstants.b_strafe_from_zone, AutoConstants.slow_speed);

                sleep(300);

                // Grab second wobbler
                // arm.autoDrive(100, 0.8);

                sleep(300);

                turnWithGyro(180, AutoConstants.turn_speed);

                moveToPosition(OneAutoConstants.c_forward_to_wobble_2, AutoConstants.slow_speed);

                // Flip back around to face zone
                turnWithGyro(OneAutoConstants.d_turn_to_face_zone_2, AutoConstants.turn_speed);

                moveToPosition(OneAutoConstants.e_forward_to_zone_2, AutoConstants.fast_speed);

                // Drop wobble goal
                // arm.autoDrive(100, 0.8);

                break;

          case FOUR: // C

              moveToPosition(FourAutoConstants.a_forward_to_zone, AutoConstants.fast_speed);

              strafeToPosition(FourAutoConstants.b_strafe_to_zone, AutoConstants.slow_speed);

              moveToPosition(FourAutoConstants.c_tap_up_to_zone, AutoConstants.slow_speed);


              sleep(300);

              // Drop wobbler
              //arm.autoDrive(100, 0.8);

              sleep(300);

              // Turn to face wobbler 2
              turnWithGyro(FourAutoConstants.d_turn_to_face_wobbler, AutoConstants.turn_speed);

              // Drive up to wobbler 2
              moveToPosition(FourAutoConstants.e_forward_to_wobbler_2, AutoConstants.slow_speed);

              sleep(300);
              // Grab wobbler 2
              // arm.autoDrive(100, -0.8);
              sleep(300);

              // Turn back to face zone
              turnWithGyro(FourAutoConstants.f_turn_to_face_zone_2, AutoConstants.turn_speed);

              moveToPosition(FourAutoConstants.g_drive_up_to_zone_2, AutoConstants.fast_speed);

              moveToPosition(FourAutoConstants.h_back_to_park, AutoConstants.fast_speed);

              break;

        } // END SWITCH
    } // END OP MODE





    /*
       _____   __
      / __\ \ / /
     | (__ \ V /
      \___| \_/

    */

    // Ring pipeline
    public static class RingDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the ring count
         */
        public enum RingCount
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x, y);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = four_rings;
        final int ONE_RING_THRESHOLD = one_ring;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingCount position = RingCount.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingCount.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingCount.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingCount.ONE;
            }else{
                position = RingCount.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    // Initialize and start streaming from the camera
    private void initCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240);
            }
        });

        // TODO Untested, maybe remove
        FtcDashboard.getInstance().startCameraStream(webcam, 15);
    }

    // Function to detect the rings
    private void detectRings() {

        RingDeterminationPipeline.RingCount ringPositionDetected = pipeline.position;

        if (ringPositionDetected == RingDeterminationPipeline.RingCount.FOUR) // Four rings
        {
            ringPos = RingPositions.FOUR;
        }
        else if (ringPositionDetected == RingDeterminationPipeline.RingCount.NONE) // No rings
        {
            ringPos = RingPositions.ZERO;
        }
        else // If there's one ring. I figure it's easier to distinguish between four and none,
             // so we might as well go four the two if it doesn't see four or none.
        {
            ringPos = RingPositions.ONE;
        }

        telemetry.clearAll();
        telemetry.addData("Ring pos:", ringPos);
        telemetry.update();

        // TODO Untested, maybe remove
        FtcDashboard.getInstance().stopCameraStream();
        webcam.stopStreaming();
        webcam.closeCameraDevice(); // Not sure quite what this does, may need to be removed
    }

    /*
       ___ _  _   _   ___
      / __| || | /_\ |   \
     | (__| __ |/ _ \| |) |
      \___|_||_/_/ \_\___/

    */

    // Forward/backward movement
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    // Gyro turn
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Strafe movement
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    // Gryo helper functions
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
}


