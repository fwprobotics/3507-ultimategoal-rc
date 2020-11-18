package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmGripper;
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


@Config
@Autonomous(name = "Blue Auto", group = "autonomous")
public class Auto extends LinearOpMode
{

    // CV Stuff
    public static int four_rings = 130;
    public static int one_ring = 127;

    public static int x = 193;
    public static int y = 215;

    OpenCvCamera webcam;
    RingDeterminationPipeline pipeline;
    RingPositions ringPos;

    // Actuators
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Arm arm;
    ArmGripper armGripper;

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
    public static class AutoConstants {
        public static double fast_speed = 0.6;
        public static double slow_speed = 0.4;
        public static double turn_speed = 0.35;
    }

    //@Config
    public static class ZeroAutoConstants
    {

        public static double a_first_strafe = 5;
        public static double a_forward_to_zone = 67;
        public static double b_strafe_to_zone = -25;
        public static double c_strafe_to_wobble2 = -10;
        public static double d_turn_face_second_wobble = 180;
        public static double e_drive_to_second_wobble = -32;
        public static double f_turn_face_zone_2 = 165;
        public static double g_drive_to_zone_2 = 34.4;
        public static double h_strafe_park = 51.6;

        public static double z_wobble_backup = -3.5;

        public static double drop_arm_power = -0.5;
        public static int drop_arm_time = 650;

        public static double raise_arm_power = -0.2;
        public static int raise_arm_time = 200;
    }

    @Config
    public static class OneAutoConstants
    {
        public static double a_first_strafe = 7;
        public static double a_forward_to_zone = 89;
        public static double b_strafe_to_zone = -41;
        public static double c_park = -18;
    }

    @Config
    public static class FourAutoConstants
    {
        public static double a_first_strafe = 5;
        public static double a_forward_to_zone = 115;
        public static double b_strafe_to_zone = -27;
        public static double c_park = -45;

        public static double z_wobble_backup = -3.5; public static double drop_arm_power = -0.5; public static int drop_arm_time = 650; }


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
        armGripper = new ArmGripper(this, hardwareMap, telemetry);

        // Setting up CV
        initCV();

        waitForStart();

        if (isStopRequested()) return;

        detectRings();

        telemetry.addLine("Ready and waiting.");

        /*

        BEGIN MOVEMENT

        */


        switch (ringPos) {

            case ZERO: // A

                strafeToPosition(ZeroAutoConstants.a_first_strafe, AutoConstants.slow_speed);

                sleep(200);

                // Drive forwards to zone
                moveToPosition(ZeroAutoConstants.a_forward_to_zone, AutoConstants.fast_speed);

                sleep(500);

                // Strafe to face zone
                strafeToPosition(ZeroAutoConstants.b_strafe_to_zone, AutoConstants.slow_speed - 0.2);

                sleep(300);

                // DROP WOBBLE
                arm.autoDrive(ZeroAutoConstants.drop_arm_time, ZeroAutoConstants.drop_arm_power);
                sleep(200);
                armGripper.open();

                sleep(200);

                strafeToPosition(1.2, 0.3);
                moveToPosition(-2.2, AutoConstants.slow_speed - 0.1);

                sleep(300);

                // Back up from wobble a bit
                moveToPosition(ZeroAutoConstants.z_wobble_backup, AutoConstants.slow_speed - 0.1);

                sleep(100);

                moveToPosition(-4, AutoConstants.slow_speed - 0.1);

                break;

            case ONE: // B

                strafeToPosition(OneAutoConstants.a_first_strafe, AutoConstants.slow_speed - 0.1);

                sleep(200);

                // Drive forwards to zone
                moveToPosition(OneAutoConstants.a_forward_to_zone, AutoConstants.fast_speed);

                sleep(500);

                // DROP WOBBLE
                arm.autoDrive(ZeroAutoConstants.drop_arm_time, ZeroAutoConstants.drop_arm_power);
                sleep(200);
                armGripper.open();

                sleep(200);

                strafeToPosition(1.2, 0.3);
                moveToPosition(-2.2, AutoConstants.slow_speed - 0.1);

                sleep(300);

                // Back up from wobble a bit
                moveToPosition(ZeroAutoConstants.z_wobble_backup, AutoConstants.slow_speed - 0.1);

                sleep(100);

                moveToPosition(OneAutoConstants.c_park, AutoConstants.fast_speed);

                break;

          case FOUR: // C

              strafeToPosition(FourAutoConstants.a_first_strafe, AutoConstants.slow_speed);

              sleep(200);

              // Drive forwards to zone
              moveToPosition(FourAutoConstants.a_forward_to_zone, AutoConstants.fast_speed);

              sleep(500);

              // Strafe to face zone
              strafeToPosition(FourAutoConstants.b_strafe_to_zone, AutoConstants.slow_speed - 0.2);

              sleep(300);

              // DROP WOBBLE
              arm.autoDrive(ZeroAutoConstants.drop_arm_time, ZeroAutoConstants.drop_arm_power);
              sleep(200);
              armGripper.open();

              sleep(200);

              strafeToPosition(1.2, 0.3);
              moveToPosition(-2.2, AutoConstants.slow_speed - 0.1);

              sleep(300);

              // Back up from wobble a bit
              moveToPosition(ZeroAutoConstants.z_wobble_backup, AutoConstants.slow_speed - 0.1);

              sleep(100);

              moveToPosition(FourAutoConstants.c_park, AutoConstants.fast_speed);

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

        static final int REGION_WIDTH = 40;
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

        FtcDashboard.getInstance().startCameraStream(webcam, 15);
    }

    // Function to detect the rings
    private void detectRings() {

        sleep(2000);

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
        //telemetry.addData("tolerance", );
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


