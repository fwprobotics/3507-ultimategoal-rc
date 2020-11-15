package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Calibrate Chad Strafe", group="chad")
public class Calibrate_Chad_strafe extends LinearOpMode {
    //
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    Integer cpr = 537; //counts per rotation
    Integer gearratio = 1;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference

//  ________  __    __  __    __  ________        ________  __    __  ______   ______
// /        |/  |  /  |/  \  /  |/        |      /        |/  |  /  |/      | /      \
// $$$$$$$$/ $$ |  $$ |$$  \ $$ |$$$$$$$$/       $$$$$$$$/ $$ |  $$ |$$$$$$/ /$$$$$$  |
//    $$ |   $$ |  $$ |$$$  \$$ |$$ |__             $$ |   $$ |__$$ |  $$ |  $$ \__$$/
//    $$ |   $$ |  $$ |$$$$  $$ |$$    |            $$ |   $$    $$ |  $$ |  $$      \
//    $$ |   $$ |  $$ |$$ $$ $$ |$$$$$/             $$ |   $$$$$$$$ |  $$ |   $$$$$$  |
//    $$ |   $$ \__$$ |$$ |$$$$ |$$ |_____          $$ |   $$ |  $$ | _$$ |_ /  \__$$ |
//    $$ |   $$    $$/ $$ | $$$ |$$       |         $$ |   $$ |  $$ |/ $$   |$$    $$/
//    $$/     $$$$$$/  $$/   $$/ $$$$$$$$/          $$/    $$/   $$/ $$$$$$/  $$$$$$/


    Double meccyBias = 1.0; // Adjust until strafing 10in to the right


    //
    Double conversion = cpi * meccyBias;
    //
    public void runOpMode() {
        //
        frontleft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontright = hardwareMap.dcMotor.get("frontRightMotor");
        backleft = hardwareMap.dcMotor.get("backLeftMotor");
        backright = hardwareMap.dcMotor.get("backRightMotor");
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        strafeToPosition(10, .3); // Should strafe 10in to the right


    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
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
}
