package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Class for controlling foundation hooks on robot. Includes
functions for teleop toggling and autonomous position setting. By Jake, Dec. 2019.
 */

public class ArmGripper {

    private Servo armGripper;
    public LinearOpMode l;
    public Telemetry realTelemetry;

    public enum servoPositions {
        CLOSED,
        OPEN
    }

    private servoPositions servo_pos;

    private boolean ButtonDown;

    @Config
    public static class armGripperConstants {

        public static double open_pos = 1;
        public static double closed_pos = 0.12;
    }

    public ArmGripper(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;

        armGripper = hardwareMap.servo.get("armGripperServo");

        armGripper.setDirection(Servo.Direction.REVERSE);
        armGripper.setPosition(armGripperConstants.closed_pos);
        servo_pos = servoPositions.CLOSED;

    }

    public void toggleHooks(boolean inputButton){
        if (inputButton && !ButtonDown) {
            switch (servo_pos) {
                case CLOSED:
                    realTelemetry.addLine("Opening");
                    armGripper.setPosition(armGripperConstants.open_pos);
                    servo_pos = servoPositions.OPEN;
                    ButtonDown = true;
                    break;
                case OPEN:
                    realTelemetry.addLine("Closing");
                    armGripper.setPosition(armGripperConstants.closed_pos);
                    servo_pos = servoPositions.CLOSED;
                    ButtonDown = true;
                    break;
            }
        }

        if (!inputButton) {
            ButtonDown = false;
        }
        realTelemetry.addData("Servo pos:", armGripper.getPosition());
    }

    public void open(){
        armGripper.setPosition(armGripperConstants.open_pos);
        servo_pos = servoPositions.OPEN;
    }

    public void close(){
        armGripper.setPosition(armGripperConstants.closed_pos);
        servo_pos = servoPositions.CLOSED;
    }

}
