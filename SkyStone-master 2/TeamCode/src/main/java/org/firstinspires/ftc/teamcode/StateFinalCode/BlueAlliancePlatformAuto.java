/* Team 16748
 * Lucas Erickson
 */

package org.firstinspires.ftc.teamcode.StateFinalCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class BlueAlliancePlatformAuto extends LinearOpMode {



    private DcMotor FL;                              //declare OpMode mechanical members and their names used in the program
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor IR;
    private DcMotor IL;
    private Servo BottomClaspLeft;
    private Servo TopClaspLeft;
    private Servo LifterLeft;
    private Servo BottomClaspRight;
    private Servo TopClaspRight;
    private Servo LifterRight;

    @Override
    public void runOpMode()
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        IL = hardwareMap.get(DcMotor.class, "IL");
        IR = hardwareMap.get(DcMotor.class, "IR");

        BottomClaspLeft = hardwareMap.get(Servo.class, "BCL");
        TopClaspLeft = hardwareMap.get(Servo.class, "TCL");
        LifterLeft = hardwareMap.get(Servo.class, "LL");
        BottomClaspRight = hardwareMap.get(Servo.class, "BCR");
        TopClaspRight = hardwareMap.get(Servo.class, "TCR");
        LifterRight = hardwareMap.get(Servo.class, "LR");

        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        lift(2000);
        driveBackwardEncoders(-0.4, 3300);
        grab(2000);
        driveForwardEncoders(1.0, 3000);
        turnLeftEncoders(1.0, 3000);
        driveBackwardEncoders(-1.0, 2500);
        lift(2000);
        driveForwardEncoders(1.0, 4000);




    }


    public void driveForward(double power)
    {
        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);
    }

    public void driveForwardEncoders(double power, int distance)
    {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(distance);
        BL.setTargetPosition(distance);
        FR.setTargetPosition(distance);
        FL.setTargetPosition(distance);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy())
        {
            driveForward(power);
        }
    }

    public void driveBackward(double power)
    {
        BR.setPower(-power);
        BL.setPower(-power);
        FL.setPower(-power);
        FR.setPower(-power);
    }

    public void driveBackwardEncoders(double power, int distance)
    {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(-distance);
        BL.setTargetPosition(-distance);
        FR.setTargetPosition(-distance);
        FL.setTargetPosition(-distance);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
        {
            driveBackward(power);
        }
    }

    public void turnRight(double power)
    {
        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(-power);
        FL.setPower(power);
    }

    public void turnRightEncoders(double power, int distance)
    {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(-distance);
        BL.setTargetPosition(distance);
        FR.setTargetPosition(-distance);
        FL.setTargetPosition(distance);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
        {
            turnRight(power);
        }
    }

    public void turnLeft(double power)
    {
        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(power);
        FL.setPower(-power);
    }

    public void turnLeftEncoders(double power, int distance)
    {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(distance);
        BL.setTargetPosition(-distance);
        FR.setTargetPosition(distance);
        FL.setTargetPosition(-distance);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
        {
            turnLeft(power);
        }
    }

    public void strafeLeft(double power)
    {
        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(-power);
    }

    public void strafeLeftEncoders(double power, int distance)
    {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(-distance);
        BL.setTargetPosition(distance);
        FR.setTargetPosition(distance);
        FL.setTargetPosition(-distance);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
        {
            strafeRight(power);
        }
    }

    public void strafeRight(double power)
    {
        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(-power);
        FL.setPower(power);
    }

    public void strafeRightEncoders(double power, int distance)
    {
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setTargetPosition(distance);
        BL.setTargetPosition(-distance);
        FR.setTargetPosition(-distance);
        FL.setTargetPosition(distance);

        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(BR.isBusy() && BL.isBusy() && FL.isBusy() && FR.isBusy())
        {
            strafeLeft(power);
        }
    }

    public void lift(int wait)
    {
        LifterLeft.setPosition(0.0);
        LifterRight.setPosition(1.0);
        sleep(wait);
        LifterLeft.setPosition(0.5);
        LifterRight.setPosition(0.5);
    }

    public void grab(int wait)
    {
        driveBackwardEncoders(0.3, 200);
        LifterLeft.setPosition(1.0);
        LifterRight.setPosition(0.0);
        TopClaspLeft.setPosition(0.5);
        BottomClaspLeft.setPosition(0.5);
        TopClaspRight.setPosition(0.5);
        BottomClaspRight.setPosition(0.5);
        sleep(wait);
        LifterLeft.setPosition(0.5);
        LifterRight.setPosition(0.5);
        TopClaspLeft.setPosition(0.5);
        BottomClaspLeft.setPosition(0.5);
        TopClaspRight.setPosition(0.5);
        BottomClaspRight.setPosition(0.5);
    }


}