package org.firstinspires.ftc.teamcode.StateFinalCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RightLineBot Far End", group = "LinearOpMode")
public class RightLineBotFarEnd extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL;                              //declare OpMode mechanical members and their names used in the program
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor IL;
    private DcMotor IR;
    private DcMotor LiR;
    private DcMotor LiL;
    private Servo drag;

    @Override
    public void runOpMode()
    {
        //name motors
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        IL = hardwareMap.get(DcMotor.class, "IL");
        IR = hardwareMap.get(DcMotor.class, "IR");
        //configure motors
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        //wait for program to start
        waitForStart();
        runtime.reset();
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);

        sleep(25000);
        park(0.5, 2000);



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

        BR.setTargetPosition(distance);
        BL.setTargetPosition(distance);
        FR.setTargetPosition(distance);
        FL.setTargetPosition(distance);

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

    public void park(double power, int distance)
    {
        driveForwardEncoders(1.0, 2500);
        strafeRightEncoders(1.0, 1000);
/*
        turnLeftEncoders(1.0, 1200);
        driveForwardEncoders(1.0, 600);

 */
    }







}

