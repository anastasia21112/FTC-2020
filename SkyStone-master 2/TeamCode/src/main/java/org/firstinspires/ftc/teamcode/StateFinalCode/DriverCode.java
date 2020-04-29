package org.firstinspires.ftc.teamcode.StateFinalCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "DriverCode", group= "Linear Opmode")


public class DriverCode extends LinearOpMode {
    private ElapsedTime runTime = new ElapsedTime(); //timer for the runtime, standard line in every OpMode
    private DcMotor FL;                              //declare OpMode mechanical members and their names used in the program
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor IL;
    private DcMotor IR;
    private DcMotor OR;
    private DcMotor OL;
    private Servo BottomClaspLeft;
    private Servo TopClaspLeft;
    private Servo LifterLeft;
    private Servo BottomClaspRight;
    private Servo TopClaspRight;
    private Servo LifterRight;
    public double up = 0.0;
    public double down = 1.0;


    public void runOpMode()
    {
        //hardwareMap initialization with names that they are in the phone
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        IL = hardwareMap.get(DcMotor.class, "IL");
        IR = hardwareMap.get(DcMotor.class, "IR");

        OR = hardwareMap.get(DcMotor.class, "OR");
        OL = hardwareMap.get(DcMotor.class, "OL");

        BottomClaspLeft = hardwareMap.get(Servo.class, "BCL");
        TopClaspLeft = hardwareMap.get(Servo.class, "TCL");
        LifterLeft = hardwareMap.get(Servo.class, "LL");
        BottomClaspRight = hardwareMap.get(Servo.class, "BCR");
        TopClaspRight = hardwareMap.get(Servo.class, "TCR");
        LifterRight = hardwareMap.get(Servo.class, "LR");





        //sets direction of the motors, some are reversed because they are mirrored
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);

        //waits for the user to press play on the phone
        waitForStart();

        //resets the runtime timer
        runTime.reset();

        //while the opmode is active loop stays true for as long as the OpMode is running
        while(opModeIsActive())
        {
            //methods used for wheels, clipper (dragging thing), the linear slide intake, and the wheels on the servo
            //thank you for the abstraction tips fifi
            gamepadDrive();
            intake();
            grab();
            liftUp();


        }
    }
    /*
        Code for the drive base. We use mechanum drive. Refer to the diagram drawn in your math notebook.

     */

    public void gamepadDrive()
    {

        //sets the left motors to the power of the left joystick on the gamepad
        //sets the right motors to the power of the right joystick on the gamepad
        //the extra x axis on the left and right wheels enables mecanum drive
        //the plus and minus signs set the powers either positive or negative
        FL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
        BR.setPower((gamepad1.right_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)*0.5);
        FR.setPower((gamepad1.right_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
        BL.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)*0.5);
    }


    public void intake()
    {
        if(gamepad2.y)
        {
            IL.setPower(1.0);
            IR.setPower(-1.0);
            OR.setPower(1.0);
            OL.setPower(-1.0);
        }
        else if(gamepad2.a)
        {
            IL.setPower(-1.0);
            IR.setPower(1.0);
            OR.setPower(-1.0);
            OL.setPower(1.0);
        }
        else
        {
            IL.setPower(0.0);
            IR.setPower(0.0);
            OR.setPower(0.0);
            OL.setPower(0.0);
        }


    }
    /*
    public void outtake()
    {
       if(gamepad2.x)
       {
           OR.setPower(1.0);
           OL.setPower(-1.0);
       }
       else if(gamepad2.b)
       {
           OR.setPower(-1.0);
           OL.setPower(1.0);
       }
       else
       {
           OR.setPower(0.0);
           OL.setPower(0.0);
       }
    }

     */

    public void grab()
    {
        if(gamepad2.dpad_left)
        {
            TopClaspRight.setPosition(1.0);
            BottomClaspRight.setPosition(1.0);

            TopClaspLeft.setPosition(0.0);
            BottomClaspLeft.setPosition(0.0);
        }
        else if(gamepad2.dpad_right)
        {
            TopClaspRight.setPosition(0.0);
            BottomClaspRight.setPosition(0.0);

            TopClaspLeft.setPosition(1.0);
            BottomClaspLeft.setPosition(1.0);
        }
        else
        {

            TopClaspRight.setPosition(0.5);
            BottomClaspRight.setPosition(0.5);

            TopClaspLeft.setPosition(0.5);
            BottomClaspLeft.setPosition(0.5);


        }

        if(gamepad2.left_bumper)
        {
            TopClaspLeft.setPosition(1.0);
            BottomClaspLeft.setPosition(1.0);
        }else if(gamepad2.right_bumper)
        {
            TopClaspRight.setPosition(0.0);
            BottomClaspRight.setPosition(0.0);
        }else{

        }
    }



    public void liftUp()
    {
        if(gamepad2.dpad_up)
        {
            LifterLeft.setPosition(up);
            LifterRight.setPosition(down);

        }
        else if(gamepad2.dpad_down)
        {
            LifterLeft.setPosition(down);
            LifterRight.setPosition(up);
        }
        else
        {
            LifterLeft.setPosition(0.5);
            LifterRight.setPosition(0.5);
        }
    }









}
