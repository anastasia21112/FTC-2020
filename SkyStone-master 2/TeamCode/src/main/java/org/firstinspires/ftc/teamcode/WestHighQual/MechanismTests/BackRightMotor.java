package org.firstinspires.ftc.teamcode.WestHighQual.MechanismTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Back Right Motor Test", group="Linear Opmode")
@Disabled
public class BackRightMotor extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BR;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BR = hardwareMap.dcMotor.get("Front Right");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            BR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        }
    }
}
