package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleGoBildaPinpointMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")

@Disabled
//@Disabled
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 360; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleGoBildaPinpointMecanumDriveCancelable drive = new SampleGoBildaPinpointMecanumDriveCancelable(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
