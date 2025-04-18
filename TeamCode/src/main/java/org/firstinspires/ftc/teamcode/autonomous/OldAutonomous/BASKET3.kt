//package org.firstinspires.ftc.teamcode.autonomous
//
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.Disabled
//import org.firstinspires.ftc.teamcode.Methods
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
//
//@Autonomous(name = "BASKET3", group = "AAAA")
//class BASKET3 : Methods() {
//    override fun runOpMode() {
//        initOdometry()
//        initMotors()
//        transferServo = hardwareMap.servo["Transfer"]
//        transferServo!!.position = transferServoClose
//        outClawServo = hardwareMap.servo["OutClaw"]
//        outClawServo!!.position = outClawOpen
//        outRotationServo = hardwareMap.servo["OutRotation"]
//        outRotationServo!!.position = outRotationCenter
//        outSwivelServo = hardwareMap.servo["OutSwivel"]
//        outSwivelServo!!.position = outSwivelPerpBack
//        inStopServo = hardwareMap.servo["InStop"]
//        inStopServo!!.position = inStopOpen
//        inRotationServo = hardwareMap.servo["InRotation"]
//        inRotationServo!!.position = inRotationUp
//
//        drive!!.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(90.00))
//
//
////        val bar: TrajectorySequence =
////            drive!!.trajectorySequenceBuilder( Pose2d(-34.09, -63.19, Math.toRadians(90.00)))
////                .setReversed(true)
////                .lineToConstantHeading(Vector2d(-10.04, -33.0))
////                .setReversed(false)
////                .build()
//
//        val sample1: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-34.09, -63.19,Math.toRadians(90.0)))
//                .lineToConstantHeading(Vector2d(-50.5, -42.5))
//                .build()
//
//        val basket1: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-50.5, -42.5, Math.toRadians(90.00)))
//                .setReversed(true)
//                .addTemporalMarker(0.5){outRotationServo!!.position = outRotationBackOut}
//                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
//                .setReversed(false)
//                .build()
//
//        val move: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-56.5, -56.5, Math.toRadians(90.00)))
//                .lineToLinearHeading(Pose2d(-63.5,-59.5,Math.toRadians(45.00)))
//                .build()
//
//        val sample2: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-63.5, -59.5, Math.toRadians(45.00)))
//                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
//                .addTemporalMarker(1.0){verticalSlideTo(-30,1.0)}
//                .lineToLinearHeading(Pose2d(-60.2, -42.5, Math.toRadians(90.00)))
//                .build()
//
//        val basket2: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-60.2, -42.5, Math.toRadians(90.00)))
//                .setReversed(true)
//                .addTemporalMarker(0.5){outRotationServo!!.position = outRotationBackOut}
//                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
//                .setReversed(false)
//                .build()
//
//        val sample3: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-63.5, -59.5, Math.toRadians(45.00)))
//                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
//                .addTemporalMarker(1.0){verticalSlideTo(-30,1.0)}
//                .lineToLinearHeading(Pose2d(-60.0, -46.5, Math.toRadians(118.00)))
//                .build()
//
//        val basket3: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-60.0, -46.5, Math.toRadians(118.00)))
//                .setReversed(true)
//                .addTemporalMarker(0.5) { outRotationServo!!.position = outRotationBackOut }
//                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
//                .setReversed(false)
//                .build()
//
//        val end: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-63.5, -59.5, Math.toRadians(45.00)))
//                .addTemporalMarker(0.3){verticalSlideTo(0,1.0)}
//                .addTemporalMarker(0.3){outRotationServo!!.position = outRotationCenter}
//                .lineTo(Vector2d(-36.0, -13.0))
//                .build()
//
//        waitForStart()
//        if (isStopRequested) {return}
//
//        //START
//        inRotationServo!!.position = inRotationPick
//
//        //SAMPLE1
//        drive!!.followTrajectorySequence(sample1)
//        outClawServo!!.position = outClawOpen
//        inRotationServo!!.position = inRotationPick
//        outSwivelServo!!.position = outSwivelPerpBack
//        inStopServo!!.position = inStopClose
//        sleep(100)
//        horizontalSlideTo(500,1.0)
//        intakeMotor!!.power = 0.7
//        sleep(1000)
//        inRotationServo!!.position = inRotationTransfer
//        sleep(100)
//        horizontalSlideTo(0,1.0)
//        inStopServo!!.position = inStopAutoOpen
//        sleep(1000)
//        outClawServo!!.position = outClawClose
//        sleep(300)
//        verticalSlideTo(verticalSlideHigh, 1.0)
//        intakeMotor!!.power = 0.0
//
//
//        //BASKET1
//        drive!!.followTrajectorySequence(basket1)
//        transferServo!!.position = transferServoOpen
//        sleep(800)
//        drive!!.followTrajectorySequence(move)
//        horizontalSlideTo(50,1.0)
//        intakeMotor!!.power = -0.7
//        sleep(500)
//        inRotationServo!!.position = inRotationPick
//        outClawServo!!.position = outClawOpen
//        sleep(200)
//        intakeMotor!!.power = 0.0
//
//        //SAMPLE2
//        drive!!.followTrajectorySequence(sample2)
//        transferServo!!.position = transferServoClose
//        outClawServo!!.position = outClawOpen
//        inRotationServo!!.position = inRotationPick
//        outSwivelServo!!.position = outSwivelPerpBack
//        inStopServo!!.position = inStopClose
//        sleep(100)
//        horizontalSlideTo(500,1.0)
//        intakeMotor!!.power = 0.7
//        sleep(1000)
//        inRotationServo!!.position = inRotationTransfer
//        sleep(50)
//        inStopServo!!.position = inStopAutoOpen
//        sleep(50)
//        horizontalSlideTo(0,1.0)
//        sleep(1300)
//        outClawServo!!.position = outClawClose
//        sleep(300)
//        intakeMotor!!.power = 0.0
//        verticalSlideTo(verticalSlideHigh, 1.0)
//
//        //BASKET2
//        drive!!.followTrajectorySequence(basket2)
//        transferServo!!.position = transferServoOpen
//        sleep(800)
//        drive!!.followTrajectorySequence(move)
//        horizontalSlideTo(50,1.0)
//        intakeMotor!!.power = -0.7
//        sleep(500)
//        inRotationServo!!.position = inRotationPick
//        outClawServo!!.position = outClawOpen
//        sleep(100)
//        intakeMotor!!.power = 0.0
//
//        //SAMPLE3
//        drive!!.followTrajectorySequence(sample3)
//        transferServo!!.position = transferServoClose
//        outClawServo!!.position = outClawOpen
//        outSwivelServo!!.position = outSwivelPerpBack
//        inRotationServo!!.position = inRotationPick
//        sleep(100)
//        horizontalSlideTo(750,1.0)
//        inStopServo!!.position = inStopClose
//        intakeMotor!!.power = 0.7
//        sleep(1000)
//        inRotationServo!!.position = inRotationTransfer
//        sleep(50)
//        inStopServo!!.position = inStopAutoOpen
//        sleep(50)
//        horizontalSlideTo(0,1.0)
//        sleep(1300)
//        outClawServo!!.position = outClawClose
//        sleep(300)
//        verticalSlideTo(verticalSlideHigh, 1.0)
//        intakeMotor!!.power = 0.0
//
//        //BASKET3
//        drive!!.followTrajectorySequence(basket3)
//        sleep(800)
//        drive!!.followTrajectorySequence(move)
//        horizontalSlideTo(0,1.0)
//        inRotationServo!!.position = inRotationPick
//        sleep(500)
//        outClawServo!!.position = outClawOpen
//        sleep(100)
//
//        //END
//        drive!!.followTrajectorySequence(end)
//    }
//}