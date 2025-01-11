package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public interface Auto {
    public abstract TrajectorySequence buildTrajectorySequence(DriveShim drive, Pose2d startPose);
}
