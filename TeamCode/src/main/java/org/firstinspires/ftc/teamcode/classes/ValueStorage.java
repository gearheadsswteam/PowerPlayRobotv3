package org.firstinspires.ftc.teamcode.classes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import static com.qualcomm.robotcore.util.Range.*;
public class ValueStorage {
    public static int holderMinCount = 2;
    public static int signalMinCount = 10;
    public static double holderDetectionThreshold = 0.75;
    public static double odoUp = 0.22;
    public static double odoDown = 0.47;
    public static double rollerDown = 0.9;
    public static double rollerUp = 0.67;
    public static double rollerRetract = 0.25;
    public static double gripperHold = 0.48;
    public static double gripperRelease = 0.43;



    public static double [] armGrabPosition = {0.2,0.4};
    public static double [] armDropPosition = {0.2,0.4};


    public static double armRest = 0.10;
    public static double armUp = 0.53;
    public static double armIn = 0.41;
    public static double wristRest = 0.57;
    public static double wristUp = 0.11;
    public static double wristIn = 0.42;
    public static double armOffset = 1.01;
    public static double wristOffset = 1.10;
    public static double liftMaxAccel = 10000; //3000
    public static double liftMaxVel = 7000;  //1400
    public static double liftKp = 0.05;
    public static double liftKi = 0;
    public static double liftKd = 0;
    public static double armMaxVel = 0.65;
    public static double armMaxAccel = 5;
    public static double wristMaxVel = 2;
    public static double wristMaxAccel = 20;
    public static double[] liftLowClose = {175, armUp, wristUp};
    public static double[] liftMedClose = {600, armUp, wristUp};
    public static double[] liftHighClose = {1025, armUp, wristUp};
    public static double[] liftHighFar = {1100, 0.70, wristUp};
    public static double[] liftGroundClose = {0, 0.90, 0.20};
    public static double[] liftPickup = {0, 0.96, 0.20};
    public static final double INTAKE_POWER_AUTO = 0.7;
    public static final double INTAKE_POWER_TELEOP = 0.65;
    public static double[] adjust(double liftPos, double increment) {
        double weight;
        double groundInterval = 0.2;
        double closeInterval = 0.9;
        if (liftPos < liftLowClose[0]) {
            weight = scale(liftPos, 0, liftLowClose[0], 0, groundInterval);
        } else if (liftPos < liftHighClose[0]){
            weight = scale(liftPos, liftLowClose[0], liftHighClose[0], groundInterval, closeInterval);
        } else {
            weight = scale(liftPos, liftHighClose[0], liftHighFar[0], closeInterval, 1);
        }
        weight = clip(weight + increment, 0, 1);
        if (weight < groundInterval) {
            return new double[] {scale(weight, 0, groundInterval, 0, liftLowClose[0]),
                    scale(weight, 0, groundInterval, liftGroundClose[1], liftLowClose[1]),
                    scale(weight, 0, groundInterval, liftGroundClose[2], liftLowClose[2])};
        } else if (weight < closeInterval) {
            return new double[] {scale(weight, groundInterval, closeInterval, liftLowClose[0], liftHighClose[0]),
                    liftLowClose[1], liftLowClose[2]};
        } else {
            return new double[] {scale(weight, closeInterval, 1, liftHighClose[0], liftHighFar[0]),
                    scale(weight, closeInterval, 1, liftHighClose[1], liftHighFar[1]), liftHighClose[2]};
        }
    }
    public static double liftKf (double input) {
        return 0.0003 * input;
    }
    public static ProfileChain forwardArmProfile1(double t) {
        return new ProfileChain(new TrapezoidalProfile(armMaxVel, armMaxAccel, t + 0.5, armRest, 0, armUp, 0));
        //.addExtendTrapezoidal(armMaxVel, armMaxAccel, armUp, 0);
    }
    public static ProfileChain forwardWristProfile1(double t) {
        return new ProfileChain(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, t + 0.7, wristRest, 0, wristIn, 0))
                .addExtendTrapezoidal(wristMaxVel, wristMaxAccel, t + 1.2, wristUp, 0);
    }
    public static ProfileChain backArmProfile1(double t) {
        return new ProfileChain(new TrapezoidalProfile(armMaxVel, armMaxAccel, t, armUp, 0, armRest, 0));
    }
    public static ProfileChain backWristProfile1(double t) {
        return new ProfileChain(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, t, wristUp, 0, wristIn, 0))
                .addExtendTrapezoidal(wristMaxVel, wristMaxAccel, t + 0.5, wristRest, 0);
    }
    public static ProfileChain forwardArmProfile2(double t) {
        return new ProfileChain(new TrapezoidalProfile(armMaxVel, armMaxAccel, t, armIn, 0, armRest, 0));
    }
    public static ProfileChain forwardWristProfile2(double t) {
        return new ProfileChain(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, t + 0.3, wristIn, 0, wristRest, 0));
    }
    public static ProfileChain autonomousArmProfile(double t) {
        return new ProfileChain(new TrapezoidalProfile(armMaxVel, armMaxAccel, t, armIn, 0, armUp, 0));
    }
    public static ProfileChain autonomousWristProfile(double t) {
        return new ProfileChain(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, t + 0.3, wristIn, 0, wristUp, 0));
    }
    public static int side = sides.RED;
    public static class sides {
        public static final int RED = 1;
        public static final int BLUE = -1;
    }
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}