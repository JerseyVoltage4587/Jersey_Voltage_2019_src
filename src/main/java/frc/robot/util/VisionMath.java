package frc.robot.util;
import frc.robot.Constants;

import java.util.Enumeration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionMath {

    private double zRobot = Constants.kZRobotInches;
    private double cameraRotation = Constants.kCameraRotation;

    private double m_xRobot,m_yRobot;
    public double getRobotX(){
        return m_xRobot;
    }
    public double getRobotY(){
        return m_yRobot;
    }
    public enum scoringZone{
        LEFT_ROCKET,
        RIGHT_ROCKET,
        CARGO_SIDES,
        CARGO_FRONT,
    }
    private scoringZone m_scoringZone = scoringZone.CARGO_FRONT;
    public void setScoringZone(scoringZone sZone){
        m_scoringZone = sZone;
    }
    private boolean m_haveCargo = false;
    public void setHaveCargo(boolean haveCargo){
        m_haveCargo = haveCargo;
    }
    private double m_g;
    public double getRotatedHdg(){
        return m_g;
    }

    public void findRobotPos(){
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
        NetworkTableEntry tve = limelightTable.getEntry("tv");
        double tv = tve.getDouble(0.0);
        if(tv == 0){
            //don't have target
            m_xRobot = 999;
            m_yRobot = 999;
            return;
        }

		NetworkTableEntry txe = limelightTable.getEntry("tx");
        double tx = txe.getDouble(0.0);
        tx *= (Math.PI/180.0);
		NetworkTableEntry tye = limelightTable.getEntry("ty");
        double ty = tye.getDouble(0.0) - 90;
        ty *= (Math.PI/180.0);
        
        double r = zRobot / (
            (Math.cos(ty)*Math.cos(cameraRotation))
            - (Math.sin(ty)*Math.cos(tx)*Math.sin(cameraRotation))
            );

        double xCam = r * (
            (Math.sin(ty)*Math.cos(tx)*Math.cos(cameraRotation))
            +(Math.cos(ty)*Math.sin(cameraRotation))
        );

        double yCam = r * (Math.sin(ty)*Math.sin(tx));

        m_g = Gyro.getYaw();
        switch(m_scoringZone){
            case CARGO_FRONT:
                //no change, 0 degrees
                break;
            case CARGO_SIDES:
                if(m_g<0){
                    //right side, -90 deg
                    m_g -= 90;
                }else{
                    //left side, +90 deg
                    m_g += 90;
                }
                break;
            case RIGHT_ROCKET:
                if(m_haveCargo){
                    //+90 deg
                    m_g += 90;
                }else{
                    //have hatch
                    if(Math.abs(m_g-Constants.kNearRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //near hatch
                        m_g+=Constants.kNearRocketHatchAngle;
                    }else if(Math.abs(m_g-Constants.kFarRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //far hatch
                        m_g+=Constants.kFarRocketHatchAngle;
                    }else{
                        //TODO flash LEDs "bad state"
                    }
                }
                break;
            case LEFT_ROCKET:
                if(m_haveCargo){
                    //-90 deg
                    m_g -= 90;
                }else{
                    //have hatch
                    if(Math.abs(m_g+Constants.kNearRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //near hatch
                        m_g-=Constants.kNearRocketHatchAngle;
                    }else if(Math.abs(m_g+Constants.kFarRocketHatchAngle)<Constants.kRocketVisionAngleTolerance){
                        //far hatch
                        m_g-=Constants.kFarRocketHatchAngle;
                    }else{
                        //TODO flash LEDs "bad state"
                    }
                }
                break;
        }
        m_g *= (Math.PI/180.0);//convert to radians for trig functions
        m_yRobot = (xCam * Math.sin(m_g)) + (yCam * Math.cos(m_g));
        m_xRobot = (xCam * Math.cos(m_g)) - (yCam * Math.sin(m_g));
    }
}
