package frc.robot.util;
import java.text.DecimalFormat;

import frc.robot.Constants;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class CalcPathToTarget {

    private Trajectory m_leftTrajectory;
    public Trajectory getLeftTrajectory(){
        return m_leftTrajectory;
    }
    private Trajectory m_rightTrajectory;
    public Trajectory getRightTrajectory(){
        return m_rightTrajectory;
    }

    private double m_xCenter, m_yCenter, m_xLeftCorner, m_yLeftCorner, m_xRightCorner, m_yRightCorner, m_hdg;

    VisionMath vm;
    CalcPathToTarget(){
        vm = new VisionMath();
    }

    private void calcArc(double outerRadiusInches, double degrees){//+degrees = turn right, -degrees = turn left
        double m_desiredDist = ((2*Math.PI*outerRadiusInches*Math.abs(degrees))/360) / 12.0;//feet
		double halfTime = Math.sqrt(m_desiredDist/Constants.kMaxAcceleration);
		int numIntervals = (int)(100*halfTime)+1;
		halfTime = numIntervals * 0.01;
        double m_acceleration = m_desiredDist / (halfTime*halfTime);
        numIntervals += 1;
        double deltaYaw = degrees / (numIntervals*2);

		Segment[] outerPath = new Segment[(numIntervals*2)];
		double xMax=0;
        double vMax=0;
        double lastYaw=0;
		for(int i = 0;i<numIntervals;i++){
			double v = m_acceleration * (i * 0.01);
            double x = 0.5 * m_acceleration * ((i * 0.01)*(i * 0.01));
            double yaw = lastYaw + deltaYaw;

			outerPath[i] = new Segment(0.01, 0, 0, x, v, m_acceleration, 0, (-yaw*Math.PI/180.0));
			xMax = x;
            vMax = v;
            lastYaw = yaw;
		}
		for(int i = 0;i<numIntervals;i++){
			double v = vMax - (m_acceleration * (i * 0.01));
			double x = xMax + (vMax*(i*0.01)) - (0.5 * m_acceleration * ((i * 0.01)*(i * 0.01)));
            double yaw = lastYaw + deltaYaw;

			outerPath[i+numIntervals] = new Segment(0.01, 0, 0, x, v, -m_acceleration, 0, (-yaw*Math.PI/180.0));
            lastYaw = yaw;
        }
        
        Segment[] innerPath = new Segment[outerPath.length];
        for(int i=0;i<outerPath.length;i++){
            double x = outerPath[i].position * (((outerRadiusInches / 12.0) - Constants.kWheelBaseFeet) / (outerRadiusInches / 12.0));
            double v = outerPath[i].velocity * (((outerRadiusInches / 12.0) - Constants.kWheelBaseFeet) / (outerRadiusInches / 12.0));
            double a = outerPath[i].acceleration * (((outerRadiusInches / 12.0) - Constants.kWheelBaseFeet) / (outerRadiusInches / 12.0));
            double yaw = outerPath[i].heading;
            innerPath[i] = new Segment(0.01, 0, 0, x, v, a, 0, yaw);
        }

        if(degrees < 0){
            //turn left
            m_rightTrajectory = new Trajectory(outerPath);
            m_leftTrajectory = new Trajectory(innerPath);
        }else{
            //turn right
            m_leftTrajectory = new Trajectory(outerPath);
            m_rightTrajectory = new Trajectory(innerPath);
        }
        /*
        DecimalFormat df = new DecimalFormat("#.##");
        System.out.print("LEFT PATH");
        for(int i=0;i<m_leftTrajectory.length();i++){
            Segment s = m_leftTrajectory.segments[i];
            System.out.println("x: "+df.format(s.position)+" v: "+df.format(s.velocity)+" a: "+df.format(s.acceleration)+" yaw: "+df.format(s.heading));
        }
        System.out.print("RIGHT PATH");
        for(int i=0;i<m_rightTrajectory.length();i++){
            Segment s = m_rightTrajectory.segments[i];
            System.out.println("x: "+df.format(s.position)+" v: "+df.format(s.velocity)+" a: "+df.format(s.acceleration)+" yaw: "+df.format(s.heading));
        }
        */
    }

    private void calcPrepStage(){
        double actualRadius = Constants.kMinimumRadiusTurn;
        
        double targetDist = (m_yCenter - Constants.kVisionApproachDist) / 2.0;
        double littleAngle = Math.atan((Constants.kMinimumRadiusTurn - m_xCenter) / targetDist);//radians
        double dashedLine = Math.sqrt((targetDist*targetDist) + ((Constants.kMinimumRadiusTurn - m_xCenter)*(Constants.kMinimumRadiusTurn - m_xCenter)));
        if(actualRadius > dashedLine * 0.9){
            actualRadius = dashedLine * 0.9;
        }
        double bigAngle = Math.asin(actualRadius / dashedLine);//radians
        double theta = Math.abs(bigAngle - littleAngle) * Math.signum(m_xCenter) * -1;

        //first arc

    }

    private void calcFinalStage(double x, double y){

    }
    
    public void calcPath(){
        vm.findRobotPos();
        double x = vm.getRobotX();
        double y = vm.getRobotY();

        double halfWheelBase = ((12* Constants.kWheelBaseFeet) / 2.0);//inches

        m_hdg = -Gyro.getYaw() * Math.PI / 180.0;
        m_xCenter = x + (Constants.kCamToBumper * Math.cos(m_hdg));
        m_yCenter = y - (Constants.kCamToBumper * Math.sin(m_hdg));
		m_xRightCorner =    m_xCenter - (halfWheelBase*Math.sin(m_hdg));
		m_xLeftCorner =     m_xCenter + (halfWheelBase*Math.sin(m_hdg));
        m_yRightCorner =    m_yCenter + Math.signum(Gyro.getYaw()) * ((halfWheelBase*Math.cos(m_hdg)));
        m_yLeftCorner =     m_yCenter - Math.signum(Gyro.getYaw()) * ((halfWheelBase*Math.cos(m_hdg)));

        double criticalHdg = Math.signum(m_xCenter) * -1.0 * Math.abs((Math.PI / 2.0) - Math.asin((Math.abs(m_yCenter) - Constants.kVisionApproachDist) / Math.abs(m_xCenter)));//radians
        if(criticalHdg > 0){
            if(m_hdg < criticalHdg){
                //need 2 stages
                
            }
        }else{
            if(m_hdg > criticalHdg){
                //need 2 stages

            }
        }



		double xGoal = -24;
		double yGoal = -Constants.kWheelBaseFeet*12/2.0;
		//double radiusTurn = Math.abs(m_xCorner - xGoal)/Math.tan(m_hdg) + (m_yCorner - yGoal);//inches
    }

}
