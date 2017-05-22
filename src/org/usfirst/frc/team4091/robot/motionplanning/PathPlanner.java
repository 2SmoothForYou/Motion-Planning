package org.usfirst.frc.team4091.robot.motionplanning;

import java.util.List;
import java.util.LinkedList;

public class PathPlanner {

	//Path vars
	public double[][] origPath;
	public double[][] nodeOnlyPath;
	public double[][] smoothPath;
	public double[][] leftPath;
	public double[][] rightPath;
	
	//Orig velocity
	public double[][] origCenterVelocity;
	public double[][] origLeftVelocity;
	public double[][] origRightVelocity;
	
	//Smooth velocity
	public double[][] smoothCenterVelocity;
	public double[][] smoothLeftVelocity;
	public double[][] smoothRightVelocity;
	
	//Accumulated heading
	public double[][] heading;
	
	double totalTime;
	double totalDistance;
	double numFinalPoints;
	
	double pathAlpha;
	double pathBeta;
	double pathTolerance;
	
	double velocityAlpha;
	double velocityBeta;
	double velocityTolerance;
	
	/* Constructor
	 * Path of waypoints defined as a double array of column vectors representing global cartesian points
	 * in the path in (x, y) coordinates, waypoints are traveled from one to the other in sequence.
	 */
	
	public PathPlanner(double[][] path){
		
		this.origPath = doubleArrayCopy(path);
		
		//Default values
		pathAlpha = .7;
		pathBeta = .3;
		pathTolerance = .0000001;
		
		velocityAlpha = .1;
		velocityBeta = .3;
		velocityTolerance = .0000001;
		
	}
	
	//Prints cartesian coordinates as column vectors
	public static void print(double[] path){
		
		System.out.println("X: \t Y:");
		
		for(double u:path){
			System.out.println(u);
		}
		
	}
	
	//Performs a deep copy of a double matrix
	public static double[][] doubleArrayCopy(double[][] arr){
		
		double[][] temp = new double[arr.length][arr[0].length];
		
		for(int i = 0; i < arr.length; i++){
			
			//Resize array in case of jagged array
			temp[i] = new double[arr[i].length];
			
			//Copy array over
			for(int j = 0; j < arr[i].length; j++){
				temp[i][j] = arr[i][j];
			}
		}
		
		return temp;
	}
	
	//Upsamples path by linear injection, provides more waypoints which gives a better result
	public double[][] inject(double[][] orig, int numToInject){
		
		double morePoints[][];
		
		//Create extended matrix to hold more data points
		morePoints = new double[orig.length + ((numToInject)*(orig.length-1))][2];
		
		int index = 0;
		
		//Loop through original array
		for(int i = 0; i < orig.length; i++){
			
			//Copy first
			morePoints[index][0] = orig[i][0];
			morePoints [index][1] = orig[i][1];
			index++;
			
			for(int j = 1; j < numToInject+1; j++){
				//Calculate intermediate x points between j and j+1 orig points
				morePoints[index][0] = j*((orig[i+1][0]-orig[i][0])/(numToInject+1))+orig[i][0];
				
				//Calculate intermediate y points between j and j+1 orig points
				morePoints[index][1] = j*((orig[i+1][1]-orig[i][1])/(numToInject+1))+orig[i][1];
				
				index++;
			}
			
		}
		
		//Copy last
		morePoints[index][0] = orig[orig.length-1][0];
		morePoints[index][1] = orig[orig.length-1][1];
		index++;
		
		return morePoints;
		
	}
	
	/*Optimization algorithm, which optimizes the data points to make a smoother trajectory
	 * This optimization uses gradient descent, while highly unlikely, it is possible for this algorithm to never converge
	 * If that ever happens, try increasing the tolerance level
	 * O(n^x), where x is the number of times the while loop iterates before tolerance is met.
	 */
	
	public double[][] smoother(double[][] path, double weightData, double weightSmooth, double tolerance){
		
		double[][] newPath = doubleArrayCopy(path);
		
		double change = tolerance;
		while(change >= tolerance){
			change = 0.0;
			for(int i = 1; i<path.length-1; i++){
				for(int j = 0; j<path[i].length; j++){
					double aux = newPath[i][j];
					newPath[i][j] += weightData * (path[i][j] - newPath[i][j]) + weightSmooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0*newPath[i][j]));
					change += Math.abs(aux-newPath[i][j]);
				}
			}
		}
		
		return newPath;
	}
	
	/*Reduces the path into only nodes which change direction, this allows the algorithm to know at which points the original waypoint vector changes
	 */
	public static double[][] nodeOnlyWayPoints(double[][] path){
		
		List<double[]> li = new LinkedList<double[]>();
		
		//Save first value
		li.add(path[0]);
		
		//Find intermediate nodes
		for(int i = 1; i < path.length-1; i++){
			
			//Calculate direction by getting the angle(direction) of the vector by converting the cartesian x y coordinates to polar r, theta coordinates and evaluating the arctangent of the resulting answer
			double vector1 = Math.atan2((path[i][1] - path[i-1][1]), path[i][0]-path[i-1][0]);
			double vector2 = Math.atan2((path[i+1][1]-path[i][1]), path[i+1][0]-path[i][0]);
			
			//Determine if both vectors have a change in direction
			if(Math.abs(vector2-vector1) >= .01){
				li.add(path[i]);
			}
			
		}
		
		//Save last
		li.add(path[path.length-1]);
		
		//Re-write nodes into new 2D array
		double[][] temp = new double[li.size()][2];
		
		for(int i = 0; i < li.size(); i++){
			temp[i][0] = li.get(i)[0];
			temp[i][1] = li.get(i)[1];
		}
		
		return temp;
		
	}
	
	/*
	 * Returns velocity as a double array, the first column vector is time, based on the timeStep, and the vector is the magnitude of the velocity
	 */
	double[][] velocity(double[][] smoothPath, double timeStep){
		double[] dxdt = new double[smoothPath.length];
		double[] dydt = new double[smoothPath.length];
		double[][] velocity = new double[smoothPath.length][2];
		
		//Set first instance to 0
		dxdt[0] = 0;
		dydt[0] = 0;
		velocity[0][0] = 0;
		velocity[0][1] = 0;
		heading[0][1] = 0;
		
		for(int i = 1; i < smoothPath.length; i++){
			dxdt[i] = (smoothPath[i][0]-smoothPath[i-1][0])/timeStep;
			dydt[i] = (smoothPath[i][1]-smoothPath[i-1][1])/timeStep;
			
			//Create time vector
			velocity[i][0] = velocity[i-1][0]+timeStep;
			heading[i][0] = heading[i-1][0]+timeStep;
			
			//Calculate velocity
			velocity[i][1] = Math.sqrt(Math.pow(dxdt[i],2)+Math.pow(dydt[i], 2));
		}
		
		return velocity;
		
	}
	
	/* Optimize velocity by minimizing error near end of travel
	 * when this function converges, the velocity will be smooth, start and end with a 0 velocity,
	 * and travel the same final distance as the original unsmoothed velocity profile
	 */
	double[][] velocityFix(double[][] smoothVelocity, double[][] origVelocity, double tolerance){
		
		/* Pseudo code
		 * Find error between the original velocity and the smooth velocity
		 * Keep increasing the velocity between the first and last node of the smooth velocity by a small amount
		 * Recalculate the difference, stop if threshold is met or repeat step 2 until the final threshold is met
		 * Return the updated smooth velocity
		 */
		
		//Calculate error difference
		double[] difference = errorSum(origVelocity, smoothVelocity);
		
		//Copy smooth velocity into new vector
		double[][] fixVel = new double[smoothVelocity.length][2];
		
		for(int i = 0; i < smoothVelocity.length; i++){
			fixVel[i][0] = smoothVelocity[i][0];
			fixVel[i][1] = smoothVelocity[i][1];
		}
		
		//Optimize velocity by minimizing error distance at end of travel
		//When this converges, the fixed velocity vector will be smooth, start
		//and end with 0 velocity, and travel the same final distance as the original 
		//un-smoothed velocity profile
		
		double increase = 0.0;
		while(Math.abs(difference[difference.length-1])>tolerance){
			increase = difference[difference.length-1]/1/50;
			for(int i = 1; i<fixVel.length-1;i++){
				fixVel[i][1] = fixVel[i][1]-increase;
			}
			difference = errorSum(origVelocity, fixVel);
		}
		
		return fixVel;
		
	}
	
	/*This method calculates the integral of the smooth velocity term and compares it to the integral
	 * of the original velocity term. Basically comparing the original velocity and the smooth velocity
	 * integrals to make sure the robot is still traveling the original distance
	 */
	private double[] errorSum(double[][] origVelocity, double[][] smoothVelocity){
		//Copy vectors
		double[] tempOrigDist = new double[origVelocity.length];
		double[] tempSmoothDist = new double[smoothVelocity.length];
		double[] difference = new double[smoothVelocity.length];
		
		double timeStep = origVelocity[1][0]-origVelocity[0][0];
		
		//Copy first elements
		tempOrigDist[0] = origVelocity[0][1];
		tempSmoothDist[0] = smoothVelocity[0][1];	
		
		//Calculate difference
		for(int i = 1; i<origVelocity.length; i++){
			tempOrigDist[i] = origVelocity[i][1]*timeStep+tempOrigDist[i-1];
			tempSmoothDist[i] = smoothVelocity[i][1]*timeStep+tempSmoothDist[i-1];
			
			difference[i] = tempSmoothDist[i]-tempOrigDist[i];
		}
		
		return difference;
	}
	
	/*This method calculates the optimal parameters for determining what amount of nodes to inject into the path
	 * to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
	 * results for the final smooth path.
	 */
	public int[] injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep){
		int first = 0;
		int second = 0;
		int third = 0;
		
		double oldPointsTotal = 0;
		
		numFinalPoints = 0;
		
		int[] ret = null;
		
		double totalPoints = maxTimeToComplete/timeStep;
		
		if(totalPoints < 100){
			double pointsFirst = 0;
			double pointsTotal = 0;
			
			for(int i = 4; i<=6; i++){
				for(int j = 1; j <= 8; j++){
					pointsFirst = i*(numNodeOnlyPoints-1) + numNodeOnlyPoints;
					pointsTotal = j*(pointsFirst-1)+pointsFirst;
					
					if(pointsTotal <= totalPoints && pointsTotal >= oldPointsTotal){
						first = i;
						second = j;
						numFinalPoints = pointsTotal;
						oldPointsTotal = pointsTotal;
					}
				}
				
				ret = new int[] {first, second, third};
				
			} 
		} else {
			
			double pointsFirst = 0;
			double pointsSecond = 0;
			double pointsTotal = 0;
			
			for(int i = 1; i<=5; i++){
				for(int j = 1; j<= 8; j++){
					for(int k = 1; k<8; k++){
						pointsFirst = i*(numNodeOnlyPoints-1) + numNodeOnlyPoints;
						pointsSecond = j*(pointsFirst-1)+pointsFirst;
						pointsTotal = k*(pointsSecond-1)+pointsSecond;
						
						if(pointsTotal <= totalPoints){
							first = i;
							second = j;
							third = k;
							numFinalPoints = pointsTotal;
						}
					}
				}
			}
			
			ret = new int[] {first, second, third};
			
		}
		
		return ret;
		
	}
	
	public void leftRight(double[][] smoothPath, double robotTrackWidth){
		
		double[][] leftPath = new double[smoothPath.length][2];
		double[][] rightPath = new double[smoothPath.length][2];
		
		double[][] gradient = new double[smoothPath.length][2];
		
		for(int i = 0; i<smoothPath.length-1; i++){
			gradient[i][1] = Math.atan2(smoothPath[i+1][1] - smoothPath[i][1], smoothPath[i+1][0]-smoothPath[i][0]);
		}
		
		gradient[gradient.length-1][1] = gradient[gradient.length-2][1];
		
		for(int i = 0; i < gradient.length; i++){
			leftPath[i][0] = (robotTrackWidth/2*Math.cos(gradient[i][1])+Math.PI/2)+smoothPath[i][0];
			leftPath[i][1] = (robotTrackWidth/2*Math.sin(gradient[i][1])+Math.PI/2)+smoothPath[i][1];
			
			rightPath[i][0] = (robotTrackWidth/2*Math.cos(gradient[i][1])-Math.PI/2)+smoothPath[i][0];
			rightPath[i][0] = (robotTrackWidth/2*Math.sin(gradient[i][1])-Math.PI/2)+smoothPath[i][1];
			
			//Convert to degrees 0-360 where 0 degrees is in the positive x direction, accumulated to aline with WPI sensor
			double deg = Math.toDegrees(gradient[i][1]);
			
			gradient[i][1] = deg;
			
			if(i > 0){
				if((deg-gradient[i-1][1]) > 180){
					gradient[i][1] = -360+deg;
				}
				if((deg-gradient[i-1][1]) < -180){
					gradient[i][1] = 360+deg;
				}
			}
		}
		
		this.heading  = gradient;
		this.leftPath = leftPath;
		this.rightPath = rightPath;
		
	}
	
	/*
	 * Returns the first column of a 2D array of doubles 
	 */
	public static double[] getXVector(double[][] arr){
		double[] temp = new double[arr.length];
		
		for(int i = 0; i < temp.length; i++){
			temp[i] = arr[i][0];	
		}
		
		return temp;
	}
	
	/*
	 * Returns the second column of a 2D array of doubles
	 */
	public static double[] getYVector(double[][] arr){
		double[] temp = new double[arr.length];
		for(int i = 0; i < temp.length; i++){
			temp[i] = arr[i][1];
		}
		
		return temp;
	}
	
	public static double[][] transposeVector(double[][] arr){
		double[][] temp = new double[arr[0].length][arr.length];
		
		for(int i = 0; i < temp.length; i++){
			for(int j = 0; j < temp[i].length; j++){
				temp[i][j] = arr[j][i];
			}
		}
		
		return temp;
	}
	
	public void setPathAlpha(double alpha){
		pathAlpha = alpha;
	}
	
	public void setPathBeta(double beta){
		pathBeta = beta;
	}
	
	public void setPathTolerance(double tolerance){
		pathTolerance = tolerance;
	}
	
	/*
	 * This code will calculate a smooth path based on the program parameters, if the user does
	 * not set any parameters, the default values will be used, which are optimized for more situations.
	 * The results will be saved into corresponding class members, they can then be accessed by
	 * .smoothPath, .leftPath, .rightPath, .smoothCenterVelocity, .smoothLeftVelocity, and .smoothRightVelocity
	 * 
	 * After calling this method, you only need to pass .smoothLeftVelocity[1] and .smoothRightVelocity[1] to the corresponding
	 * speed controllers for each side, and step through each step point
	 * 
	 */
	public void calculate(double totalTime, double timeStep, double robotTrackWidth){
		/*
		 * Pseudo code:
		 * Reduce input waypoints to only essential(direction changing) node points
		 * Calculate how many datapoints are needed to satisfy the controller for playback
		 * Inject and smooth path until the path is smooth and has the required number of data points
		 * Calculate left and right wheel paths
		 */
		
		//Find only direction changing nodes
		nodeOnlyPath = nodeOnlyWayPoints(origPath);
		
		//Figure out how many nodes to inject
		int[] inject = injectionCounter2Steps(nodeOnlyPath.length, totalTime, timeStep);
		
		//Iteratively inject and smooth the path
		for(int i = 0; i < inject.length; i++){
			if(i == 0){
				smoothPath = inject(nodeOnlyPath, inject[0]);
				smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
			} else {
				smoothPath = inject(nodeOnlyPath, inject[i]);
				smoothPath = smoother(smoothPath, .1, .3, .0000001);
			}
		}
		
		//Calculate left and right path based on center path
		leftRight(smoothPath, robotTrackWidth);
		
		origCenterVelocity = velocity(smoothPath, timeStep);
		origLeftVelocity = velocity(leftPath, timeStep);
		origRightVelocity = velocity(rightPath, timeStep);
		
		//Copy smooth velocities into fix velocities
		smoothCenterVelocity = doubleArrayCopy(origCenterVelocity);
		smoothLeftVelocity =  doubleArrayCopy(origLeftVelocity);
		smoothRightVelocity =  doubleArrayCopy(origRightVelocity);
		
		//Set final vel to 0
		smoothCenterVelocity[smoothCenterVelocity.length-1][1] = 0.0;
		smoothLeftVelocity[smoothLeftVelocity.length-1][1] = 0.0;
		smoothRightVelocity[smoothRightVelocity.length-1][1] = 0.0;
		
		//Smooth velocity with zero final V
		smoothCenterVelocity = smoother(smoothCenterVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothLeftVelocity = smoother(smoothLeftVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothRightVelocity = smoother(smoothRightVelocity,velocityAlpha, velocityBeta, velocityTolerance);
		
		//Fix velocity distance error
		smoothCenterVelocity = velocityFix(smoothCenterVelocity, origCenterVelocity, 0.0000001);
		smoothLeftVelocity = velocityFix(smoothLeftVelocity, origLeftVelocity, 0.0000001);
		smoothRightVelocity = velocityFix(smoothRightVelocity, origRightVelocity, 0.0000001);
		
	}
	
}
