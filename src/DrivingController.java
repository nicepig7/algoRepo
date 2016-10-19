public class DrivingController {	
	






	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
		public String toString() {
			return steer+"/"+accel+"/"+brake+"/"+backward;
		}
	};
	boolean ISBRAKE = false;
	double BRAKE_SPEED = 0;
	
	final double BREAK_ABIL = 25;
	final double ALMOST_ZERO = 0.01;
	final double TRACK_LEN = 3185.83;
	
	double prevTfd = 0;
	
	// warning!! double corner?
	int CURRENT_SECTION = 0;
	int CORN_START = 0;
	int CORN_END = 0;
	double CORN_ANG = 0;
	double CORN_LEN = 0;
	double CORN_POS = 0;
	int NEXT_CORN_START = 0;
	int NEXT_CORN_END = 0;
	
	
	public DrivingCmd controlDriving(double[] driveArray, double[] aicarArray, double[] trackArray, double[] damageArray, int[] rankArray, int trackCurveType, double[] trackAngleArray, double[] trackDistArray, double trackCurrentAngle){
		DrivingCmd cmd = new DrivingCmd();
		
		////////////////////// input parameters
		double toMiddle     = driveArray[DrivingInterface.drvie_toMiddle    ];
		double angle        = driveArray[DrivingInterface.drvie_angle       ];
		double speed        = driveArray[DrivingInterface.drvie_speed       ];

		double toStart				 = trackArray[DrivingInterface.track_toStart		];
		double dist_track			 = trackArray[DrivingInterface.track_dist_track		];
		double track_width			 = trackArray[DrivingInterface.track_width			];
		double track_dist_straight	 = trackArray[DrivingInterface.track_dist_straight	];
		int track_curve_type		 = trackCurveType==1?-1:1;

		double[] tfa	= trackAngleArray;
		double[] tfd	= trackDistArray;
		//double track_current_angle		= trackCurrentAngle;
		
		double[] dist_cars = aicarArray;
		
		//double damage		 = damageArray[DrivingInterface.damage];
		//double damage_max	 = damageArray[DrivingInterface.damage_max];

		//int total_car_num	 = rankArray[DrivingInterface.rank_total_car_num	];
		//int my_rank			 = rankArray[DrivingInterface.rank_my_rank			];
		//int opponent_rank	 = rankArray[DrivingInterface.rank_opponent_rank	];		
		////////////////////// END input parameters
		
		// To-Do : Make your driving algorithm
		final int chkLen = 5;
		
		for( int i=1, ii=chkLen; i<ii ; i++ ) {
			if(tfd[chkLen-i] == prevTfd) {
				CURRENT_SECTION+=i;
				prevTfd = tfd[chkLen];
				break;
			}
		}
		
		//System.out.println("["+I++ +"]\t"+System.currentTimeMillis()%100000+ "\t"+(speed)+"\t "+toStart+"\t"+track_forward_dists[0]);
		
		
		
		// 변수 선언...
		double steer = 0;
		double accel = 1;
		double brake = 0;
		
		double fitPosition = 0;
		
		
		// 1. 현재 상태
		double targetSpeed = -1;
		double currAng = tfa[1]-tfa[0];
		double sectionLength = tfd[1]-tfd[0];
		sectionLength += sectionLength<0 ? TRACK_LEN : 0;
		
		// 1.1 직선주로
		if( isStraight(speed, track_dist_straight) ) {
			// 1.1.1 set fit position
			if( track_curve_type < 0 ) {
				fitPosition = 0.4*track_width;
			} else {
				fitPosition = -0.4*track_width;
			}
			
			// 1.1.2 reset corner
			/*CORN_START = 0;
			CORN_END = 0;
			CORN_ANG = 0;
			CORN_LEN = 0;/**/
			
			System.out.print(".."+CURRENT_SECTION+"\t");
			brake = 0;
			accel = 1;
		}
		// 1.2 앞에 코너 발견 - 감속구간
		else if( almostEqual(currAng, 0) ) {
			// 1.2.1 얼마까지 줄여야 하낭?
			if( ISBRAKE ) {
				targetSpeed = BRAKE_SPEED;
				
				System.out.print(" >"+CURRENT_SECTION+"\t");
				brake = getBrakeAmount(speed, targetSpeed);
				accel = getAccelAmount(speed, targetSpeed);
			} else {
				targetSpeed = calcNextSpeed(speed, toStart, tfa, tfd);
				if( NOT_CALCED != targetSpeed) {
					ISBRAKE = true;
					BRAKE_SPEED = targetSpeed;
					
					System.out.print("?>"+CURRENT_SECTION+"\t");
					brake = getBrakeAmount(speed, targetSpeed);
					accel = getAccelAmount(speed, targetSpeed);
				} else {
					System.out.print("?X"+CURRENT_SECTION+"\t");
					brake = 0;
					accel = 1;
				}
			}
			
			
		}
		// 1.3 코너링 중에서
		else {
			ISBRAKE = false;
			double diff = sectionLength;
			targetSpeed = angleToFitSpeed(currAng, diff);
			
			System.out.print(") "+CURRENT_SECTION+"\t");
			brake = getBrakeAmount(speed, targetSpeed);
			accel = getAccelAmount(speed, targetSpeed);
		}
		
		
		
		// 2. calc steer
		double fitDiff = toMiddle-fitPosition;
		double midDiffRatio = fitDiff/track_width;
		steer = 0.5*(angle-midDiffRatio);
		
		
		// x. current stat
		
		System.out.println(
				r(currAng)+"\t"+r(fitPosition)+"\t"+r(fitDiff)+"\t"+r(sectionLength)+"\t"+
				r(targetSpeed)+"\t"+r(speed)+"\t"+
				CORN_START+"\t"+CORN_END+"\t"+r(CORN_POS,1)+"\t"+r(CORN_ANG)+"\t"+r(CORN_LEN,1)+"\t"+
				r(accel)+"\t"+r(brake)+"\t"+r(steer));
		////////////////////// output values
		cmd.steer = steer;
		cmd.accel = accel;
		cmd.brake = brake;
		cmd.backward = DrivingInterface.gear_type_forward;
		////////////////////// END output values
		
		return cmd;
	}
	
	
	private static final int NOT_CALCED = -1;
	private double calcNextSpeed(double speed, double toStart, double[] tfa, double[] tfd) {
		double targetSpeed = NOT_CALCED;
		double[] tfDiffAngle = getTFDiffAngle(tfa);
		double currAng  = tfDiffAngle[0];
		
		// 1. calc next speed;
		for( int i=0, ii=tfDiffAngle.length ; i<ii ; i++ ) {
			double afterAng = tfDiffAngle[i];
			double distance = tfd[i] - toStart;
			
			if( almostEqual(currAng, afterAng)) {continue;}
			
			double afterSpeed = getFinalVelocity(distance, BREAK_ABIL, speed);
			double expSpeed = angleToFitSpeed(afterAng, tfd[i+1]-tfd[i]);
			
			if( afterSpeed > expSpeed ) {
				targetSpeed = expSpeed;
			}
			
			CORN_ANG = afterAng;
			
			break;
		}
		// 2. check secktion length;
		double sectionLen = 0;
		double prevAng = -1;
		double stPos = 0;
		CORN_START = CORN_END = 0;
		for(int i=0, ii=tfDiffAngle.length; i<ii ; i++ ) {
			double myAng = tfDiffAngle[i];
			if( !almostEqual(prevAng,myAng) ) {
				if( myAng == CORN_ANG ) {
					CORN_START = CURRENT_SECTION+i;
					CORN_POS = tfd[i];
				} else if( prevAng == CORN_ANG ) {
					break;
				}
			}
			if( almostEqual(myAng,CORN_ANG) ){
				CORN_END = CURRENT_SECTION+i;
				CORN_LEN = tfd[i] - CORN_POS;
			}
			prevAng = myAng;
		}
		
		// 3. corner length
		
		
		return targetSpeed;
	}
	
	
	
	
	
	private boolean almostEqual(double v1, double v2) {
		return Math.abs(v1 - v2) < ALMOST_ZERO;
	}
	
	private boolean isStraight(double speed, double track_dist_straight) {
		double afterSpeed = getFinalVelocity(track_dist_straight, BREAK_ABIL, speed);
		return afterSpeed <= 0;
	}

	double LOW_CORNER_SPEED = 15;
	
	private double getBrakeAmount(double speed, double targetSpeed) {
		double SMOOTH_SPEED_GAP = 5;
		if( speed < targetSpeed ) {return 0;}
		if( speed - SMOOTH_SPEED_GAP > targetSpeed ) {return 1;}
		return Math.sqrt((speed - targetSpeed)/SMOOTH_SPEED_GAP);
	}
	private double getAccelAmount(double speed, double targetSpeed) {
		double SMOOTH_SPEED_GAP = 5;
		//if( MAX_SPEED == targetSpeed ) { return 1;}
		if( targetSpeed < speed ) {return 0;}
		if( targetSpeed - SMOOTH_SPEED_GAP > speed ) {return 1;}
		return Math.sqrt((targetSpeed - speed)/SMOOTH_SPEED_GAP);
	}
	
	
	
	private double getFinalVelocity(double s, double a, double v) {
		if(v*v<2*s*a) {return 0;}
		double t = (v - Math.sqrt(v*v-2*s*a))/a;
		return v-a*t;
	}
	
	
	/**
	 *  F = M V^2 / R -> V = sqrt(R*C) (C = F/M)
	 * @param angle
	 * @return
	 */
	final double INFINITE_SPEED = 200;
	final double C = 17;
	private double angleToFitSpeed(double angle, double diff) {
		double absAngle = Math.abs(angle);
		if( absAngle < ALMOST_ZERO ) {return INFINITE_SPEED;}
		double r = diff / absAngle;
		return Math.sqrt(r*C);
	}
	
	
	
	/**
	 * track forward angle -> diff array : da[n] = a[n+1] - a[n]
	 * @param tfa
	 * @return
	 */
	private double[] getTFDiffAngle(double[] tfa) {
		int tfa_len = tfa.length;
		double[] tfDiffAngle = new double[tfa_len-1];
		for( int i=0, ii=tfa_len ; i<ii-1 ; i++ ) {
			tfDiffAngle[i] = tfa[i+1] - tfa[i];
		}
		return tfDiffAngle;
	}
	
	
	
	
	
	
	/**
	 * 반올림 하는 한수
	 * @param d 값
	 * @return
	 */
	private double r(double d) {
		return r(d,3);
	}
	
	private double r(double d, int p) {
		int SIZE = 1;
		for( ; p>0 ; p-- ) {SIZE*=10;}
		return (double)(Math.round(d*SIZE))/SIZE;
	}
	
	
	
	
	
	
	public static void main(String[] args) {
		DrivingInterface driving = new DrivingInterface();
		DrivingController controller = new DrivingController();
		
		double[] driveArray = new double[DrivingInterface.INPUT_DRIVE_SIZE];
		double[] aicarArray = new double[DrivingInterface.INPUT_AICAR_SIZE];
		double[] trackArray = new double[DrivingInterface.INPUT_TRACK_SIZE];
		double[] damageArray = new double[DrivingInterface.INPUT_DAMAGE_SIZE];
		int[] rankArray = new int[DrivingInterface.INPUT_RANK_SIZE];
		int[] trackCurveType = new int[1];
		double[] trackAngleArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackDistArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackCurrentAngle = new double[1];
				
		// To-Do : Initialize with your team name.
		int result = driving.OpenSharedMemory();
		
		if(result == 0){
			boolean doLoop = true;
			int i=0;
			while(doLoop){
				result = driving.ReadSharedMemory(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType, trackAngleArray, trackDistArray, trackCurrentAngle);
				switch(result){
				case 0:
					DrivingCmd cmd = controller.controlDriving(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType[0], trackAngleArray, trackDistArray, trackCurrentAngle[0]);
					driving.WriteSharedMemory(cmd.steer, cmd.accel, cmd.brake, cmd.backward);
					break;
				case 1:
					break;
				case 2:
					// disconnected
				default:
					// error occurred
					doLoop = false;
					break;
				}
			}
		}
	}
}