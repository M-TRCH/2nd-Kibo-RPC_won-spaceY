package jp.jaxa.iss.kibo.rpc.defaultapk;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.*;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import net.sourceforge.zbar.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import static org.opencv.android.Utils.matToBitmap;

//      [Team member: 1.Tee:Tula 2.Earth:Teerachot 3.Rew:Siraphop]                                                                       lll         lll
//                                                                								                                             lll       lll
//      lll      lllll      lll lllllllllllllll llllll      lll         llllllllllll llllllllllll    lllll       llllllllllll llllllllllll lll     lll
//      lll     lll lll     lll lll         lll lll lll     lll         lll          lll      lll   lll lll      lll          lll           lll   lll
//       lll   lll   lll   lll  lll         lll lll  lll    lll         lll          lll      lll  lll   lll     lll          lll            lll lll
//       lll   lll   lll   lll  lll         lll lll   lll   lll lllllll llllllllllll llllllllllll lllllllllll    lll          llllllllllll    lllll
//        lll lll     lll lll   lll         lll lll    lll  lll                  lll lll         lll       lll   lll          lll              lll
//        lll lll     lll lll   lll         lll lll     lll lll                  lll lll        lll         lll  lll          lll              lll
//         lllll       lllll    lllllllllllllll lll      llllll         llllllllllll lll       lll           lll llllllllllll llllllllllll     lll

public class YourService extends KiboRpcService
{
	int Pattern = 0;	//	Store pattern value.
	boolean QRCodeFinish	= false;	//	State of QR event.
	final Quaternion Zero_Quaternion	= new Quaternion(0, 0, 0, 0);		//	No rotate quaternion
	Quaternion Quaternion_Target 		= new Quaternion(0, 0, 0, 0);		//	Quaternion of target point.
	final Point Point_A 	= new Point(10.73, -9.8, 4.79);		//	Point of A
	final Point Point_B 	= new Point(10.6, -8.0, 4.5);			//	Point of B
	Point Point_A_Prime 	= new Point(0, 0, 0);					//	Point of A prime
	Point Point_Target 		= new Point(11.21, -10.585, 5.375);	//	Default Point of Target.

	protected void runPlan1()
		/* Main plan */ {
		/* -> */	Log.i("Robot[Stage_1]: ", "Preparing to start");
		getQuaTargetPoint();	//	Calculate quaternion for rotate to QR tag position.
		/* -> */	Log.i("Robot[Stage_2]: ", "Starting the mission");
		api.startMission();
		/* -> */	Log.i("Robot[Stage_3]: ", "Laser is on");
		api.laserControl(true);
		/* -> */	Log.i("Robot[Stage_4]: ", "QR Code scanning");
		QR_Discover();
		/* -> */	Log.i("Robot[Stage_5]: ", "Get the target point");
		getTargetPosition();
		/* -> */	Log.i("Robot[Stage_6]: ", "Aim to target point");
		moveTo(Point_A, Quaternion_Target, 3);
		/* -> */	Log.i("Robot[Stage_7]: ", "Take a snapshot");
		delay(3000);	api.takeSnapshot();
		/* -> */	Log.i("Robot[Stage_8]: ", "Move to B point");
		moveTo(Point_B, Zero_Quaternion, 2);
		/* -> */	Log.i("Robot[Stage_9]: ", "Mission completed");
		finishMission();
	}
	protected void runPlan2()
		/* Plan 2 */ {
	}
	protected void runPlan3()
		/* Plan 3 */ {
	}
	public double[] getVector(double[] p, double[] q)
		/* Finding a vector between of position (input two position array). */ {
		return new double[]{p[0]-q[0], p[1]-q[1], p[2]-q[2]};
	}
	public double[] getRoot(double a, double b, double c)
		/* Get the root of 2 degree equation (input coefficient a, b, c). */ {
		double InSqrt = Math.pow(b, 2)-4*a*c;
		return new double[]
				{
						(-b+Math.sqrt(InSqrt))/2*a,
						(-b-Math.sqrt(InSqrt))/2*a
				};
	}
	public double getVal(double[] p)
		/* Get the value of vector (input array of vector). */ {
		return Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
	}
	public double[] getQua(double[] p, double[] q)
		/* Convert 2 vector to quaternion (input two array of vector). */{
		double [] n = vecCross(p,q);
		double [] r = {p[0]-q[0],p[1]-q[1],p[2]-q[2]};
		double [] v =
				{
						Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2])),
						Math.sqrt((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])),
						Math.sqrt((r[0]*r[0])+(r[1]*r[1])+(r[2]*r[2])),
						Math.sqrt((n[0]*n[0])+(n[1]*n[1])+(n[2]*n[2]))
				};
		double [] u = {n[0]/v[3], n[1]/v[3], n[2]/v[3]};
		double theta =  Math.acos(((v[0]*v[0])+(v[1]*v[1])-(v[2]*v[2]))/(2*v[0]*v[1]));
		return new double[]
				{
						Math.cos(theta/2),
						Math.sin(theta/2)*u[0],
						Math.sin(theta/2)*u[1],
						Math.sin(theta/2)*u[2]
				};
	}
	public double[] vecCross(double[] p, double[] q)
		/* Cross product of 3D vector (input 2 array of vector). */ {
		return new double[]
				{
						p[1]*q[2]-p[2]*q[1],
						p[2]*q[0]-p[0]*q[2],
						p[0]*q[1]-p[1]*q[0]
				};
	}
	public void delay(long sleep)
		/* sleep function */ {
		try
		{
			Thread.sleep(sleep);
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
	}
	public void moveTo(Point point, Quaternion quaternion, int max_count)
		/* Re-check api.moveTo function is successful or until max number of counter. */ {
		Result result;
		int count = 0;
		do
		{
			result = api.moveTo(point, quaternion, false);
			count++;
		}
		while (!result.hasSucceeded() && count < max_count);
	}
	public void finishMission()
		/* Re-check api.reportMissionCompletion, The cause is sometimes unresponsive.*/ {
		Log.d("reportMissionCompletion[State]: ", "Starting");
		boolean ReportFinish = false;
		int loopCount = 0;
		while(!ReportFinish)
		{
			try
			{
				ReportFinish = api.reportMissionCompletion();
			}
			catch (Exception error)
			{
				Log.e("reportMissionCompletion[State]: ", "Failure");
				Log.e("reportMissionCompletion[Error]: ", error.getMessage());
			}
			Log.d("reportMissionCompletion[Count]: ", "" + loopCount);
			loopCount++;
		}
		Log.d("reportMissionCompletion[State]: ", "Finished");
	}
	public double[] StringParseToDouble(String contents)
		/* Split the data obtained from the QR code into double numbers. */ {
		Log.d("StringParseToDouble[State]: ", "Starting");
		Log.d("StringParseToDouble[Input]: ", "" + contents);
		double pattern = 0, point_x = 0, point_y = 0, point_z = 0;
		try
		{
			// Assume : {"p":5,"x":11.07,"y":-9.80,"z":5.49}
			String contents_cut = contents.substring(1, 35);      // Result > p":5,"x":11.07,"y":-9.80,"z":5.49
			String[] commaParse = contents_cut.split(",");  // Result > "p":5 , "x":11.07 , "y":-9.80 , "z":5.49
			String[] pParse = commaParse[0].split(":");     // Result > "p" , 5
			String[] xParse = commaParse[1].split(":");     // Result > "x" , 11.07
			String[] yParse = commaParse[2].split(":");     // Result > "y" , -9.80
			String[] zParse = commaParse[3].split(":");     // Result > "z" , 5.49

			// String to double value convert.
			pattern = Double.parseDouble(pParse[1]);
			point_x = Double.parseDouble(xParse[1]);
			point_y = Double.parseDouble(yParse[1]);
			point_z = Double.parseDouble(zParse[1]);
			Log.d("StringParseToDouble[Info]: ", "p: " + pattern + ", x: " + point_x + ", y: " + point_y + ", z: " + point_z);
		}
		catch (Exception error)
		{
			Log.e("StringParseToDouble[State]: ", "Failure");
			Log.e("StringParseToDouble[Error]: ", error.getMessage());
		}
		Log.d("StringParseToDouble[State]: ", "Finished");
		// Double return : pattern, point x, point y, point z
		return new double[]{pattern, point_x, point_y, point_z};
	}
	public Rect CustomCrop(int originWidth, int originHeight, int offsetTop, int offsetBottom, int offsetLeft, int offsetRight)
		/* Calculate crop image, requiring both sides offset of x and y. */ {
		Log.d("CustomCrop[State]: ", "Starting");
		int width	= originWidth-offsetLeft-offsetRight;		// Calculate for width of image
		int height 	= originHeight-offsetTop-offsetBottom;		// Calculate for height of image
		Log.d("CustomCrop[State]: ", "Finished");
		return new Rect(offsetLeft, offsetTop, width, height);
	}
	public void QR_Discover()
		/* All events that take position at the point of A include QR Code Reading, AR Code Reading and Target position analyzing. */ {
		/* Set parameter related to image from navigation camera. */
		Rect crop 	= CustomCrop(1280, 960, 60, 450, 400, 430);
		Rect raw 	= CustomCrop(1280, 960,0,0,0,0);
		/* Set parameter about QR Code. */
		Mat	matSrc 	= new Mat(),		// Matrix for storing image form navigation camera.
			matSrc_raw;
		Bitmap bitmapSrc;				// Bitmap image according to the requirement of QR reading.
		String QR_Info = null; 			// Content from QR Code reader.
		/* Set general parameter */
		int loopCount = 0; 				// Variable of loop counter.
		final int loopCountDetect = 4;	// Variable of max loop count.
		final int loopCountMax = 12;		// Variable of max loop count.

		while(!QRCodeFinish && loopCount < loopCountMax)
		{ /* The condition is defined with success ar code reading and the number of loops (the AR code must be read before the AR code can be read). */
			Log.d("Move&GetImage[State]:", " Starting");
			long timeStart = SystemClock.elapsedRealtime();

			// First step : Movement to point A positionn -> Get image form Nav Camera -> Get realtime position of robot.
			moveTo(Point_A, Quaternion_Target, 3);
			matSrc_raw = api.getMatNavCam();	//	Get image form navigation camera.

			if(loopCount >= loopCountDetect)	// 	Cope with situations where the robot loses self-localization.
			{
				matSrc = new Mat(matSrc_raw, raw);
				bitmapSrc = Bitmap.createBitmap(raw.width, raw.height, Bitmap.Config.ARGB_8888);
			}
			else
			{
				Imgproc.warpPerspective(new Mat(matSrc_raw, crop), matSrc, getH(), new Size(crop.width, crop.height));
				bitmapSrc = Bitmap.createBitmap(matSrc.width(), matSrc.height(), Bitmap.Config.ARGB_8888);
			}
			Log.d("Move&GetImage[State]:", " Finished");
			Log.d("Move&GetImage[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);

			// Second step (QR code event) : Get image form Nav Camera -> QR Code decoder -> Send data of QR code to judge.
			try
			{
				Log.d("QR[State]:", " Starting");
				timeStart = SystemClock.elapsedRealtime();

				matToBitmap(matSrc, bitmapSrc, false); 					// Convert mat image to bitmap image.
				int[] pixel = new int[bitmapSrc.getWidth() * bitmapSrc.getHeight()];  // Calculate image dimension.
				bitmapSrc.getPixels(pixel, 0, bitmapSrc.getWidth(), 0, 0, bitmapSrc.getWidth(), bitmapSrc.getHeight());
				Image QR_code = new Image(bitmapSrc.getWidth(), bitmapSrc.getHeight(), "RGB4");
				QR_code.setData(pixel);

				ImageScanner reader = new ImageScanner();	// Set parameter about reader.
				reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
				reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
				reader.scanImage(QR_code.convert("Y800"));

				SymbolSet Symbols = reader.getResults(); // Get result form reader.
				for (Symbol Sym : Symbols)
				{
					QR_Info = Sym.getData();
					Log.e("QR[State]:", " Detected");
					Log.d("QR[Data][Raw]:", " " + QR_Info);
				}
			}
			catch (Exception error)
			{
				Log.e("QR[State]:", " Not detected");
				Log.e("QR[Error]: ", error.getMessage());
			}
			finally
			{
				if(QR_Info != null)
				{ /* The condition is QR Code reading successful. */
					api.sendDiscoveredQR(QR_Info); // Send data of QR code to judge.
					double[] Buf = StringParseToDouble(QR_Info);
					Pattern = (int) Buf[0];
					Point_A_Prime = new Point(Buf[1], Buf[2], Buf[3]);
					Log.d("QR[Data][Pattern]:", " " + Pattern);
					Log.d("QR[Data][A_Prime]:", " X: " + Point_A_Prime.getX() + ", Y: " + Point_A_Prime.getY() + ", Z: " + Point_A_Prime.getZ());
					QRCodeFinish = true; // Set new condition.
				}
				Log.d("QR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
			}
			Log.d("A_Event[State]:", " Finished");
			Log.d("A_Event[Count]:", " " + loopCount);
			loopCount++;
		}
	}
	public void getQuaTargetPoint()
		/* Similar to the moveTo six variables but between is vector ends are pointed by a laser pointer not center of robot. */ {
		double [] robot_pos		= {Point_A.getX(), Point_A.getY(), Point_A.getZ()};
		double [] target_pos	= {Point_Target.getX(), Point_Target.getY(), Point_Target.getZ()};
		double [] target_vec	= getVector(target_pos, robot_pos);
		double target_val		= getVal(target_pos);
		double [] laser_shift	= {0.1302, 0.0572, -0.111} ;
		double laser_val		= getVal (laser_shift);
		double las_angle		= 136.11699 * 0.01745;
		double [] length		= getRoot(1,-2*laser_val*Math.cos(las_angle),(laser_val*laser_val)-(target_val*target_val));
		double laser_length;
		if (length[0] > 0 )		laser_length = length[0];
		else 					laser_length = length[1];

		double [] origin_vec	= {laser_shift[0]+laser_length, laser_shift[1], laser_shift[2]};
		double [] Qua			= getQua(origin_vec, target_vec);
		Quaternion_Target = new Quaternion((float) Qua[1], (float) Qua[2], (float) Qua[3], (float) Qua[0]);
		Log.d("FinalQuaternion= ", "w: " + Qua[0] + ", a: " + Qua[1] + ", b: " + Qua[2] +", c: " + Qua[3]);
	}
	public void getTargetPosition()
		/* The winner's secret is here. */ {
			 if(Pattern == 0)	Point_Target = new Point(11.08059,-10.585,5.41544);
		else if(Pattern == 1)	Point_Target = new Point(11.11328,-10.585,5.44136);
		else if(Pattern == 2)	Point_Target = new Point(11.07238,-10.585,5.39629);
		else if(Pattern == 3)	Point_Target = new Point(11.04639,-10.585,5.42993);
		else if(Pattern == 4)	Point_Target = new Point(11.11752,-10.585,5.42484);
		else if(Pattern == 5)	Point_Target = new Point(11.05738,-10.585,5.38108);
		else if(Pattern == 6)	Point_Target = new Point(11.11449,-10.585,5.44592);
		else if(Pattern == 7)	Point_Target = new Point(11.05118,-10.585,5.41778);
		else if(Pattern == 8)	Point_Target = new Point(11.07113,-10.585,5.38637);
		getQuaTargetPoint();
	}
	public Mat getH()
		/* The constant of homography from the lab experiment. */ {
		final double[] H_Array =
		{
			0.900417921032328,		-0.3502817876735206,	40.18939928174051,
			0.5441117400558353,		0.8458889627475615,		-279.6971999106549,
			-0.001735209403836954,	-0.0001491801126560375,	1
		};
		Mat H = new Mat(3, 3, CvType.CV_32F);
		H.put(0, 0, H_Array);
		return H;
	}
}
