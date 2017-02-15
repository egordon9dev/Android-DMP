package com.example.ethan.mydmp;
        import android.hardware.Sensor;
        import android.hardware.SensorEvent;
        import android.hardware.SensorEventListener;
        import android.hardware.SensorManager;
        import android.os.Bundle;
        import android.app.Activity;
        import android.content.Context;
        import android.widget.TextView;
        import java.text.DecimalFormat;

public class MainActivity extends Activity implements SensorEventListener {
    TextView textGX, textGY, textGZ, textAX, textAY, textAZ, textFiltX, textFiltY, textFiltZ;
    SensorManager sensorManager;
    Sensor sensor;
    Sensor sensAccel;
    long firstTime = 0;
    long time = 0, timeA = 0;
    long prevTime = 0, prevTimeA = 0;
    private double gx = 0.0, gy = 0.0, gz = 0.0, ax = 0.0, ay = 0.0, az = 0.0,
            gPitch = 0.0, gRoll = 0.0, gYaw = 0.0, apitch = 0.0, aroll = 0.0, ayaw = 0.0,
            filteredPitch = 0.0, filteredRoll = 0.0, filteredYaw = 0.0, gTotPitch = 0.0, gTotRoll = 0.0, gTotYaw = 0.0;
    private double forceMagnitude = 0.0;
    private final double kG = 9.8;

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        sensAccel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, sensAccel , SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, sensor , SensorManager.SENSOR_DELAY_NORMAL);

        textGX = (TextView) findViewById(R.id.textGX);
        textGY = (TextView) findViewById(R.id.textGY);
        textGZ = (TextView) findViewById(R.id.textGZ);
        textAX = (TextView) findViewById(R.id.textAX);
        textAY = (TextView) findViewById(R.id.textAY);
        textAZ = (TextView) findViewById(R.id.textAZ);
        textFiltX = (TextView) findViewById(R.id.textFilteredX);
        textFiltY = (TextView) findViewById(R.id.textFilteredY);
        textFiltZ = (TextView) findViewById(R.id.textFilteredZ);

        firstTime = System.currentTimeMillis();
    }

    public void onResume() {
        super.onResume();
        sensorManager.registerListener(this, sensAccel,
                SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, sensor,
                SensorManager.SENSOR_DELAY_NORMAL);
    }

    public void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    public void onStop() {
        super.onStop();
        sensorManager.unregisterListener(this);
    }
















    //=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

//---------------------------------------------------------------------------------------------------
// Definitions
    float sampleFreq = 50.0f;			// sample frequency in Hz
    float twoKpDef = (2.0f * 0.5f);	// 2 * proportional gain
    float twoKiDef = (2.0f * 0.0f);	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

    volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
    volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        float hx, hy, bx, bz;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = (float)Math.sqrt( (hx * hx) + (hy * hy) );
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute and apply integral feedback if enabled
            if(twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            }
            else {
                integralFBx = 0.0f;	// prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute and apply integral feedback if enabled
            if(twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            }
            else {
                integralFBx = 0.0f;	// prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

    float invSqrt(float x) {
        return 1.0f / (float)Math.sqrt(x);
        /*
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;*/
    }

//====================================================================================================
// END OF CODE
//====================================================================================================






























    long g_pt = 0;
    @Override
    public void onSensorChanged(SensorEvent event) {
        Sensor mySensor = event.sensor;

        time = System.currentTimeMillis();
        double deltaT = ((double)(time - prevTime))/(1000.0);
        prevTime = time;


        //time = System.currentTimeMillis();
        DecimalFormat df = new DecimalFormat("###.#");
        //increase tau -> increase response rate
        double tau = 0.003;
        double alpha=(tau)/(tau+deltaT);

        double test1 = 0.0, test2 = 0.0;

        if (mySensor.getType() == Sensor.TYPE_GYROSCOPE) {
            long g_ct = System.currentTimeMillis();
            double g_dt = ((double)(g_ct - g_pt))/(1000.0);
            g_pt = g_ct;
            if(time-firstTime < 1000) {
                gx = 0.0;
                gy = 0.0;
                gz = 0.0;
            } else {
                gx = event.values[0];
                gy = event.values[1];
                gz = event.values[2];

                gPitch = gx*g_dt * (180.0/Math.PI);
                gRoll = gy*g_dt * (180.0/Math.PI);
                gYaw = gz*g_dt * (180.0/Math.PI);

                gTotRoll += gRoll/Math.cos(gPitch*(Math.PI/180.0));
                gTotPitch += gPitch/Math.cos(gRoll*(Math.PI/180.0));
                gTotYaw += gYaw;

                gTotPitch += gTotRoll*Math.sin(gYaw*(Math.PI/180.0));
                gTotRoll -= gTotPitch*Math.sin(gYaw*(Math.PI/180.0));
/*
                gTotRoll -= gPitch*Math.sin(gYaw);
                gTotPitch += gRoll*Math.sin(gYaw);*/

                textGX.setText("gyro x :   " + df.format(gTotPitch) + " °");//x
                textGY.setText("gyro y :   " + df.format(gTotRoll) + " °");//y
                textGZ.setText("gyro z :   " + df.format(gTotYaw) + " °");//gyro yaw :   " + df.format(gyaw) + " °");//z
            }
        }
        else if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            timeA = System.currentTimeMillis();
           ax = event.values[0];
            ay = event.values[1];
            az = event.values[2];
            if(timeA-firstTime < 1000) {
                ax = 0.0;
                ay = 0.0;
                az = 0.0;
            }
            apitch = Math.atan2(ay, az) * (180/Math.PI);
            aroll = Math.atan2(ax, az) * (180/Math.PI);
            forceMagnitude = Math.abs(ax) + Math.abs(ay) + Math.abs(az);
            textAX.setText("a p :   " + df.format(apitch) + " °   " + df.format(filteredPitch) + " °");//x
            textAY.setText("a r :   " + df.format(aroll) + " °   "+ df.format(filteredRoll) + " °");//y
            textAZ.setText("");//x: " + df.format(ax) + "\ny: " + df.format(ay) + "\nz: " + df.format(az)/*"accel z :   " + df.format(az) + ""*/);//z
            prevTimeA = timeA;
        } else return;
        // if !bullshit:      sum sides > hypotenuse
        if (forceMagnitude > kG/2.0 && forceMagnitude < 2*kG)
        {
            //angle = (1-alpha)*(angle + gyro * dt) + (alpha)*(acc)
            filteredPitch = ((1-alpha) * (filteredPitch + gx*deltaT) + (alpha * apitch));
            filteredRoll = ((1-alpha) * (filteredRoll + gy*deltaT) + (alpha * aroll));
            filteredYaw = gYaw;
        }
        textFiltX.setText("f p :   " + df.format(filteredPitch) + " °");//x
        textFiltY.setText("f r :   " + df.format(filteredRoll) + " °");//y
        textFiltZ.setText("");//filtered yaw :   " + df.format(filteredYaw) + " °");
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}