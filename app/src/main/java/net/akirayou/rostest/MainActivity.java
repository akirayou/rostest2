package net.akirayou.rostest;

import android.Manifest;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.PermissionChecker;
import android.support.v7.app.AlertDialog;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoCameraPreview;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import org.ros.android.RosAppCompatActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.ArrayList;

public class MainActivity extends RosAppCompatActivity {




    private TestTalker talker;
    private boolean tangoEnabled=false;
    private TangoCameraPreview preview;
    private static final String TAG = MainActivity.class.getSimpleName();

    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("rostest2", "rostest2");


        //mTango.disconnect();
        //mTango=null;
    }
    private int REQUEST_CODE_CAMERA_PERMISSION = 0x01;
    private void requestCameraPermission() {
        if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                Manifest.permission.CAMERA)) {

            Log.d(TAG, "shouldShowRequestPermissionRationale:追加説明");
            // 権限チェックした結果、持っていない場合はダイアログを出す
            new AlertDialog.Builder(this)
                    .setTitle("パーミッションの追加説明")
                    .setMessage("このアプリで写真を撮るにはパーミッションが必要です")
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            ActivityCompat.requestPermissions(MainActivity.this,
                                    new String[]{Manifest.permission.CAMERA},
                                    REQUEST_CODE_CAMERA_PERMISSION);
                        }
                    })
                    .create()
                    .show();
            return;
        }
        // 権限を取得する
        ActivityCompat.requestPermissions(this, new String[]{
                        Manifest.permission.CAMERA
                },
                REQUEST_CODE_CAMERA_PERMISSION);
        return;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        findViewById(R.id.bt_kick).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                talker.kick();
            }
        });
        preview=new TangoCameraPreview(this);
        LinearLayout ll = (LinearLayout)findViewById(R.id.surfaceView);
        ll.addView(preview);


        if (PermissionChecker.checkSelfPermission(
                MainActivity.this, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            // パーミッションをリクエストする
            requestCameraPermission();
            return;
        }
    }
    private boolean rosInited=false;
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) { //for ROS

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        rosInited=true;

        talker=new TestTalker();
        nodeMainExecutor.execute(talker, nodeConfiguration);
    }


    private Tango mTango=null;
    private TangoConfig mConfig=null;



    @Override
    protected void onResume() {
        super.onResume();
        if(!rosInited)return;
        // Initialize Tango Service as a normal Android Service. Since we call mTango.disconnect()
        // in onPause, this will unbind Tango Service, so every time onResume gets called we
        // should create a new Tango object.
        if(tangoEnabled){
            Log.e(TAG,"ignore double resume for tango");
            return; //Ha?
        }

        Log.i(TAG,"tangoStart on resume");
        mTango = new Tango(MainActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                synchronized (MainActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        preview.connectToTangoCamera(mTango,
                                TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
                        mTango.connect(mConfig);
                        startupTango();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, "out of data", e);
                        showsToastAndFinishOnUiThread("out of date");
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "tango error", e);
                        showsToastAndFinishOnUiThread("tango error");
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, "tango invalid", e);
                        showsToastAndFinishOnUiThread("tando invalid");
                    }
                }
            }
        });
        tangoEnabled=true;
    }

    @Override
    protected void onPause() {
        super.onPause();
        if(!rosInited)return;
        synchronized (this) {
            try {
                if(tangoEnabled) {
                    mTango.disconnect();
                    tangoEnabled = false;
                }else{

                    Log.e(TAG,"tango double disable");
                }
            } catch (TangoErrorException e) {
                Log.e(TAG, "tango error", e);
            }
        }
    }

    /**
     * Sets up the tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango) {
        Log.i(TAG,"setupTangoConfig");
        // Create a new Tango Configuration and enable the HelloMotionTrackingActivity API.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA,true);

        // Tango Service should automatically attempt to recover when it enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);
        return config;
    }

    /**
     * Set up the callback listeners for the Tango Service and obtain other parameters required
     * after Tango connection.
     * Listen to new Pose data.
     */
    private void startupTango() {
        Log.i(TAG,"startupTango");
        // Lock configuration and connect to Tango.
        // Select coordinate frame pair.
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        // Listen for new Tango data.
        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                //Log.i(TAG,"pose");

                runOnUiThread(new Runnable() {
                    public void run() {

                        ((TextView) findViewById(R.id.txPose)).setText(pose.toString());
                    }});
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzIj) {
                // We are not using onXyzIjAvailable for this app.
                Log.i(TAG,"xyz");
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                // We are not using onPointCloudAvailable for this app.
                Log.i(TAG,"pointcloud");

            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                // Ignoring TangoEvents.
                Log.i(TAG,"tangoEvent");

            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // We are not using onFrameAvailable for this application.
                Log.i(TAG,"frame");
                if(cameraId== TangoCameraIntrinsics.TANGO_CAMERA_COLOR){
                    preview.onFrameAvailable();
                }

            }
        });
    }




    private void showsToastAndFinishOnUiThread(final String res) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(MainActivity.this,res, Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }
}
