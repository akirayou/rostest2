package net.akirayou.rostest;

import android.Manifest;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.PermissionChecker;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoAreaDescriptionMetaData;
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
import com.projecttango.tangosupport.TangoPointCloudManager;

import java.util.ArrayList;

public class LearnActivity extends AppCompatActivity {
    private String targetName="";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_learn);
        targetName = getIntent().getExtras().getString("targetName");
        findViewById(R.id.bt_learn).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String uuid=mTango.saveAreaDescription();
                TangoAreaDescriptionMetaData metadata = new TangoAreaDescriptionMetaData();
                metadata = mTango.loadAreaDescriptionMetaData(uuid);
                metadata.set(TangoAreaDescriptionMetaData.KEY_NAME, targetName.getBytes());
                mTango.saveAreaDescriptionMetadata(uuid, metadata);
                finish();
            }
        });
        //for tango Initialize
        preview=new TangoCameraPreview(this);
        LinearLayout ll = (LinearLayout)findViewById(R.id.surfaceView);
        ll.addView(preview);
        if (PermissionChecker.checkSelfPermission(
                LearnActivity.this, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            requestCameraPermission();
        }
        startActivityForResult(Tango.getRequestPermissionIntent(
                Tango.PERMISSIONTYPE_ADF_LOAD_SAVE), Tango.TANGO_INTENT_ACTIVITYCODE);

    }

    //for TTango
    private boolean tangoEnabled=false;
    private TangoCameraPreview preview;
    private TangoPointCloudManager mPointCloudManager;
    //For Logging(Debug)
    private static final String TAG = LearnActivity.class.getSimpleName();


    //Camera request for Tango preview
    private void requestCameraPermission() {
        final int REQUEST_CODE_CAMERA_PERMISSION = 0x01;
        if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                Manifest.permission.CAMERA)) {
            new AlertDialog.Builder(this)
                    .setTitle("パーミッションの追加説明")
                    .setMessage("このアプリで写真を撮るにはパーミッションが必要です")
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            ActivityCompat.requestPermissions(LearnActivity.this,
                                    new String[]{Manifest.permission.CAMERA},
                                    REQUEST_CODE_CAMERA_PERMISSION);
                        }
                    })
                    .create()
                    .show();
            return;
        }
        ActivityCompat.requestPermissions(this, new String[]{
                        Manifest.permission.CAMERA
                },
                REQUEST_CODE_CAMERA_PERMISSION);
        return;
    }
    //Tango Control
    private Tango mTango=null;
    private TangoConfig mConfig=null;
    private TangoConfig setupTangoConfig(Tango tango) {
        Log.i(TAG,"setupTangoConfig");
        // Create a new Tango Configuration and enable the HelloMotionTrackingActivity API.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA,true);

        // Tango Service should automatically attempt to recover when it enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);
        return config;
    }
    private void startTango(){
        Log.i(TAG,"===============START TANGO==============");
        if(tangoEnabled){
            Log.e(TAG,"ignore double resume for tango");
            return; //Ha?
        }

        Log.i(TAG,"tangoStart on resume");
        mTango = new Tango(LearnActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                synchronized (LearnActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);

                        preview.connectToTangoCamera(mTango,
                                TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
                        mTango.connect(mConfig);
                        attachTango();
                        //showsToastAndFinishOnUiThread("Tango Enabled");
                        tangoEnabled=true;
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, "out of data", e);
                        showsToastAndFinishOnUiThread("out of date");
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "tango error", e);
                        showsToastAndFinishOnUiThread("tango error");
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, "tango invalid", e);
                        showsToastAndFinishOnUiThread("tango invalid");
                    }
                }
            }
        });


    }
    private void stopTango(){
        Log.i(TAG,"===============STOP TANGO==============");
        synchronized (LearnActivity.this) {
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

    //Tango EventHandler
    private double lastPoseTime=0;
    private void attachTango() {
        Log.i(TAG,"attachTango");
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

                runOnUiThread(new Runnable() {
                    public void run() {
                        ((TextView) findViewById(R.id.txPose)).setText(pose.toString());
                    }});
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {}

            @Override
            public void onTangoEvent(final TangoEvent event) {
                runOnUiThread(new Runnable() {
                    public void run() {
                        ((TextView) findViewById(R.id.txEvent)).setText(event.toString());
                    }});
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                if(cameraId== TangoCameraIntrinsics.TANGO_CAMERA_COLOR){
                    preview.onFrameAvailable(); //Need to call every time. Do not skip.
                }
            }
        });
    }


    @Override
    protected void onStart(){
        super.onStart();
        startTango();
    }

    @Override
    protected void onStop() {
        stopTango();
        super.onStop();
    }


    private void showsToastAndFinishOnUiThread(final String res) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(LearnActivity.this,res, Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }


}
