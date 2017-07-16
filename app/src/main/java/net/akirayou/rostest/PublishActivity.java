package net.akirayou.rostest;

import android.Manifest;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
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

import org.ros.android.RosAppCompatActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.tf2_ros.StaticTransformBroadcaster;
import org.ros.tf2_ros.TransformBroadcaster;

import java.util.ArrayList;

import geometry_msgs.TransformStamped;


public class PublishActivity extends RosAppCompatActivity {
    //TF2 broad caster
    private  TransformBroadcaster mTB=null;
    private TransformStamped tfs=null ;
    private TransformStamped stfs=null;
    private org.ros.tf2_ros.StaticTransformBroadcaster mSTB=null;
    //example of publisher
    private TestTalker talker;
    private String targetUuid="";
    //for TTango
    private boolean tangoEnabled=false;
    private TangoCameraPreview preview;
    //For Logging(Debug)
    private static final String TAG = PublishActivity.class.getSimpleName();


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
                            ActivityCompat.requestPermissions(PublishActivity.this,
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
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        //config.putInt(TangoConfig.KEY_INT_DEPTH_MODE,TangoConfig.TANGO_DEPTH_MODE_XYZ_IJ);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);

        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA,true);
        config.putInt(TangoConfig.KEY_INT_RUNTIME_DEPTH_FRAMERATE,4);

        // Tango Service should automatically attempt to recover when it enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);

        if(!targetUuid.equals("")) {
            Log.i(TAG,"===========USING uuid="+targetUuid);
            config.putString(TangoConfig.KEY_STRING_AREADESCRIPTION, targetUuid);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, false);

        }


        return config;
    }
    private void startTango(){
        if(tangoEnabled){
            Log.e(TAG,"ignore double resume for tango");
            return; //Ha?
        }
        preview=new TangoCameraPreview(this);
        LinearLayout ll = (LinearLayout)findViewById(R.id.surfaceView);
        ll.addView(preview);
        if (PermissionChecker.checkSelfPermission(
                PublishActivity.this, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            requestCameraPermission();
        }

        /*
        startActivityForResult(Tango.getRequestPermissionIntent(
                Tango.PERMISSIONTYPE_MOTION_TRACKING), Tango.TANGO_INTENT_ACTIVITYCODE);*/
        startActivityForResult(Tango.getRequestPermissionIntent(
                Tango.PERMISSIONTYPE_ADF_LOAD_SAVE), Tango.TANGO_INTENT_ACTIVITYCODE);

        Log.i(TAG,"tangoStart on resume");
        mTango = new Tango(PublishActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                synchronized (PublishActivity.this) {
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
                        showsToastAndFinishOnUiThread("TANGOOOOOO invalid");
                    }
                }
            }
        });


    }
    private void stopTango(){
        synchronized (PublishActivity.this) {
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
        if(targetUuid.equals("")) {
            framePairs.add(new TangoCoordinateFramePair(
                    TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                    TangoPoseData.COORDINATE_FRAME_DEVICE));
        }else {
            framePairs.add(new TangoCoordinateFramePair(
                    TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                    TangoPoseData.COORDINATE_FRAME_DEVICE));
        }
        // Listen for new Tango data.
        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                final  double poseSpan=100e-3;
                if (pose.statusCode == pose.POSE_VALID && rosNodeEnable && pose.timestamp-lastPoseTime>poseSpan) {
                    final float pos[] = pose.getTranslationAsFloats();
                    final float rot[] = pose.getRotationAsFloats();
                    //X=y Y=-z Z=x
                    tfs.getTransform().getTranslation().setX(pos[0]);
                    tfs.getTransform().getTranslation().setY(pos[1]);
                    tfs.getTransform().getTranslation().setZ(pos[2]);
                    tfs.getTransform().getRotation().setX(rot[0]);
                    tfs.getTransform().getRotation().setY(rot[1]);
                    tfs.getTransform().getRotation().setZ(rot[2]);
                    tfs.getTransform().getRotation().setW(rot[3]);


                    tfs.getHeader().setFrameId("tango_base");
                    tfs.setChildFrameId("tango_device");
                    tfs.getHeader().setStamp(new Time(pose.timestamp));
                    mTB.sendTransform(tfs);
                    lastPoseTime=pose.timestamp;
                    //Log.w(TAG,"pose");
                }
                runOnUiThread(new Runnable() {
                    public void run() {
                        ((TextView) findViewById(R.id.txPose)).setText(pose.toString());
                    }});
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                // We are not using onPointCloudAvailable for this app.
                Log.w(TAG,"pointcloud");

                int numPoints=pointCloud.numPoints;

                int nofElement=pointCloud.points.capacity();
                Log.i(TAG,String.valueOf(nofElement)+" "+ String.valueOf(numPoints));


                pubPC.publish(pointCloud);
                //super.onPointCloudAvailable(pointCloud);
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                // Ignoring TangoEvents.
                //Log.i(TAG,event.toString());

            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // We are not using onFrameAvailable for this application.
                if(cameraId== TangoCameraIntrinsics.TANGO_CAMERA_COLOR){
                    preview.onFrameAvailable(); //Need to call every time. Do not skip.

                }
            }
        });
    }

    public PublishActivity() {
        super("rostest2", "rostest2");

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

        targetUuid = getIntent().getExtras().getString("targetUuid");
    }
    private boolean rosEnable=false;
    private boolean rosNodeEnable=false;
    private PubPointCloud pubPC;
    //For ROS
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) { //for ROS

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());

        talker=new TestTalker();
        nodeMainExecutor.execute(talker, nodeConfiguration);

        pubPC=new PubPointCloud();
        nodeMainExecutor.execute(pubPC, nodeConfiguration);

        mTB = new TransformBroadcaster();




        nodeMainExecutor.execute(mTB, nodeConfiguration);
        tfs = mTB.newMessage();


        mSTB = new StaticTransformBroadcaster();
        nodeMainExecutor.execute(mSTB, nodeConfiguration);

        stfs= mSTB.newMessage();
        stfs.getTransform().getTranslation().setX(0);
        stfs.getTransform().getTranslation().setY(0);
        stfs.getTransform().getTranslation().setZ(0);
        stfs.getTransform().getRotation().setX(0.70710678);
        stfs.getTransform().getRotation().setY(-0.70710678);
        stfs.getTransform().getRotation().setZ(0);
        stfs.getTransform().getRotation().setW(0);


        stfs.getHeader().setFrameId("tango_device");
        stfs.setChildFrameId("tango_depth_device");

        rosEnable=true;


        new Handler(Looper.getMainLooper()).postDelayed(new Runnable() {
            @Override
            public void run() {
                mSTB.sendTransform(stfs);
                rosNodeEnable = true;
            }
        }, 1000);
    }
    @Override
    protected void onStart(){
        super.onStart();
        if(rosEnable) startTango();

    }

    @Override
    protected void onStop() {
        if(rosEnable)stopTango();
        super.onStop();
    }


    private void showsToastAndFinishOnUiThread(final String res) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(PublishActivity.this,res, Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }
}
