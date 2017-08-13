package net.akirayou.rostest;

import android.Manifest;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.media.MediaScannerConnection;
import android.os.Bundle;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoAreaDescriptionMetaData;
import com.obsez.android.lib.filechooser.ChooserDialog;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class SelectActivity extends AppCompatActivity {
    private static final String TAG = SelectActivity.class.getSimpleName();
    private ArrayList<String> uuids;
    private ArrayList<String> names = new ArrayList<String>();
    private Tango mTango = null;
    private ArrayAdapter<String> sp_uuid_ad;


    private boolean readyADF=false;
    private void onReadyToADF(){
        readyADF=true;
        runOnTango(new Runnable() {
            @Override
            public void run() {
                loadUuidList();
            }
        });
    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_select);

        startActivityForResult(Tango.getRequestPermissionIntent(
                Tango.PERMISSIONTYPE_ADF_LOAD_SAVE), Tango.TANGO_INTENT_ACTIVITYCODE);

        findViewById(R.id.bt_publish).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(), PublishActivity.class);
                int i = ((Spinner) findViewById(R.id.sp_uuid)).getSelectedItemPosition();
                String uuid = uuids.get(i);
                intent.putExtra("targetUuid", uuid);
                startActivity(intent);
            }
        });
        findViewById(R.id.bt_learn).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(), LearnActivity.class);
                intent.putExtra("targetName", ((EditText) findViewById(R.id.et_newName)).getText().toString());
                startActivity(intent);
            }
        });
        findViewById(R.id.bt_export).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                exportADF();
            }
        });
        findViewById(R.id.bt_import).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                importADF();
            }
        });
        findViewById(R.id.bt_del).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                deleteADF();
            }
        });

        sp_uuid_ad = new ArrayAdapter<String>(SelectActivity.this, android.R.layout.simple_spinner_item);
        ((Spinner) findViewById(R.id.sp_uuid)).setAdapter(sp_uuid_ad);
        if(readyADF)runOnTango(new Runnable() {
            @Override
            public void run() {
                loadUuidList();
            }
        });
    }

    private File basePath(){


        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);

    }
    interface UuidRunnable extends Runnable{
        void setUuid(String uuid);

    }
    private boolean runOnSelectedUuid(UuidRunnable r){
        int i = ((Spinner) findViewById(R.id.sp_uuid)).getSelectedItemPosition();
        if ("".equals(uuids.get(i))) {
            showsToastAndFinishOnUiThread("No uuid is selected");
            return false;
        } else {
            r.setUuid(uuids.get(i));
            r.run();
            return true;
        }
    }
    private  void deleteADF(){
        runOnSelectedUuid(new UuidRunnable() {
            private String uuid;
            @Override
            public void setUuid(String _uuid) {
                uuid=_uuid;
            }
            @Override
            public void run() {
                MediaScannerConnection.scanFile(getApplicationContext(), new String[]{basePath().getAbsolutePath() + "/" + uuid}, null, null);
                runOnTango(new Runnable() {
                    @Override
                    public void run() {
                        Log.i(TAG,"runOnTango in delete");
                        mTango.deleteAreaDescription(uuid);
                        showsToastAndFinishOnUiThread("DELETE:" + uuid);
                        loadUuidList();
                    }
                });

            }
        });
    }
    protected void exportADF() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) !=
                PackageManager.PERMISSION_GRANTED) {
            // 以前に許諾して、今後表示しないとしていた場合は、ここにはこない
            if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                    Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
                // ユーザに許諾してもらうために、なんで必要なのかを説明する
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                builder.setMessage("Permit Exporting?");
                builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        String[] permissions = new String[] {
                                Manifest.permission.WRITE_EXTERNAL_STORAGE
                        };
                        ActivityCompat.requestPermissions(SelectActivity.this, permissions, /*REQ CODE*/ 2);
                    }});
                builder.setNegativeButton("CANCEL", null);
                builder.show();
            } else {
                // startActivityForResult()みたいな感じで許諾を要求
                String[] permissions = new String[] {
                        Manifest.permission.WRITE_EXTERNAL_STORAGE
                };
                ActivityCompat.requestPermissions(SelectActivity.this, permissions, /*REQ CODE*/ 2);
            }
        } else {
            runOnSelectedUuid(new UuidRunnable() {
                private String uuid;
                @Override
                public void setUuid(String _uuid) {
                    uuid=_uuid;
                }
                @Override
                public void run() {
                    MediaScannerConnection.scanFile(getApplicationContext(), new String[]{basePath().getAbsolutePath() + "/" + uuid}, null, null);
                    runOnTango(new Runnable() {
                        @Override
                        public void run() {
                            mTango.exportAreaDescriptionFile(uuid, basePath().toString());
                            //showsToastAndFinishOnUiThread("EXPORT:" + basePath().getAbsolutePath());
                        }
                    });

                }
            });
        }
    }
    protected void importADF(){
        new ChooserDialog().with(SelectActivity.this)
                .withStartFile(basePath().toString())
                .withChosenListener(new ChooserDialog.Result() {
                    @Override
                    public void onChoosePath(String path, File pathFile) {
                        //showsToastAndFinishOnUiThread( "FILE: " + path);
                        final String r_path=path;
                        runOnTango(new Runnable() {
                            @Override
                            public void run() {
                                mTango.importAreaDescriptionFile(r_path);
                            }
                        });

                    }
                })
                .build()
                .show();

    }
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        //reload uuid list after import(and export[no need])
        if(requestCode==Tango.TANGO_INTENT_ACTIVITYCODE  && resultCode == RESULT_OK){//Tango import Request Intent
            onReadyToADF();
        }

    }
    private void runOnTango(final Runnable run){
        mTango = new Tango(this, new Runnable() {
            @Override
            public void run() {
                synchronized (SelectActivity.this) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            run.run();

                            mTango.disconnect();
                        }
                    });
                }
            }
        });

    }
    private void loadUuidList(){
        names.clear();
        uuids = mTango.listAreaDescriptions();
        for (int i = 0; i < uuids.size(); i++) {
            TangoAreaDescriptionMetaData metadata = new TangoAreaDescriptionMetaData();
            metadata = mTango.loadAreaDescriptionMetaData(uuids.get(i));
            byte[] nameBytes = metadata.get(TangoAreaDescriptionMetaData.KEY_NAME);
            byte[] epoch = metadata.get(TangoAreaDescriptionMetaData.KEY_DATE_MS_SINCE_EPOCH);
            ByteBuffer buf = ByteBuffer.wrap(epoch);
            buf.order(ByteOrder.LITTLE_ENDIAN);
            long epochVal = buf.getLong();
            String name;
            if (nameBytes != null) {
                name = new String(nameBytes);
            }else {
                name = "NO-NAME";
            }
            SimpleDateFormat fmt = new SimpleDateFormat("yyyyMMdd_HH:mm:ss");
            //fmt.setTimeZone(TimeZone.getTimeZone("UTC"));
            names.add(name+"/"+fmt.format(new Date(epochVal))+"/"+uuids.get(i));
        }

        uuids.add(0,"");
        names.add(0,"No use");
        Log.e(TAG,names.toString());
        ((Spinner) findViewById(R.id.sp_uuid)).setSelection(0,true);
        ArrayAdapter<String> ad=(ArrayAdapter<String>) ((Spinner) findViewById(R.id.sp_uuid)).getAdapter();
        ad.clear();
        ad.addAll(names);
        ad.notifyDataSetChanged();
        ((Spinner) findViewById(R.id.sp_uuid)).invalidate();
    }






    private void showsToastAndFinishOnUiThread(final String res) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(SelectActivity.this,res, Toast.LENGTH_LONG).show();
                // finish();
            }
        });
    }


}



///////TODO: write uuid select code to selectActivity  / modify PublishActivity to use ADF