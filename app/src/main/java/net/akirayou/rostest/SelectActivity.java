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
import java.util.ArrayList;

public class SelectActivity extends AppCompatActivity {
    private static final String TAG = SelectActivity.class.getSimpleName();
    private ArrayList<String> uuids;
    private ArrayList<String> names = new ArrayList<String>();
    private Tango mTango = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_select);
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
        Log.i(TAG, "==============READING ADF UUID===========");

        startActivityForResult(Tango.getRequestPermissionIntent(
                Tango.PERMISSIONTYPE_ADF_LOAD_SAVE), Tango.TANGO_INTENT_ACTIVITYCODE);
    }

    private File basePath(){


        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);

    }
    protected void exportADF() {

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) !=
                PackageManager.PERMISSION_GRANTED) {
            // 以前に許諾して、今後表示しないとしていた場合は、ここにはこない
            if (ActivityCompat.shouldShowRequestPermissionRationale(this,
                    Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
                // ユーザに許諾してもらうために、なんで必要なのかを説明する
                AlertDialog.Builder builder = new AlertDialog.Builder(this);
                builder.setMessage("なんで、そのパーミッションが必要なのかを説明");
                builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        String[] permissions = new String[] {
                                Manifest.permission.WRITE_EXTERNAL_STORAGE
                        };
                        ActivityCompat.requestPermissions(SelectActivity.this, permissions, /*REQ CODE*/ 2);
                    }});
                builder.setNegativeButton("だめ", null);
                builder.show();
            } else {
                // startActivityForResult()みたいな感じで許諾を要求
                String[] permissions = new String[] {
                        Manifest.permission.WRITE_EXTERNAL_STORAGE
                };
                ActivityCompat.requestPermissions(SelectActivity.this, permissions, /*REQ CODE*/ 2);
            }
        } else {
            int i = ((Spinner) findViewById(R.id.sp_uuid)).getSelectedItemPosition();
            final String uuid = uuids.get(i);
            final String file = basePath().toString() ;

            MediaScannerConnection.scanFile(getApplicationContext(), new String[]{basePath().getAbsolutePath()+"/"+uuid}, null, null);
            runInTango(new Runnable() {
                @Override
                public void run() {
                    mTango.exportAreaDescriptionFile(uuid, file);
                    //showsToastAndFinishOnUiThread("EXPORT:" + basePath().getAbsolutePath());
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
                        showsToastAndFinishOnUiThread( "FILE: " + path);
                    }
                })
                .build()
                .show();

    }
    private void runInTango(final Runnable run){
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


    @Override
    protected void onStart(){
        super.onStart();
        names.clear();
        runInTango(new Runnable() {
            @Override
            public void run() {
                uuids = mTango.listAreaDescriptions();
                for (int i = 0; i < uuids.size(); i++) {
                    TangoAreaDescriptionMetaData metadata = new TangoAreaDescriptionMetaData();
                    metadata = mTango.loadAreaDescriptionMetaData(uuids.get(i));
                    byte[] nameBytes = metadata.get(TangoAreaDescriptionMetaData.KEY_NAME);
                    byte[] epoch = metadata.get(TangoAreaDescriptionMetaData.KEY_DATE_MS_SINCE_EPOCH);
                    ByteBuffer buf = ByteBuffer.wrap(epoch);
                    buf.order(ByteOrder.LITTLE_ENDIAN);
                    long epochVal = buf.getLong();
                    if (nameBytes != null) {
                        names.add(new String(nameBytes)+"-"+String.valueOf(epochVal));
                    } else {
                        names.add(uuids.get(i)+"-"+Long.toUnsignedString(epochVal));
                    }

                }

                uuids.add("");
                names.add("No use");

                ArrayAdapter<String> ad = new ArrayAdapter<String>(SelectActivity.this, android.R.layout.simple_spinner_item, names);
                ((Spinner) findViewById(R.id.sp_uuid)).setAdapter(ad);

            }
        });

/*
        mTango = new Tango(this, new Runnable() {
            @Override
            public void run() {
                synchronized (SelectActivity.this) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            uuids = mTango.listAreaDescriptions();
                            for (int i = 0; i < uuids.size(); i++) {
                                TangoAreaDescriptionMetaData metadata = new TangoAreaDescriptionMetaData();
                                metadata = mTango.loadAreaDescriptionMetaData(uuids.get(i));
                                byte[] nameBytes = metadata.get(TangoAreaDescriptionMetaData.KEY_NAME);
                                byte[] epoch = metadata.get(TangoAreaDescriptionMetaData.KEY_DATE_MS_SINCE_EPOCH);
                                ByteBuffer buf = ByteBuffer.wrap(epoch);
                                buf.order(ByteOrder.LITTLE_ENDIAN);
                                long epochVal = buf.getLong();
                                if (nameBytes != null) {
                                    names.add(new String(nameBytes)+"-"+String.valueOf(epochVal));
                                } else {
                                    names.add(uuids.get(i)+"-"+Long.toUnsignedString(epochVal));
                                }

                            }

                            uuids.add("");
                            names.add("No use");

                            ArrayAdapter<String> ad = new ArrayAdapter<String>(SelectActivity.this, android.R.layout.simple_spinner_item, names);
                            ((Spinner) findViewById(R.id.sp_uuid)).setAdapter(ad);
                            mTango.disconnect();
                        }

                    });
                }
            }
        });
*/

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