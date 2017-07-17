package net.akirayou.rostest;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoAreaDescriptionMetaData;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

public class SelectActivity extends AppCompatActivity {
    private static final String TAG = SelectActivity.class.getSimpleName();
    private ArrayList<String> uuids;
    private ArrayList<String> names=new ArrayList<String>();
    private Tango mTango=null;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_select);
        findViewById(R.id.bt_publish).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(), PublishActivity.class);
                int i=((Spinner)findViewById(R.id.sp_uuid)).getSelectedItemPosition();
                String uuid=uuids.get(i);
                intent.putExtra("targetUuid",uuid);
                startActivity(intent);
            }
        });
        findViewById(R.id.bt_learn).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(), LearnActivity.class);
                intent.putExtra("targetName",((EditText)findViewById(R.id.et_newName)).getText().toString());
                startActivity(intent);
            }
        });
        Log.i(TAG,"==============READING ADF UUID===========");
        startActivityForResult(Tango.getRequestPermissionIntent(
                Tango.PERMISSIONTYPE_ADF_LOAD_SAVE), Tango.TANGO_INTENT_ACTIVITYCODE);
    }
    @Override
    protected void onStart(){
        super.onStart();
        names.clear();
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


    }


    private void showsToastAndFinishOnUiThread(final String res) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(SelectActivity.this,res, Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }


}



///////TODO: write uuid select code to selectActivity  / modify PublishActivity to use ADF