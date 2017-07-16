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

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

public class SelectActivity extends AppCompatActivity {
    private static final String TAG = SelectActivity.class.getSimpleName();
    private ArrayList<String> uuids=new ArrayList<String>();
    private ArrayList<String> names=new ArrayList<String>();

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
        uuids.clear();
        names.clear();
        uuids.add("");
        names.add("No use");
        Log.i(TAG,"==============READING ADF UUID===========");
        try {
            //BufferedReader in = Files.newBufferedReader("uuid.txt", StandardCharsets.UTF_8);
            BufferedReader in = new BufferedReader(new InputStreamReader( openFileInput("uuid.txt")));

            while(true){
                String s;

                s=in.readLine();if(s==null)break;
                String uuid=s;
                uuids.add(uuid);
                Log.i(TAG,"READ UUID"+uuid);
                s=in.readLine();if(s==null)break;
                String name=s;
                names.add(name+uuid);
            }
            in.close();
        } catch (FileNotFoundException e) {
            //showsToastAndFinishOnUiThread("No learned ADF yet");
            Log.e(TAG,"Can not open uuid");
        }catch (IOException e) {
            //showsToastAndFinishOnUiThread("ADF readError");
            Log.e(TAG,"IO error in reading uuid");
        }
        ArrayAdapter<String> ad= new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item,names);
        ((Spinner)findViewById(R.id.sp_uuid)).setAdapter(ad);
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