package com.example.esp32controller;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import androidx.appcompat.app.AppCompatActivity;

public class ControllerActivity extends AppCompatActivity {

    private UdpManager udpManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_controller);

        udpManager = new UdpManager(this);

        Button buttonForward = findViewById(R.id.buttonForward);
        Button buttonBackward = findViewById(R.id.buttonBackward);
        Button buttonStop = findViewById(R.id.buttonStop);

        buttonForward.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                udpManager.sendCommand("FORWARD");
            }
        });

        buttonBackward.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                udpManager.sendCommand("BACKWARD");
            }
        });

        buttonStop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                udpManager.sendCommand("STOP");
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        udpManager.close();
    }
}