package com.example.esp32controller;

import android.content.Intent;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize the UDP manager
        UdpManager udpManager = new UdpManager(this);
        udpManager.start();

        // Set up button listeners or other UI elements here
        // For example, navigate to the controller activity
        findViewById(R.id.button_start_controller).setOnClickListener(v -> {
            Intent intent = new Intent(MainActivity.this, ControllerActivity.class);
            startActivity(intent);
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Stop the UDP manager when the activity is destroyed
        UdpManager.stop();
    }
}