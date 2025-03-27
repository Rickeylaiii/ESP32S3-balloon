package com.example.esp32controller;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.AsyncTask;
import android.util.Log;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UdpManager {
    private static final String TAG = "UdpManager";
    private static final int UDP_PORT = 8888; // Port for UDP communication
    private DatagramSocket socket;
    private Context context;

    public UdpManager(Context context) {
        this.context = context;
        try {
            socket = new DatagramSocket();
        } catch (Exception e) {
            Log.e(TAG, "Error initializing UDP socket", e);
        }
    }

    public void sendData(String message, String ipAddress) {
        new SendDataTask().execute(message, ipAddress);
    }

    public void receiveData() {
        new ReceiveDataTask().execute();
    }

    private class SendDataTask extends AsyncTask<String, Void, Void> {
        @Override
        protected Void doInBackground(String... params) {
            String message = params[0];
            String ipAddress = params[1];
            try {
                byte[] buffer = message.getBytes();
                InetAddress address = InetAddress.getByName(ipAddress);
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, UDP_PORT);
                socket.send(packet);
                Log.d(TAG, "Sent: " + message + " to " + ipAddress);
            } catch (Exception e) {
                Log.e(TAG, "Error sending UDP packet", e);
            }
            return null;
        }
    }

    private class ReceiveDataTask extends AsyncTask<Void, String, Void> {
        @Override
        protected Void doInBackground(Void... voids) {
            try {
                byte[] buffer = new byte[1024];
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                socket.receive(packet);
                String receivedMessage = new String(packet.getData(), 0, packet.getLength());
                publishProgress(receivedMessage);
            } catch (Exception e) {
                Log.e(TAG, "Error receiving UDP packet", e);
            }
            return null;
        }

        @Override
        protected void onProgressUpdate(String... values) {
            super.onProgressUpdate(values);
            // Handle the received message (update UI or notify user)
            Log.d(TAG, "Received: " + values[0]);
        }
    }

    public void close() {
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }

    public boolean isNetworkAvailable() {
        ConnectivityManager connectivityManager = (ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkInfo activeNetworkInfo = connectivityManager.getActiveNetworkInfo();
        return activeNetworkInfo != null && activeNetworkInfo.isConnected();
    }
}