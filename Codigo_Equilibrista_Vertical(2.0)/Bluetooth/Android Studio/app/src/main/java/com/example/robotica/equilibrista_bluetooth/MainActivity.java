package com.example.robotica.equilibrista_bluetooth;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

public class MainActivity extends AppCompatActivity {

    BluetoothAdapter bluetoothAdapter;
    private static final int SOLICITA_ATIVACAO_BLUETOOTH = 1;
    Button buttonConexao;
    ImageView buttonCima, buttonBaixo, buttonEsquerda, buttonDireita;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        buttonConexao = (Button) findViewById(R.id.buttonConexao);
        buttonCima = (ImageView) findViewById(R.id.imageViewCima);
        buttonEsquerda = (ImageView) findViewById(R.id.imageViewEsquerda);
        buttonDireita = (ImageView) findViewById(R.id.imageViewDireita);
        buttonBaixo = (ImageView) findViewById(R.id.imageViewBaixo);

        if (bluetoothAdapter == null) {
            Toast.makeText(getApplicationContext(), "Seu dispositivo não possui bluetooth", Toast.LENGTH_LONG);
        }
        else if (!bluetoothAdapter.isEnabled()) {
            Intent ativaBluetooth = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(ativaBluetooth, SOLICITA_ATIVACAO_BLUETOOTH);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        //super.onActivityResult(requestCode, resultCode, data);
        switch (requestCode) {
            case SOLICITA_ATIVACAO_BLUETOOTH:
                if (resultCode == Activity.RESULT_OK) {
                    Toast.makeText(getApplicationContext(), "O bluetooth foi ativado", Toast.LENGTH_LONG);
                }
                else {
                    Toast.makeText(getApplicationContext(), "O bluetooth não foi ativado, o app será encerrado", Toast.LENGTH_LONG);
                    finish();
                }
                break;


        }
    }
}
