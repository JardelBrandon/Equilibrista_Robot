package com.example.robotica.equilibrista_bluetooth;

import android.app.ListActivity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.Set;

public class ListaDispositivos extends ListActivity {
    private BluetoothAdapter bluetoothAdapter;
    static String ENDERECO_MAC;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        ArrayAdapter<String> arrayAdapterBluetooth = new ArrayAdapter<String>(getApplicationContext(), android.R.layout.simple_list_item_1);

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        Set<BluetoothDevice> dispositivosPareados = bluetoothAdapter.getBondedDevices();

        if (dispositivosPareados.size() > 0) {
            for (BluetoothDevice dispositivo : dispositivosPareados) {
                String nomeBluetooth = dispositivo.getName();
                String macBluetooth = dispositivo.getAddress();

                arrayAdapterBluetooth.add(nomeBluetooth + "\n" + macBluetooth);
            }
        }
        setListAdapter(arrayAdapterBluetooth);
    }

    @Override
    protected void onListItemClick(ListView l, View v, int position, long id) {
        super.onListItemClick(l, v, position, id);

        String informacaoGeral = ((TextView) v).getText().toString();
        //Toast.makeText(getApplicationContext(), "Info:" + informacaoGeral, Toast.LENGTH_LONG).show();

        String enderecoMac = informacaoGeral.substring(informacaoGeral.length() - 17);
        //Toast.makeText(getApplicationContext(), "Mac:" + enderecoMac, Toast.LENGTH_LONG).show();

        Intent retornaMac = new Intent();
        retornaMac.putExtra(ENDERECO_MAC, enderecoMac);
        setResult(RESULT_OK, retornaMac);
        finish();
    }
}
