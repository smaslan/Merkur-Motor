/*
 * Android control app for simple stepper motor controller. More details here:
 * https://github.com/smaslan/Merkur-Motor
 *
 * !!! Warning !!!
 *   This is my very first app attempt for Android and very first Kotlin project
 *   so it is far from being optimal/nice/safe/whatever!
 *
 * * (c) 2024-2025, Stanislav Maslan, s.maslan@seznam.cz
 * V1.0, 2025-01-01
 *
 * The code and all its parts are distributed under MIT license
 * https://opensource.org/licenses/MIT.
 */

package com.example.merkurmotor

import android.Manifest
import android.annotation.SuppressLint
import android.app.AlertDialog
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.content.res.Configuration
import android.os.Build
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.ImageButton
import android.widget.SeekBar
import android.widget.Spinner
import android.widget.TextView
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.ActivityResult
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.app.ActivityCompat
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.Locale
import java.util.UUID
import kotlin.math.max
import kotlin.math.min

// BT device MAC (will auto select device of this address)
const val BT_MAC = "BB:E9:6B:E7:D3:A6"
// BT SSP device UUID
const val SSP_UUID = "00001101-0000-1000-8000-00805F9B34FB"
// use SSP_UUID filter for BT devices? (note it did not work on Android 11)
const val BT_FILTER_SSP = false
// BT command timeout [ms]
const val BT_TIMEOUT = 3000L


// -------------------------------------------------------------------------------------------------
// Merkur Motor lib (singleton class - lives independently on activities)
// -------------------------------------------------------------------------------------------------
public object btMotor {

    lateinit var btSocket: BluetoothSocket
    lateinit var btOut: OutputStream
    lateinit var btInput: InputStream
    var speedLim: Double = 1.0

    // try connect to BlueTooth SSP device, open in/out streams
    @SuppressLint("MissingPermission")
    fun Connect(btDev: BluetoothDevice): Int{
        val uuid = UUID.fromString(SSP_UUID)
        btSocket = btDev.createRfcommSocketToServiceRecord(uuid)
        try{
            btSocket.connect()
        } catch(err: IOException){
            //Toast.makeText(this, "Nelze se pripojit k zarizeni!", Toast.LENGTH_LONG).show()
            return(1)
        }
        try {
            this.btOut = btSocket.getOutputStream()
            this.btInput = btSocket.getInputStream()
        }catch (err: IOException){
            //Toast.makeText(this, "Nelze otevrit IO stream!", Toast.LENGTH_LONG).show()
            return(1)
        }
        return(0)
    }

    // disconnect BT device
    fun Disconnect(): Int{

        if(this::btOut.isInitialized)
            this.btOut.close()
        if(this::btInput.isInitialized)
            this.btInput.close()
        if(this::btSocket.isInitialized)
            this.btSocket.close()
        return 0;
    }

    // is device opened?
    fun IsOpened(): Boolean{
        if(!this::btOut.isInitialized || !this::btInput.isInitialized || !this::btSocket.isInitialized)
            return(false)
        try{
            btInput.available()
            //btOut.write("\n".toByteArray())
            if(!btSocket.isConnected)
                return(false)
        }catch (err: Exception){
            return(false)
        }
        return(true)
    }

    // write command, wait for LF-terminated answer if command ends with '?'
    // wait for maximum timeout [ms]
    fun Query(command: String, timeout: Long=0): String {

        if (!IsOpened())
            return ("")

        // default answer
        var ans: String = ""
        var failed: Boolean = false

        // very nasty attempt to handle eventual loss of BT device connection
        try{
            // run InputStream read loop in separate thread so it can be killed
            val thr =Thread {
                try {
                    // flush input stream
                    this.btInput.skip(this.btInput.available().toLong())

                    // is this query command?
                    val isQuery: Boolean = command.last() == '?'

                    // add LF of not present
                    var cmd = command;
                    if (cmd.last() != '\n')
                        cmd = cmd.plus('\n')

                    // send command
                    this.btOut.write(cmd.toByteArray())

                    if(isQuery) {
                        // read answer till LF (excluding)
                        val tRef = System.currentTimeMillis()
                        var done: Boolean = false
                        while (!done) {
                            var tord = this.btInput.available()
                            while (tord-- > 0) {
                                var sym = this.btInput.read().toChar()
                                if (sym == '\n') {
                                    done = true
                                    break
                                }
                                ans += sym.toChar()
                            }
                            if (timeout > 0 && System.currentTimeMillis() - tRef > timeout)
                                throw Exception("SSP Read - timeout!")
                        }
                    }
                }catch(err: Exception) {
                    failed = true;
                }
            }
            thr.start()
            // thread not finishing timeout
            val tRef = System.currentTimeMillis()
            while(System.currentTimeMillis() - tRef < timeout + 100L){
                if(thr.state == Thread.State.TERMINATED)
                    break
                Thread.sleep(10L)
            }
            if(thr.state != Thread.State.TERMINATED)
                thr.interrupt()
            thr.join(100L)

        }catch(err: Exception){
            // timeout - BT device possibly disconnected or failed to response
            failed = true
        }
        if(failed){
            Disconnect()
            // call user callback function
            btErrorCallback()
        }
        return ans
    }

    // Query error callback function
    lateinit var btErrorCallback: ()->Unit

    // --- List of BT device commands ---

    // Get system voltage [V]
    fun GetVmon(timeout: Long=0): Double{
        return Query("SYST:VMON?",timeout).toDouble()
    }

    // Get system temperature [degC]
    fun GetTemp(timeout: Long=0): Double{
        return Query("SYST:TEMP?",timeout).toDouble()
    }

    // set speed [rpm]
    fun SetSpeed(speed: Double,timeout: Long=0): String{
        return Query(String.format(Locale.ROOT,"SPEED %.3f;SYST:ERR?", speed), timeout)
    }

    // Get speed limit [rpm]
    fun GetSpeedLimit(timeout: Long=0): Double{
        return Query("SPEED:LIMIT?",timeout).toDouble()
    }

    // Get current speed [rpm]
    fun GetSpeed(timeout: Long=0): Double{
        return Query("SPEED?",timeout).toDouble()
    }

    // Get last set speed [rpm]
    fun GetSpeedSet(timeout: Long=0): Double{
        return Query("SPEED:SET?",timeout).toDouble()
    }

    // get acceleration const [rpm/s]
    fun GetAcc(timeout: Long=0): Double{
        return Query("MOT:ACC?",timeout).toDouble()
    }

    // set acceleration const [rpm/s]
    fun SetAcc(acceleration: Double,timeout: Long=0): String{
        return Query(String.format(Locale.ROOT,"MOT:ACC %.0f;SYST:ERR?", acceleration), timeout)
    }

    // set speed limit [rpm]
    fun SetSpeedLimit(speed: Double,timeout: Long=0): String{
        return Query(String.format(Locale.ROOT,"SPEED:LIMIT %.0f;SYST:ERR?", speed), timeout)
    }

    // get sleep timeout [s]
    fun GetSleepTimeout(timeout: Long=0): Double{
        return Query("SLEEP:TIMEOUT?",timeout).toDouble()
    }

    // set sleep timeout [s]
    fun SetSleepTimeout(time: Double,timeout: Long=0): String{
        return Query(String.format(Locale.ROOT,"SLEEP:TIMEOUT %.0f;SYST:ERR?", time), timeout)
    }

    // Get *IDN? string
    fun GetIDN(timeout: Long=0): String{
        return Query("*IDN?",timeout)
    }

    // check last error SYST:ERR?
    fun GetSystErr(timeout: Long=0): String{
        return Query("SYST:ERR?",timeout)
    }

    // save setup to EEPROM
    fun SaveEE(timeout: Long=0L): String{
        return Query("SYST:EESAVE;SYST:ERR?",timeout)
    }

    // start motor
    fun Start(timeout: Long=0L): String{
        return Query("START;SYST:ERR?",timeout)
    }

    // stop motor
    fun Stop(timeout: Long=0L): String{
        return Query("STOP;SYST:ERR?",timeout)
    }

    // set direction
    enum class DIRECTION {
        CW, CCW
    }
    fun Direction(dir: DIRECTION,timeout: Long=0L): String{
        if(dir == DIRECTION.CW)
            return Query("DIR CW;SYST:ERR?",timeout)
        else if(dir == DIRECTION.CCW)
            return Query("DIR CCW;SYST:ERR?",timeout)
        else
            return ""
    }

    // sleep controller
    fun Sleep(timeout: Long=0L){
        Query("SLEEP", timeout)
    }

}






// -------------------------------------------------------------------------------------------------
// Main Activity (main panel)
// -------------------------------------------------------------------------------------------------
class MainActivity : ComponentActivity() {

    private lateinit var spinDev: Spinner
    private lateinit var btnConn: Button
    private lateinit var txtIdn: TextView
    private lateinit var btnStart: Button
    private lateinit var btnStop: Button
    private lateinit var btnLeft: Button
    private lateinit var btnRight: Button
    private lateinit var sbSpeed: SeekBar
    private lateinit var txtSpeedDir: TextView
    private lateinit var btnState: Button
    private lateinit var txtVmon: TextView
    private lateinit var txtTemp: TextView
    private lateinit var btnExit: Button
    private lateinit var txtErr: TextView
    private lateinit var btnSleep: Button
    private lateinit var btnCfg: Button

    private val btDevList: MutableSet<BluetoothDevice> = mutableSetOf<BluetoothDevice>()

    // callback function for BT communication error (when BT device does not respond)
    fun onBTerror() {
        guiDisconnect()
        Toast.makeText(this, "Zarizeni neodpovida! Mozna je vyple nebo dosla baterka?", Toast.LENGTH_LONG).show()
    }

    // activity result callback
    private var activityResultLauncher: ActivityResultLauncher<Intent> =
        registerForActivityResult<Intent, ActivityResult>(
            ActivityResultContracts.StartActivityForResult()
        ) { result: ActivityResult ->
            //Toast.makeText(this, "BT zaplej!", Toast.LENGTH_LONG).show()
            if(result.resultCode == RESULT_OK)
                btInit()
        }

    // permission request callback
    private val requestPermissionLauncher =
        registerForActivityResult(
            ActivityResultContracts.RequestPermission()
        ) { isGranted: Boolean ->
            if (isGranted) {
                btInit()
            } else {
                finish()
            }
        }


    // BT initializer
    private fun btInit(){

        // is Android 12 or higher?
        val isA12: Boolean = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S

        // check BT connect permission, eventually ask for permission
        //  note: does not work under Android 12!
        if (!isA12 || ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            //Toast.makeText(this, "Povoleni?", Toast.LENGTH_LONG).show()

            if(!isA12) {
                // legacy version for Android < 12
                requestPermissions(
                    arrayOf(
                        Manifest.permission.BLUETOOTH,
                        Manifest.permission.BLUETOOTH_SCAN,
                        Manifest.permission.BLUETOOTH_CONNECT,
                        Manifest.permission.BLUETOOTH_ADMIN,
                        Manifest.permission.ACCESS_FINE_LOCATION
                    ), 1
                )
            }
            else {
                // newer version for Android >= 12
                requestPermissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT)
                return
            }
        }

        // check BT adapter existence
        val bluetoothManager = this.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val btAdapter = bluetoothManager.adapter
        if(btAdapter == null) {
            // BT adapter not found
            val builder: AlertDialog.Builder = AlertDialog.Builder(this)
            builder
                .setTitle("Bluetooth problem")
                .setMessage("Nenalezen Bluetooth adapter! Bez nej to asi nepojede.")
                .setPositiveButton("OK") { dialog, which ->
                    // Do something.
                    finish()
                }
            val dialog: AlertDialog = builder.create()
            dialog.show()
        }

        // check BT enable state, eventually offer enabling
        if(!btAdapter.isEnabled){
            val intent: Intent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            activityResultLauncher.launch(intent)
        }

        // look for paired devices
        val bondedDevices = btAdapter.bondedDevices
        val list: ArrayList<String> = ArrayList()
        var myDevId: Int = -1
        bondedDevices.forEach{
            // check for SSP BT devices only
            val uuids= it.uuids
            val id = uuids.indexOfFirst { it.toString().uppercase() == SSP_UUID}
            if(id >= 0 || !BT_FILTER_SSP){ /* optional SSP_UUID filter (did not work under Android 12 */
                list.add(it.name.toString() + " (mac: " + it.address.toString() + ")")
                btDevList.add(it)
                if(it.address.toString() == BT_MAC){
                    myDevId = btDevList.count() - 1
                }
            }
        }
        if(btDevList.isEmpty()) {
            // no device found
            list.add(">>> Zadne zarizeni nenalezeno <<<")
            Toast.makeText(this, "Nenalezeno zadne sparovane zarizeni typu SSP!", Toast.LENGTH_LONG).show()
            finish()
        }

        // fill device spinner list
        val adapter: ArrayAdapter<*> =
            ArrayAdapter(this, android.R.layout.simple_spinner_item, list)
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        spinDev.adapter = adapter

        // auto select device if MAC address match found
        if(myDevId >= 0)
            spinDev.setSelection(myDevId)
    }

    // try open BT and init device
    private fun btOpen(btDev: BluetoothDevice){

        txtIdn.text = "device *IDN?"

        // try connect to BT
        if(!btMotor.IsOpened() && btMotor.Connect(btDev) > 0){
            throw Exception("Opening BT device failed!")
        }

        // needed before any traffic to device because BT module produces some debug data during connect and confuses controller
        Thread.sleep(500L)

        // throw away possible device errors coming from BT module prior connection
        btMotor.GetSystErr(BT_TIMEOUT)

        // get device *IDN? string
        txtIdn.text = btMotor.GetIDN(BT_TIMEOUT)

        // get maximum speed [rpm]
        btMotor.speedLim = btMotor.GetSpeedLimit(BT_TIMEOUT)

        // set initial speed for seeker
        setSpeedSeeker(btMotor.GetSpeed(BT_TIMEOUT))
    }


    // On main activity creation (called also when devices switches orientation
    @SuppressLint("MissingPermission", "ServiceCast")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        //enableEdgeToEdge() // don't like this because it's rendering under system bars

        // select view orientation, load appropriate layout
        if(this.resources.configuration.orientation == Configuration.ORIENTATION_LANDSCAPE)
            setContentView(R.layout.layout_land)
        else if(this.resources.configuration.orientation == Configuration.ORIENTATION_PORTRAIT)
            setContentView(R.layout.layout)
        else {
            // unknown display orientation
            Toast.makeText(
                this,
                "Neznama orientace zobrazovky?",
                Toast.LENGTH_LONG
            ).show()
            finish()
        }

        // get UI item refs
        spinDev = findViewById(R.id.spinDev)
        btnConn = findViewById(R.id.btnConnect)
        txtIdn = findViewById(R.id.txtIDN)
        btnStart = findViewById(R.id.btnStart)
        btnStop = findViewById(R.id.btnStop)
        btnLeft = findViewById(R.id.btnLeft)
        btnRight = findViewById(R.id.btnRight)
        sbSpeed = findViewById(R.id.sbSpeed)
        txtSpeedDir = findViewById(R.id.txtSpeedDir)
        btnState = findViewById(R.id.btnGetState)
        txtVmon = findViewById(R.id.txtVmon)
        txtTemp = findViewById(R.id.txtTemp)
        btnExit = findViewById(R.id.btnExit)
        txtErr = findViewById(R.id.txtErr)
        btnSleep = findViewById(R.id.btnSleep)
        btnCfg = findViewById(R.id.btnCfg)

        // init GUI stuff
        guiDisconnect()

        // init BT stuff
        btInit()

        // try open BT device
        if(btMotor.IsOpened()){
            val btDev = btDevList.elementAt(spinDev.selectedItemId.toInt())
            try {
                btOpen(btDev)
                guiConnect()
            } catch (err: Exception) {
                // failed
                Toast.makeText(
                    this,
                    "Pripojeni k zarizeni selhalo! Mozna je vypnute?",
                    Toast.LENGTH_LONG
                ).show()
            }
        }

        // BT comm error handling callback (called when Query() fails)
        btMotor.btErrorCallback = this::onBTerror

        // show config panel?
        btnCfg.setOnClickListener {
            val cfgintent = Intent(this, CfgActivity::class.java)
            startActivity(cfgintent)
        }

        // connect/disconnect BT device?
        btnConn.setOnClickListener {
            if(btMotor.IsOpened()){
                // disconnect
                btMotor.Disconnect()
                guiDisconnect()
            }else {
                // connect
                val btDev = btDevList.elementAt(spinDev.selectedItemId.toInt())
                try {
                    btOpen(btDev)
                    guiConnect()
                } catch (err: Exception) {
                    // failed
                    Toast.makeText(
                        this,
                        "Pripojeni k zarizeni selhalo! Mozna je vypnute?",
                        Toast.LENGTH_LONG
                    ).show()
                }
            }
        }

        // start motor
        btnStart.setOnClickListener {
            txtErr.text = btMotor.Start(BT_TIMEOUT)
            setSpeedSeeker(btMotor.GetSpeedSet(BT_TIMEOUT))
        }

        // stop motor
        btnStop.setOnClickListener {
            txtErr.text = btMotor.Stop(BT_TIMEOUT)
        }

        // start left
        btnLeft.setOnClickListener {
            txtErr.text = btMotor.Direction(btMotor.DIRECTION.CCW, BT_TIMEOUT)
            setSpeedSeeker(btMotor.GetSpeedSet(BT_TIMEOUT))
        }

        // start right
        btnRight.setOnClickListener {
            txtErr.text = btMotor.Direction(btMotor.DIRECTION.CW, BT_TIMEOUT)
            setSpeedSeeker(btMotor.GetSpeedSet(BT_TIMEOUT))
        }

        // speed dial
        sbSpeed.setOnSeekBarChangeListener(
            object : SeekBar.OnSeekBarChangeListener {

                var tRefSpeed: Long = 0L

                // Seek position to speed [rpm]
                fun toSpeed(seek: SeekBar): Double {
                    val rel = (2.0*seek.progress.toDouble() - seek.max.toDouble())/seek.max.toDouble()
                    return btMotor.speedLim*rel
                }

                // update speed
                fun setSpeed(seek: SeekBar){
                    val speed: Double = toSpeed(seek)
                    txtErr.text = btMotor.SetSpeed(speed, BT_TIMEOUT)
                    txtSpeedDir.text = String.format("Rychlost a Smer (%.0f ot/min):", speed)
                }

                // Handle when the progress changes
                override fun onProgressChanged(seek: SeekBar,
                                               progress: Int, fromUser: Boolean) {

                    if(!fromUser)
                        return
                    if(System.currentTimeMillis() - tRefSpeed > 200L) {
                        // limit update rate to not hangup the app
                        tRefSpeed = System.currentTimeMillis()
                        setSpeed(seek)
                    }
                }

                // Handle when the user starts tracking touch
                override fun onStartTrackingTouch(seek: SeekBar) {
                    tRefSpeed = System.currentTimeMillis()
                }

                // Handle when the user stops tracking touch
                override fun onStopTrackingTouch(seek: SeekBar) {
                    setSpeed(seek)
                }
            }
        )

        // check system state values
        btnState.setOnClickListener {
            txtVmon.text = String.format(Locale.ROOT, "Napeti: %.2f V", btMotor.GetVmon(BT_TIMEOUT))
            txtTemp.text = String.format(Locale.ROOT, "Teplota: %.0f \"C", btMotor.GetTemp(BT_TIMEOUT))
            setSpeedSeeker(btMotor.GetSpeed(BT_TIMEOUT))
        }

        // exit anctivity activity
        btnExit.setOnClickListener {
            btMotor.Disconnect()
            guiDisconnect()
            finish()
        }

        // set device to sleep
        btnSleep.setOnClickListener {
            // sleep now
            btMotor.Sleep(BT_TIMEOUT)

            // wait a bit because BT module sends stuff to controller and wakes it up again
            Thread.sleep(1500L)

            // disconnect BT
            btMotor.Disconnect()
            guiDisconnect()
        }

    }

    // activity exit
    override fun onDestroy() {
        //btMotor.Disconnect()
        //guiDisconnect()
        super.onDestroy()
    }

    // set speed/dir seeker by speed value
    fun setSpeedSeeker(speed: Double){
        sbSpeed.progress = min(max((sbSpeed.max.toDouble()*(0.5 + 0.5*speed/btMotor.speedLim)).toInt(),0),sbSpeed.max)
        txtSpeedDir.text = String.format("Rychlost a Smer (%.0f ot/min):", speed)
    }

    // set GUI elements to disconnected state
    fun guiDisconnect(){
        btnConn.text = "Pripojit zarizeni"
        txtIdn.text = "device *IDN?"
        txtErr.text = "device SYST:ERR?"
        btnLeft.isEnabled = false
        btnRight.isEnabled = false
        btnStart.isEnabled = false
        btnStop.isEnabled = false
        sbSpeed.isEnabled = false
        btnState.isEnabled = false
        btnSleep.isEnabled = false
        btnCfg.isEnabled = false
    }

    // set GUI elements to connected state
    fun guiConnect(){
        btnConn.text = "Odpojit zarizeni"
        btnLeft.isEnabled = true
        btnRight.isEnabled = true
        btnStart.isEnabled = true
        btnStop.isEnabled = true
        sbSpeed.isEnabled = true
        btnState.isEnabled = true
        btnSleep.isEnabled = true
        btnCfg.isEnabled = true
    }

    // -------------------------------------------------------------------------------------------------
    // Config Activity (config panel)
    // -------------------------------------------------------------------------------------------------
    class CfgActivity : ComponentActivity() {

        // config item
        class CfgItem(
            val label: String,
            val default: Double,
            val fget: (Long)->Double,
            val fset: (Double,Long)->String){
        }

        // recycler view adapter
        class CustomAdapter(private val dataSet: Array<CfgItem>, val txtError: TextView) :
            RecyclerView.Adapter<CustomAdapter.ViewHolder>() {

            /**
             * Provide a reference to the type of views that you are using
             * (custom ViewHolder)
             */
            class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
                val txtLabel: TextView
                val editValue: EditText
                val btnStore: ImageButton
                val btnLoad: ImageButton
                val btnDefault: ImageButton
                init {
                    txtLabel = view.findViewById(R.id.txtLabel)
                    editValue = view.findViewById(R.id.editValue)
                    btnStore = view.findViewById(R.id.ibtnStore)
                    btnLoad = view.findViewById(R.id.ibtnLoad)
                    btnDefault = view.findViewById(R.id.ibtnDefault)
                }
            }

            // Create new views (invoked by the layout manager)
            override fun onCreateViewHolder(viewGroup: ViewGroup, viewType: Int): ViewHolder {
                // Create a new view, which defines the UI of the list item
                val view = LayoutInflater.from(viewGroup.context)
                    .inflate(R.layout.cfg_item, viewGroup, false)
                return ViewHolder(view)
            }

            // number to edit
            fun num2edit(edit: EditText, num: Double){
                edit.setText(String.format(Locale.ROOT,"%.0f",num))
            }

            // Replace the contents of a view (invoked by the layout manager)
            override fun onBindViewHolder(viewHolder: ViewHolder, position: Int) {

                // Get element from your dataset at this position and replace the
                // contents of the view with that element
                val item = dataSet[position]

                // label
                viewHolder.txtLabel.text = item.label
                num2edit(viewHolder.editValue,item.fget(BT_TIMEOUT))

                // on default click
                viewHolder.btnDefault.setOnClickListener {
                    num2edit(viewHolder.editValue, item.default)
                }

                // on write click
                viewHolder.btnLoad.setOnClickListener{
                    num2edit(viewHolder.editValue,item.fget(BT_TIMEOUT))
                }

                // on reload click
                viewHolder.btnStore.setOnClickListener {
                    txtError.text = item.fset(viewHolder.editValue.text.toString().toDouble(), BT_TIMEOUT)
                }
            }

            // Return the size of your dataset (invoked by the layout manager)
            override fun getItemCount() = dataSet.size
        }

        // create config activity
        override fun onCreate(savedInstanceState: Bundle?) {
            super.onCreate(savedInstanceState)

            // select layout
            if(this.resources.configuration.orientation == Configuration.ORIENTATION_LANDSCAPE)
                setContentView(R.layout.layout_cfg_recycler_land)
            else if(this.resources.configuration.orientation == Configuration.ORIENTATION_PORTRAIT)
                setContentView(R.layout.layout_cfg_recycler)
            else {
                // unknown orientation
                Toast.makeText(
                    this,
                    "Neznama orientace zobrazovky?",
                    Toast.LENGTH_LONG
                ).show()
                finish()
            }

            val txtErr: TextView = findViewById(R.id.txtCfgErr)
            txtErr.text = "SYST:ERR?"

            // recycler items menu
            val dataset: Array<CfgItem> = arrayOf(
                CfgItem("Cas do uspani [s]:", 60.0, btMotor::GetSleepTimeout, btMotor::SetSleepTimeout),
                CfgItem("Maximalni rychlost [ot/min]:", 270.0, btMotor::GetSpeedLimit, btMotor::SetSpeedLimit),
                CfgItem("Akcelerace [ot/min/s]:", 700.0, btMotor::GetAcc, btMotor::SetAcc)
            )
            val customAdapter = CustomAdapter(dataset, txtErr)
            val recyclerView: RecyclerView = findViewById(R.id.recyclerCfg)
            recyclerView.layoutManager = LinearLayoutManager(this)
            recyclerView.adapter = customAdapter

            // panel exit?
            val btnExitCfg: Button = findViewById(R.id.btnExitCfg)
            btnExitCfg.setOnClickListener {
                finish()
            }

            // write to EEPROM?
            val btnSave: Button = findViewById(R.id.btnSave)
            btnSave.setOnClickListener {
                btMotor.SaveEE(BT_TIMEOUT)
            }
        }

        // activity exit
        override fun onDestroy() {
            try {
                btMotor.speedLim = btMotor.GetSpeedLimit(BT_TIMEOUT)
            }catch(err: Exception){

            }
            super.onDestroy()
        }
    }

}



