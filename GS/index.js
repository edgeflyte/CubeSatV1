const { app, BrowserWindow } = require("electron");
const path = require("node:path");
const { SerialPort } = require("serialport");
const express = require("express");
const ea = express();
const http = require("http");
const server = http.createServer(ea);
const { Server } = require("socket.io");
const io = new Server(server);
const { ReadlineParser } = require("@serialport/parser-readline");



var payloads = [];


var sys = {
    gsConnected: false,
    csConnected: false,

};

var gsSerialPort = new SerialPort({ path: "COM", baudRate: 9600, autoOpen: false});
var gsSerialLine = gsSerialPort.pipe(new ReadlineParser({ delimiter: "\r\n" }));

var csSerialPort = new SerialPort({ path: "COM", baudRate: 9600, autoOpen: false});
var csSerialLine = csSerialPort.pipe(new ReadlineParser({ delimiter: "\r\n" }));


const createWindow = () => {
  const mainWindow = new BrowserWindow({
    width: 1050,
    height: 650,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
    },
    titleBarStyle: "hidden",
    titleBarOverlay: {
      color: "#213f98",
      symbolColor: "#FFF",
      height: 15,
    }, 
  });

  mainWindow.loadFile("main.html");
};


app.whenReady().then(() => {
  createWindow();
  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});







ea.get("/", (req, res) => {
  res.sendFile(__dirname + "/index.html");
});

io.on("connection", (socket) => {
  console.log("a user connected");


  socket.on("pollGSCom", (e)=>{
      // Poll available 
      SerialPort.list().then(function (data) {
        console.log(data);
          io.emit("gsCom", data);
      });
  });


  socket.on("pollCSCom", (e) => {
    // Poll available
    SerialPort.list().then(function (data) {
      io.emit("csCom", []);
    });
  });


  socket.on("comCS_CN", (port)=>{
    console.log("Connecting CS");
    csSerial(port, 1);
  });

  socket.on("comGS_CN", (port)=>{
    console.log("Connecting GS");
    gsSerial(port, 1);
  });

  socket.on("comDC", (dev)=>{
    if(dev == "gs") gsSerial("", 0);
    if (dev == "cs") csSerial("", 0);
  });


  socket.on("toggleBMSCH", (r)=>{
    // TODO: Send Packet to CS
    // Format: {id: CSID, ch: LDOCH 1-4}
    console.log(r);
  });

});




setInterval(()=>{
    io.emit("sys", sys);
}, 1000);




function gsSerial(port, con){
    if(!con){
        //disconnect port
        try {
          gsSerialPort.drain(() => console.log("DONE"));
          gsSerialPort._disconnected(()=> console.log("Error Disconnecting GS"));
          gsSerialPort.destroy();
        } catch (error) {
          
        }

        return;
    }

    // Connect to GS Serial Port
    console.log(port)

    gsSerialPort = new SerialPort({
      path: port,
      baudRate: 9600,
      autoOpen: true
    }, function (err) {
      if (err) { return console.log("Error opening port: ", err.message);}
      console.log("GS Connected")
      sys.gsConnected = true;
    });

    gsSerialLine = gsSerialPort.pipe(new ReadlineParser({ delimiter: "\n" }));

    gsSerialPort.on("close", function () {
      console.log("GS Serial Disconnected");
      sys.gsConnected = false;
    });

    gsSerialLine.on("data", (dt) => {
      parseRawGS(dt);
    });



}

function csSerial(port, con){
    if(!con){
      //disconnect port
      return;
    }





}

csSerialLine.on("data", console.log);





csSerialPort.on("open", function () {
  console.log("CS Serial Connected");
  sys.csConnected = true;
});
csSerialPort.on("close", function () {
  console.log("CS Serial Disconnected");
  sys.csConnected = false;
});




function parseRawGS(line){
  // console.log(line);
  io.emit("rpkt", line);

  if(!line.startsWith("#")) return;   // Probably a bad packet
  // if(!line.endsWith("*")) return;

  io.emit("valid_pkt", line); // For plugin support

  var d = line.split(",");

  d[1] = d[1].substring(0, 2);

  var ind = payloads.findIndex(function(item, i){
    return item.id == d[1];
  });


  if(ind == -1){
    // new payload
    payloads.push({id: d[1], dt: Date.now()});

    ind = payloads.findIndex(function (item, i) {
      return item.id == d[1];
    });
  }

  switch (d[0]) {
    case "#A":
      payloads[ind].lat = parseFloat(d[2]);
      payloads[ind].lng = parseFloat(d[3]);
      // ind.alt = parseFloat(d[4]);

      break;
    case "#B":
      payloads[ind].alt = parseFloat(d[2]);
      payloads[ind].sats = parseInt(d[3]);
      payloads[ind].speed = parseFloat(d[4]);
      payloads[ind].angle = parseFloat(d[5]);
      break;
    case "#C":
      payloads[ind].t1 = parseFloat(d[2]);
      payloads[ind].t2 = parseFloat(d[3]);
      payloads[ind].rh = parseFloat(d[4]);
      break;
    case "#D":
      payloads[ind].ax = parseFloat(d[2]);
      payloads[ind].ay = parseFloat(d[3]);
      payloads[ind].az = parseFloat(d[4]);
      break;
    case "#E":
      payloads[ind].gx = parseFloat(d[2]);
      payloads[ind].gy = parseFloat(d[3]);
      payloads[ind].gz = parseFloat(d[4]);
      break;
    case "#F":
      payloads[ind].mx = parseFloat(d[2]);
      payloads[ind].my = parseFloat(d[3]);
      payloads[ind].mz = parseFloat(d[4]);
      break;
    case "#G":
      payloads[ind].bms_ch1 = parseInt(d[2]);
      payloads[ind].bms_ch2 = parseInt(d[3]);
      payloads[ind].bms_ch3 = parseInt(d[4]);
      payloads[ind].bms_ch4 = parseInt(d[5]);
      payloads[ind].batV = parseFloat(d[6]);
      payloads[ind].batPct = parseFloat(d[7]);
      break;
    case "#H":
      payloads[ind].sol1V = parseFloat(d[2]);
      payloads[ind].sol2V = parseFloat(d[3]);
      payloads[ind].bms_err = parseInt(d[4]);
      break;
    case "#I":
      payloads[ind].qw = parseFloat(d[2]);
      payloads[ind].qx = parseFloat(d[3]);
      payloads[ind].qy = parseFloat(d[4]);
      payloads[ind].qz = parseFloat(d[5]);

      var q = {
        w: payloads[ind].qw,
        x: payloads[ind].qx,
        y: payloads[ind].qy,
        z: payloads[ind].qz,
      };

      payloads[ind].yaw = Math.atan2(
        2.0 * (q.y * q.z + q.w * q.x),
        q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z
      );
      payloads[ind].pitch = Math.asin(-2.0 * (q.x * q.z - q.w * q.y));
      payloads[ind].roll = Math.atan2(
        2.0 * (q.x * q.y + q.w * q.z),
        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z
      );

      break;

    case "#J":
      payloads[ind].asb_t1 = parseFloat(d[2]);
      payloads[ind].asb_t2 = parseFloat(d[3]);
      payloads[ind].asb_rh = parseFloat(d[4]);
      payloads[ind].asb_p1 = parseFloat(d[5]);
      break;

    case "#K":
      payloads[ind].asb_pa1 = parseFloat(d[2]);
      payloads[ind].asb_aqi = parseInt(d[3]);
      payloads[ind].asb_tvoc = parseInt(d[4]);
      payloads[ind].asb_eco2 = parseInt(d[5]);
      break;

    case "#L":
      payloads[ind].asb_co2 = parseInt(d[2]);
      payloads[ind].asb_typ = parseFloat(d[3]);
      break;

    case "#M":
      payloads[ind].asb_mc10 = parseFloat(d[2]);
      payloads[ind].asb_mc25 = parseFloat(d[3]);
      payloads[ind].asb_mc40 = parseFloat(d[4]);
      break;
    case "#N":
      payloads[ind].asb_mc100 = parseFloat(d[2]);
      payloads[ind].asb_nc05 = parseFloat(d[3]);
      payloads[ind].asb_nc10 = parseFloat(d[4]);
      break;

    case "#O":
      payloads[ind].asb_nc25 = parseFloat(d[2]);
      payloads[ind].asb_nc40 = parseFloat(d[3]);
      payloads[ind].asb_nc100 = parseFloat(d[4]);
      break;

    default:
      console.log(`invalid packet ${line}`);
      break;
  }

  // console.log(payloads[ind]);
  io.emit("plPkt", payloads[ind]);
}



setInterval(()=>{
   io.emit("plds", payloads);
}, 1000);


setInterval(()=>{
  console.log(payloads)
}, 5000);




server.listen(45460, () => {
  console.log("listening on *:45460");
});