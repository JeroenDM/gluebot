// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({
  url: "ws://localhost:9090"
});

ros.on("connection", function() {
  console.log("Connected to websocket server.");
});

ros.on("error", function(error) {
  console.log("Error connecting to websocket server: ", error);
});

ros.on("close", function() {
  console.log("Connection to websocket server closed.");
});

// Calling a service
// -----------------

var planPathClient = new ROSLIB.Service({
  ros: ros,
  name: "/plan_path",
  serviceType: "std_srvs/Trigger"
});

function planPath() {
  var request = new ROSLIB.ServiceRequest({});

  planPathClient.callService(request, function(result) {
    console.log(
        planPathClient.name +
        ": " +
        result.success +
        " Info: " +
        result.message
    );
    document.getElementById("planPath").innerHTML = result.message;
  });
}

var showPathClient = new ROSLIB.Service({
  ros: ros,
  name: "/show_path",
  serviceType: "std_srvs/Trigger"
});

function showPath() {
  var request = new ROSLIB.ServiceRequest({});

  showPathClient.callService(request, function(result) {
    console.log(
        planPathClient.name +
        ": " +
        result.success +
        " Info: " +
        result.message
    );
  });
}

var executePathService = new ROSLIB.Service({
  ros: ros,
  name: "/execute_path",
  serviceType: "std_srvs/Trigger"
});

function executePath() {
  var request = new ROSLIB.ServiceRequest({});

  executePathService.callService(request, function(result) {
    console.log(
        executePathService.name +
        ": " +
        result.success +
        " Info: " +
        result.message
    );
    document.getElementById("executePath").innerHTML = result.message;
  });
}

var moveHomeClient = new ROSLIB.Service({
  ros: ros,
  name: "/move_home",
  serviceType: "std_srvs/Trigger"
});

function moveHome() {
  var request = new ROSLIB.ServiceRequest({});

  moveHomeClient.callService(request, function(result) {
    console.log(
        moveHomeClient.name +
        ": " +
        result.success +
        " Info: " +
        result.message
    );
    document.getElementById("moveHome").innerHTML = result.message;
  });
}


var setGlueGunClient = new ROSLIB.Service({
  ros: ros,
  name: "/set_glue_gun",
  serviceType: "std_srvs/SetBool"
});

var GLUEGUN_STATE = false;

function setGlueGun() {
  var request;

  if (GLUEGUN_STATE) {
    request = new ROSLIB.ServiceRequest({
      data: false
    });

    setGlueGunClient.callService(request, function(result) {
      console.log(
          setGlueGunClient.name +
          ": " +
          result.success +
          " Info: " +
          result.message
      );
    });

    document.getElementById("setGlueGunButton").setAttribute("class", "btn btn-danger");
    GLUEGUN_STATE = false;
  }
  else {
    request = new ROSLIB.ServiceRequest({
      data: true
    });

    setGlueGunClient.callService(request, function(result) {
      console.log(
          setGlueGunClient.name +
          ": " +
          result.success +
          " Info: " +
          result.message
      );
    });

    document.getElementById("setGlueGunButton").setAttribute("class", "btn btn-success");

    GLUEGUN_STATE = true;
  }
}