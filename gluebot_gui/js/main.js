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
      "Result for service call on " +
        planPathClient.name +
        ": " +
        result.success +
        " Info: " +
        result.message
    );
    document.getElementById("planPath").innerHTML = result.message;
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
      "Result for service call on " +
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
      "Result for service call on " +
        moveHomeClient.name +
        ": " +
        result.success +
        " Info: " +
        result.message
    );
    document.getElementById("moveHome").innerHTML = result.message;
  });
}
