var ros = new ROSLIB.Ros();

ros.on("error", function (error) {
  document.getElementById("connecting").style.display = "none";
  document.getElementById("connected").style.display = "none";
  document.getElementById("closed").style.display = "none";
  document.getElementById("error").style.display = "inline";
  console.log(error);
});

ros.on("connection", function () {
  console.log("Connection made!");
  document.getElementById("connecting").style.display = "none";
  document.getElementById("error").style.display = "none";
  document.getElementById("closed").style.display = "none";
  document.getElementById("connected").style.display = "inline";
});

ros.on("close", function () {
  console.log("Connection closed.");
  document.getElementById("connecting").style.display = "none";
  document.getElementById("connected").style.display = "none";
  document.getElementById("closed").style.display = "inline";
});

ros.connect("ws://localhost:9090");

var listener = new ROSLIB.Topic({
  ros: ros,
  name: "/robot/all",
  messageType: "std_msgs/String",
});

listener.subscribe(function (message) {
  console.log("Received message on " + listener.name + ": " + message.data);

  let data = message.data.replace(/[\[\]]/g, "").split(",");

  let sensorData = data.map(function (item) {
    return parseInt(item);
  });

  chart.data.datasets[0].data = sensorData;
  chart.update();
  document.getElementById("last_message").innerText =
    "Last Reading: " + message.data.replace(/[\[\]]/g, "");
});

let ctx = document.getElementById("canvas");
ctx = ctx.getContext("2d");
let chart = new Chart(ctx, {
  type: "bar",
  data: {
    labels: ["Sensor Left", "Sensor Center", "Sensor Right"],
    datasets: [
      {
        label: "Ultrasonic Range Data (cm)",
        data: [0, 0, 0],
        backgroundColor: [
          "rgba(255, 99, 132, 0.2)",
          "rgba(54, 162, 235, 0.2)",
          "rgba(255, 206, 86, 0.2)",
        ],
        borderColor: [
          "rgba(255, 99, 132, 1)",
          "rgba(54, 162, 235, 1)",
          "rgba(255, 206, 86, 1)",
        ],
        borderWidth: 1,
      },
    ],
  },
  options: {
    scales: {
      y: {
        beginAtZero: true,
      },
    },
  },
});

function action(action) {
  let actionTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/robot/control",
    messageType: "std_msgs/String",
  });

  let msg = new ROSLIB.Message({
    data: action,
  });

  actionTopic.publish(msg);
}
