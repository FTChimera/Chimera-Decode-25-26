{
  "startPoint": {
    "x": 128.13,
    "y": 112,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180
  },
  "lines": [
    {
      "id": "mk8xc9t6-e3juwv",
      "name": "Launch",
      "endPoint": {
        "x": 110,
        "y": 110,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 45
      },
      "controlPoints": [],
      "color": "#6DBBB9",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkvwjfs0-ubq43s",
      "name": "StartPose",
      "endPoint": {
        "x": 123.5,
        "y": 122.8,
        "heading": "linear",
        "reverse": false,
        "startDeg": 45,
        "endDeg": 37.5
      },
      "controlPoints": [],
      "color": "#5BCACA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [],
  "sequence": [
    {
      "kind": "path",
      "lineId": "mk8xc9t6-e3juwv"
    },
    {
      "kind": "path",
      "lineId": "mkvwjfs0-ubq43s"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-01-27T01:13:29.511Z"
}